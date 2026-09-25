/*
 * Cycle-exact link transmit kernel.
 *
 * Sends one unit (see protocol/link.h) on this core's 8-bit dedicated GPIO
 * port. Every instruction after the marker takes exactly one cycle and every
 * byte is held for two cycles, except where noted, so the FPGA receiver can
 * sample each byte at a fixed offset from the marker's falling edge. Any
 * change here must be mirrored in fpga/rtl/link_lane.v.
 *
 *   marker     0xFF for 240 cycles, then 0x00
 *   header     8 bytes, 2 cycles each
 *   groups     40 bytes in 91 cycles: 16 words x 2 bytes, then 8 nibble bytes
 *   fragments  up to four (head, bulk, wrapped bulk, tail), each a run of
 *              groups; moving to the next fragment costs a fixed gap
 *   trailer    checksum and end marker, 4 bytes each
 *
 * Register use inside the kernel:
 *   a2         descriptor (see capture.c: tx_descriptor)
 *   a3, a4     current fragment: source address, groups remaining
 *   QACC_L/H   the four fragment descriptors, loaded once
 *   q6 lane 1  index of the current fragment
 *   q4, q5     16-bit weights for the running checksum (ACCX)
 *   q7         0x0F nibble mask
 *   q0..q3     the group being prepared: loads and unzips of the next group
 *              are interleaved with the byte writes of the current one
 *
 * The checksum is accumulated in ACCX by multiply-accumulate instructions
 * fused with the loads, so it costs no extra cycles. Its largest possible
 * value stays below 2^31, so the 40-bit saturating accumulator never clips.
 *
 * Assembled twice: TX_NAME/TX_SECTION select the lane's copy, which the
 * linker script places at a fixed address in that core's code bank.
 */
#include "link.h"

#ifndef TX_NAME
#define TX_NAME transmit_lane0
#endif
#ifndef TX_SECTION
#define TX_SECTION .tx_lane0
#endif

.section TX_SECTION, "ax", @progbits

/* Write the four bytes of \reg, least significant first. */
.macro SEND_WORD reg
    wur.gpio_out \reg
    srli \reg, \reg, 8
    wur.gpio_out \reg
    srli \reg, \reg, 8
    wur.gpio_out \reg
    srli \reg, \reg, 8
    wur.gpio_out \reg
.endm

/* Start a fragment: arm the hardware loop for its groups after the first,
 * then load and unzip its first group into q0 (low halves) / q2 / q1. */
.macro PRIME_FRAGMENT
    movi a5, 0
    wsr.lcount a5
    addi a4, a4, -1
    beqz a4, .Lprime_vectors\@
    addi a5, a4, -1
    wsr.lcount a5
.Lprime_vectors\@:
    rsync
    ee.vld.128.ip q0, a3, 16
    ee.vld.128.ip q1, a3, 16
    ee.vld.128.ip q2, a3, 16
    ee.vld.128.ip q3, a3, 16
    ee.vunzip.16 q0, q1
    ee.vunzip.16 q2, q3
    ee.vunzip.8 q1, q3
    ee.andq q1, q1, q7
    ee.vunzip.8 q1, q3
    ee.vsl.32 q3, q3
    ee.orq q1, q1, q3
.endm

/* Second-cycle filler of a word's last byte: prepares the next group, or in
 * the final group of a fragment is a NOP (hold=1) or nothing (hold=0). */
.macro PREPARE_NEXT hold, op:vararg
    .if final_group
        .if \hold
            _nop
        .endif
    .else
        \op
    .endif
.endm

/* Checksum \payload with \weights; outside the final group also load the
 * next group's quarter into \target. */
.macro CHECKSUM_LOAD target, payload, weights
    .if final_group
        ee.vmulas.u16.accx \payload, \weights
    .else
        ee.vmulas.u16.accx.ld.ip \target, a3, 16, \payload, \weights
    .endif
.endm

/* One 40-byte group: 16 low-half words from q0/q2, 8 nibble bytes from q1. */
.macro SEND_GROUP
    ee.movi.32.a q0, a5, 0
    ee.movi.32.a q0, a6, 1
    ee.movi.32.a q0, a7, 2
    ee.movi.32.a q0, a8, 3
    ee.movi.32.a q2, a9, 0
    ee.movi.32.a q2, a10, 1
    ee.movi.32.a q2, a11, 2
    ee.movi.32.a q2, a12, 3
    ee.movi.32.a q1, a13, 0
    ee.movi.32.a q1, a14, 1
    SEND_WORD a5
    CHECKSUM_LOAD q0, q0, q4
    SEND_WORD a6
    CHECKSUM_LOAD q1, q1, q5
    SEND_WORD a7
    CHECKSUM_LOAD q2, q2, q4
    SEND_WORD a8
    PREPARE_NEXT 1, ee.vld.128.ip q3, a3, 16
    SEND_WORD a9
    PREPARE_NEXT 1, ee.vunzip.16 q0, q1
    SEND_WORD a10
    PREPARE_NEXT 1, ee.vunzip.16 q2, q3
    SEND_WORD a11
    PREPARE_NEXT 1, ee.vunzip.8 q1, q3
    SEND_WORD a12
    PREPARE_NEXT 1, ee.andq q1, q1, q7
    SEND_WORD a13
    PREPARE_NEXT 1, ee.vunzip.8 q1, q3
    SEND_WORD a14
    PREPARE_NEXT 0, ee.vsl.32 q3, q3
    PREPARE_NEXT 0, ee.orq q1, q1, q3
.endm

    .align 4
    .global TX_NAME
    .type TX_NAME, @function
TX_NAME:
    entry a1, 32
    movi a5, 0
    wsr.lcount a5
    movi a5, .Lgroup
    wsr.lbeg a5
    movi a5, .Lfinal_group
    wsr.lend a5
    rsync

    /* Fragment descriptors into QACC; end marker (XOR its mask) into QACC_L_4. */
    addi a3, a2, 16
    ee.ld.qacc_l.l.128.ip a3, 16
    ee.ld.qacc_h.l.128.ip a3, 16
    movi a5, LINK_END_MARKER
    l32i a7, a2, 4
    xor a5, a5, a7
    wur.qacc_l_4 a5

    /* Checksum weights: 1 for every 16-bit lane of the 32 low-half bytes and
     * the 8 nibble bytes, 0 for the unused upper half of q5. */
    movi a10, 0x00010001
    ee.movi.32.q q4, a10, 0
    ee.movi.32.q q4, a10, 1
    ee.movi.32.q q4, a10, 2
    ee.movi.32.q q4, a10, 3
    ee.movi.32.q q5, a10, 0
    ee.movi.32.q q5, a10, 1
    movi a5, 0
    ee.movi.32.q q5, a5, 2
    ee.movi.32.q q5, a5, 3
    movi a5, 0
    ee.movi.32.q q6, a5, 1
    movi a10, 0x0f0f0f0f
    ee.movi.32.q q7, a10, 0
    ee.movi.32.q q7, a10, 1
    ee.movi.32.q q7, a10, 2
    ee.movi.32.q q7, a10, 3
    ssai 4

    /* Find the first non-empty fragment. */
.Lfind_first:
    ee.movi.32.a q6, a5, 1
    beqi a5, 0, .Lfirst0
    beqi a5, 1, .Lfirst1
    beqi a5, 2, .Lfirst2
    rur.qacc_h_2 a3
    rur.qacc_h_3 a4
    j .Lfirst_loaded
.Lfirst0:
    rur.qacc_l_0 a3
    rur.qacc_l_1 a4
    j .Lfirst_loaded
.Lfirst1:
    rur.qacc_l_2 a3
    rur.qacc_l_3 a4
    j .Lfirst_loaded
.Lfirst2:
    rur.qacc_h_0 a3
    rur.qacc_h_1 a4
.Lfirst_loaded:
    bnez a4, .Lfirst_prime
    ee.movi.32.a q6, a5, 1
    addi a5, a5, 1
    ee.movi.32.q q6, a5, 1
    bnei a5, 4, .Lfind_first
    retw.n

.Lfirst_prime:
    PRIME_FRAGMENT
    /* Checksum seed: the header's four 16-bit words, XOR the seed mask. */
    l32i a5, a2, 8
    l32i a6, a2, 12
    extui a15, a5, 0, 16
    srli a7, a5, 16
    add a15, a15, a7
    extui a7, a6, 0, 16
    add a15, a15, a7
    srli a7, a6, 16
    add a15, a15, a7
    l32i a7, a2, 0
    xor a15, a15, a7
    wur.accx_0 a15
    movi a7, 0
    wur.accx_1 a7

    /* Marker: 0xFF for 240 cycles, then the header. */
    movi a15, 255
    wur.gpio_out a15
    .rept 238
    _nop
    .endr
    _movi a15, 0
    wur.gpio_out a15
    _nop
    .set final_group, 0
    SEND_WORD a5
    _nop
    SEND_WORD a6
    beqz a4, .Lfinal_group
    j .Lgroup

    .p2align 4
.Lnext_fragment:
    ee.movi.32.a q6, a5, 1
    beqi a5, 1, .Lfragment1
    beqi a5, 2, .Lfragment2
    rur.qacc_h_2 a3
    rur.qacc_h_3 a4
    j .Lfragment_loaded
.Lfragment1:
    rur.qacc_l_2 a3
    rur.qacc_l_3 a4
    j .Lfragment_loaded
.Lfragment2:
    rur.qacc_h_0 a3
    rur.qacc_h_1 a4
.Lfragment_loaded:
    beqz a4, .Lfragment_done
    PRIME_FRAGMENT
    beqz a4, .Lfinal_group
    j .Lgroup

    /* Hardware loop body: every group of a fragment except its last. */
    .p2align 4
.Lgroup:
    .set final_group, 0
    SEND_GROUP
.Lfinal_group:
    .set final_group, 1
    SEND_GROUP
.Lfragment_done:
    ee.movi.32.a q6, a5, 1
    addi a5, a5, 1
    ee.movi.32.q q6, a5, 1
    bnei a5, 4, .Lnext_fragment

    /* Trailer: checksum, then end marker. */
    rur.accx_0 a5
    SEND_WORD a5
    rur.qacc_l_4 a6
    SEND_WORD a6
    _nop
    retw.n
    .size TX_NAME, . - TX_NAME
