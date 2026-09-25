/*
 * ESP32-S3 -> FPGA parallel link.
 *
 * Sixteen GPIO lines form two independent 8-bit lanes. CPU core 0 drives
 * lane 0 (GPIO 4,5,6,7,15,16,17,18) and core 1 drives lane 1
 * (GPIO 3,46,9,10,11,12,13,14), each through its dedicated-GPIO port. The
 * FPGA samples every line at 240 MHz, synchronous with the ESP's 240 MHz CPU
 * clock (both derive from the FPGA's 40 MHz reference).
 *
 * The ADC writes 80 Msps IQ pairs into four 16384-pair SRAM banks. Core 0
 * owns banks 0 and 2, core 1 owns banks 1 and 3. A "unit" is the contiguous
 * run of pairs one bank received before the writer moved to the next bank.
 * Even sequence numbers travel on lane 0, odd ones on lane 1, so units
 * alternate between lanes in chronological order.
 *
 * Unit on the wire (one byte per two CPU cycles unless noted):
 *   marker   0xFF held for LINK_MARKER_CYCLES, then 0x00
 *   header   sequence:u32, layout:u32 (little-endian)
 *   payload  groups of 16 pairs; each group is 32 bytes of the low 16 bits
 *            of every pair (LE), then 8 bytes holding the high 4 bits of
 *            pairs (2k, 2k+1) in the low/high nibble of byte k
 *   trailer  checksum:u32, end marker:u32
 *
 * layout = first | count << 14 | bank << 28 | LINK_VERSION << 30, where
 * first is the ring index of the first pair and count the number of pairs.
 * A unit whose first pair is not 16-aligned starts with a padded head group,
 * and one whose length is not a multiple of 16 ends with a padded tail group;
 * padding pairs are zero and are discarded by the receiver.
 *
 * checksum is the 32-bit sum of every little-endian 16-bit word of the header
 * and payload. It catches every single-bit error; the sequence, layout and
 * end marker are checked independently.
 *
 * A pair is 20 bits: I in bits 0..9 and Q in bits 10..19, both two's
 * complement. The raw ESP convention is LO minus RF for I+jQ.
 *
 * The byte timing inside a unit is fixed by the transmit kernel
 * (esp32s3/src/transmit.S) and mirrored by the FPGA receiver
 * (fpga/rtl/link_lane.v). Change both together or neither.
 */
#ifndef IQSTREAM_LINK_H
#define IQSTREAM_LINK_H

#define LINK_VERSION 3
#define LINK_PAIRS_PER_SECOND 80000000 /* ADC rate: pairs the ESP captures per second */
#define LINK_END_MARKER 0xA5FAA96D
#define LINK_RING_PAIRS 16384
#define LINK_THRESHOLD_PAIRS 15360   /* switch banks after this many pairs */
#define LINK_MIN_PAIRS 15360
#define LINK_MAX_PAIRS 16288         /* leaves a 96-pair gap in the ring */
#define LINK_MARKER_CYCLES 240
#define LINK_GROUP_PAIRS 16
#define LINK_GROUP_BYTES 40
#define LINK_GROUP_CYCLES 91
#define LINK_HEADER_BYTES 8
#define LINK_TRAILER_BYTES 8

#endif
