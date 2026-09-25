/*
 * Compressed IQ record format, produced by the FPGA and decoded by the host.
 *
 * The FPGA splits the chronological stream of 20-bit IQ pairs into blocks of
 * up to IQR_BLOCK_PAIRS pairs and emits one self-contained record per block.
 * Records are packed back to back with no padding between them.
 *
 * Header (32 bytes, little-endian):
 *   0  magic       u32  IQR_MAGIC ("IIS1")
 *   4  version     u8   IQR_VERSION
 *   5  mode        u8   IQR_RAW, IQR_MIN, IQR_CENTER, IQR_DELTA or IQR_END
 *   6  k_i, k_q    u8   per-component coding parameter
 *   8  count       u16  pairs in this block (1..1024)
 *   10 bits        u16  payload length in bits
 *   12 base_i/q    i16  per-component predictor base
 *   16 index       u64  sample index of the first pair since FPGA_ARM
 *   24 crc         u32  CRC32 of the block's pairs, each as a LE u32
 *   28 header_crc  u32  CRC32 of bytes 0..27
 *
 * The payload follows, LSB-first, zero-padded to a multiple of 16 bits. For
 * every pair, I is coded before Q. Components are 10-bit two's complement.
 *   IQR_RAW     each component as 10 raw bits; k and base are zero
 *   IQR_MIN     component - base as a k-bit unsigned value (base = block min)
 *   IQR_CENTER  Rice(zigzag(component - base), k), base = block mean
 *   IQR_DELTA   Rice(zigzag(component - previous), k), previous starts at base
 * Differences wrap modulo 1024 before the zigzag mapping. Rice(z, k) is q
 * zero bits, a one bit, then the k low bits of z, with q = z >> k; when
 * q >= 15 it is instead 15 zero bits, a one bit, and z as 10 raw bits.
 *
 * The encoder picks the mode with the fewest 16-bit payload words, preferring
 * RAW, MIN, CENTER, DELTA on ties. CENTER's base is the signed mean truncated
 * toward zero for a full block and zero for a shorter final block.
 *
 * Records normally follow each other without gaps in their indices. If the
 * FPGA lost part of the stream (a link unit it could not receive), the next
 * record's index skips over the missing pairs, so indices always count
 * sample time since FPGA_ARM.
 *
 * When the stream stops, an IQR_END header follows the last record: count,
 * bits, k and bases are zero, index is the total span (pairs delivered plus
 * pairs lost) and crc is the CRC32 of every delivered pair (as LE u32). Zero
 * bytes then pad the stream to the next IQR_TRANSFER_BYTES boundary.
 */
#ifndef IQSTREAM_IQ_RECORD_H
#define IQSTREAM_IQ_RECORD_H

#define IQR_MAGIC 0x31534949
#define IQR_VERSION 1
#define IQR_BLOCK_PAIRS 1024
#define IQR_HEADER_BYTES 32
#define IQR_MAX_RECORD_BYTES 2592   /* raw record: 32 + 1024 * 20 / 8 */
#define IQR_TRANSFER_BYTES 1048576  /* the FPGA sends whole 1 MiB USB transfers */

#define IQR_OFF_MAGIC 0
#define IQR_OFF_VERSION 4
#define IQR_OFF_MODE 5
#define IQR_OFF_K_I 6
#define IQR_OFF_K_Q 7
#define IQR_OFF_COUNT 8
#define IQR_OFF_BITS 10
#define IQR_OFF_BASE_I 12
#define IQR_OFF_BASE_Q 14
#define IQR_OFF_INDEX 16
#define IQR_OFF_CRC 24
#define IQR_OFF_HEADER_CRC 28

#define IQR_RAW 0
#define IQR_MIN 1
#define IQR_CENTER 2
#define IQR_DELTA 3
#define IQR_END 4

#ifndef __ASSEMBLER__
#ifdef __cplusplus
#include <cstddef>
#include <cstdint>

namespace iqr {

struct Header {
    uint8_t mode = 0;
    uint8_t k[2] = {0, 0};
    int16_t base[2] = {0, 0};
    uint16_t count = 0;
    uint16_t bits = 0;
    uint64_t index = 0;
    uint32_t crc = 0;
};

/* CRC32 (IEEE) of a byte buffer, and of pairs stored as LE u32 words. */
uint32_t crc32(const uint8_t *bytes, size_t size);
uint32_t crc32_pairs(const uint32_t *pairs, size_t count);

/*
 * Encodes 1..IQR_BLOCK_PAIRS pairs (each < 2^20) into out. mode -1 selects
 * the best mode as the FPGA does; 0..3 forces one (for tests). Returns the
 * record size, or 0 on invalid input or insufficient capacity. A forced Rice
 * mode may need up to 32 + 6656 bytes.
 */
size_t encode(const uint32_t *pairs, size_t count, uint64_t index, int mode,
              uint8_t *out, size_t capacity);

/* Validates a header; returns the full record size, or 0 if invalid. */
size_t inspect(const uint8_t *record, size_t available, Header *header);

/*
 * Decodes one complete record of exactly `size` bytes into pairs. Checks the
 * header, every payload bit, the zero padding and the block CRC. Returns
 * false on any error. An END record decodes to zero pairs.
 */
bool decode(const uint8_t *record, size_t size, uint32_t *pairs, size_t capacity,
            Header *header);

}  // namespace iqr

#endif
#endif
#endif
