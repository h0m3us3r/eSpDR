// Reference encoder/decoder for the compressed IQ record format.
// The FPGA encoder is byte-for-byte identical to encode() with mode -1.
#include "iq_record.h"

#include <algorithm>
#include <array>
#include <cstring>

namespace iqr {
namespace {

constexpr unsigned kMaxRiceQuotient = 15;
constexpr unsigned kEscapeBits = 26;  // 15 zeros, a one, 10 raw bits

const std::array<uint32_t, 256> kCrcTable = [] {
    std::array<uint32_t, 256> table{};
    for (unsigned i = 0; i < 256; ++i) {
        uint32_t c = i;
        for (int bit = 0; bit < 8; ++bit)
            c = (c & 1) ? (c >> 1) ^ 0xEDB88320u : c >> 1;
        table[i] = c;
    }
    return table;
}();

uint32_t crc_update(uint32_t crc, uint8_t byte) {
    return kCrcTable[(crc ^ byte) & 0xFF] ^ (crc >> 8);
}

// Component c (0 = I, 1 = Q) of a pair, sign-extended from 10 bits.
int component(uint32_t pair, unsigned c) {
    unsigned x = (pair >> (10 * c)) & 1023;
    return int(x & 511) - int(x & 512);
}

// Wraps a difference to the 10-bit signed range, as the hardware does.
int wrap10(int x) { return component(uint32_t(x) & 1023, 0); }

unsigned zigzag(int x) { return x < 0 ? unsigned(-2 * x - 1) : unsigned(2 * x); }
int unzigzag(unsigned z) { return (z & 1) ? -int((z + 1) / 2) : int(z / 2); }

unsigned bit_width(unsigned x) { return x ? 32 - __builtin_clz(x) : 0; }

unsigned rice_bits(unsigned z, unsigned k) {
    unsigned q = z >> k;
    return q >= kMaxRiceQuotient ? kEscapeBits : q + 1 + k;
}

uint64_t load_le(const uint8_t *p, unsigned bytes) {
    uint64_t v = 0;
    for (unsigned i = 0; i < bytes; ++i) v |= uint64_t(p[i]) << (8 * i);
    return v;
}

void store_le(uint8_t *p, uint64_t v, unsigned bytes) {
    for (unsigned i = 0; i < bytes; ++i) p[i] = uint8_t(v >> (8 * i));
}

class BitWriter {
public:
    explicit BitWriter(uint8_t *out) : out_(out) {}

    void put(unsigned value, unsigned bits) {
        acc_ |= uint64_t(value) << used_;
        used_ += bits;
        while (used_ >= 8) {
            *out_++ = uint8_t(acc_);
            acc_ >>= 8;
            used_ -= 8;
        }
    }

    void rice(unsigned z, unsigned k) {
        unsigned q = z >> k;
        if (q >= kMaxRiceQuotient)
            put((z << 16) | (1u << 15), kEscapeBits);
        else
            put((1u << q) | ((z & ((1u << k) - 1)) << (q + 1)), q + 1 + k);
    }

    void flush() {
        if (used_) *out_ = uint8_t(acc_);
    }

private:
    uint8_t *out_;
    uint64_t acc_ = 0;
    unsigned used_ = 0;
};

class BitReader {
public:
    BitReader(const uint8_t *data, const uint8_t *end, unsigned limit_bits)
        : data_(data), end_(end), limit_(limit_bits) {}

    unsigned take(unsigned bits) {
        if (position_ + bits > limit_) return fail();
        while (used_ < bits) {
            if (data_ == end_) return fail();
            acc_ |= uint64_t(*data_++) << used_;
            used_ += 8;
        }
        unsigned value = unsigned(acc_) & ((1u << bits) - 1);
        acc_ >>= bits;
        used_ -= bits;
        position_ += bits;
        return value;
    }

    unsigned rice(unsigned k) {
        unsigned q = 0;
        while (good_ && take(1) == 0) {
            if (++q > kMaxRiceQuotient) return fail();
        }
        if (!good_) return 0;
        unsigned z = q == kMaxRiceQuotient ? take(10) : (q << k) | take(k);
        if (z > 1023) return fail();
        return z;
    }

    bool good() const { return good_; }
    unsigned position() const { return position_; }

private:
    unsigned fail() {
        good_ = false;
        return 0;
    }

    const uint8_t *data_;
    const uint8_t *end_;
    uint64_t acc_ = 0;
    unsigned used_ = 0;
    unsigned position_ = 0;
    unsigned limit_;
    bool good_ = true;
};

// Total Rice cost of one component for every k in 0..9, and the best k.
struct RiceChoice {
    unsigned k = 0;
    unsigned bits = 0;
};

RiceChoice best_rice(const uint32_t *pairs, size_t count, unsigned c, int base,
                     bool delta) {
    unsigned cost[10] = {};
    int previous = base;
    for (size_t j = 0; j < count; ++j) {
        int x = component(pairs[j], c);
        unsigned z = zigzag(wrap10(x - previous));
        for (unsigned k = 0; k < 10; ++k) cost[k] += rice_bits(z, k);
        if (delta) previous = x;
    }
    unsigned k = unsigned(std::min_element(cost, cost + 10) - cost);
    return {k, cost[k]};
}

}  // namespace

uint32_t crc32(const uint8_t *bytes, size_t size) {
    uint32_t crc = ~0u;
    while (size--) crc = crc_update(crc, *bytes++);
    return ~crc;
}

uint32_t crc32_pairs(const uint32_t *pairs, size_t count) {
    uint32_t crc = ~0u;
    for (size_t i = 0; i < count; ++i)
        for (unsigned b = 0; b < 4; ++b) crc = crc_update(crc, uint8_t(pairs[i] >> (8 * b)));
    return ~crc;
}

size_t encode(const uint32_t *pairs, size_t count, uint64_t index, int forced_mode,
              uint8_t *out, size_t capacity) {
    if (count == 0 || count > IQR_BLOCK_PAIRS || forced_mode < -1 || forced_mode > IQR_DELTA)
        return 0;
    for (size_t j = 0; j < count; ++j)
        if (pairs[j] >> 20) return 0;

    Header candidates[4];
    for (unsigned mode = 0; mode < 4; ++mode) {
        candidates[mode].mode = uint8_t(mode);
        candidates[mode].count = uint16_t(count);
        candidates[mode].index = index;
    }
    unsigned bits[4] = {unsigned(20 * count), 0, 0, 0};

    for (unsigned c = 0; c < 2; ++c) {
        int lo = 511, hi = -512, sum = 0;
        for (size_t j = 0; j < count; ++j) {
            int x = component(pairs[j], c);
            lo = std::min(lo, x);
            hi = std::max(hi, x);
            sum += x;
        }
        Header &min = candidates[IQR_MIN];
        min.base[c] = int16_t(lo);
        min.k[c] = uint8_t(bit_width(unsigned(hi - lo)));
        bits[IQR_MIN] += unsigned(count) * min.k[c];

        Header &center = candidates[IQR_CENTER];
        center.base[c] = int16_t(count == IQR_BLOCK_PAIRS ? sum / int(count) : 0);
        RiceChoice rc = best_rice(pairs, count, c, center.base[c], false);
        center.k[c] = uint8_t(rc.k);
        bits[IQR_CENTER] += rc.bits;

        Header &delta = candidates[IQR_DELTA];
        delta.base[c] = int16_t(component(pairs[0], c));
        rc = best_rice(pairs, count, c, delta.base[c], true);
        delta.k[c] = uint8_t(rc.k);
        bits[IQR_DELTA] += rc.bits;
    }

    unsigned mode = IQR_RAW;
    for (unsigned m = 1; m < 4; ++m)
        if ((bits[m] + 15) / 16 < (bits[mode] + 15) / 16) mode = m;
    if (forced_mode >= 0) mode = unsigned(forced_mode);

    Header &h = candidates[mode];
    h.bits = uint16_t(bits[mode]);
    size_t size = IQR_HEADER_BYTES + 2 * ((bits[mode] + 15) / 16);
    if (bits[mode] > 0xFFFF || size > capacity) return 0;

    std::memset(out, 0, size);
    store_le(out + IQR_OFF_MAGIC, IQR_MAGIC, 4);
    out[IQR_OFF_VERSION] = IQR_VERSION;
    out[IQR_OFF_MODE] = h.mode;
    out[IQR_OFF_K_I] = h.k[0];
    out[IQR_OFF_K_Q] = h.k[1];
    store_le(out + IQR_OFF_COUNT, h.count, 2);
    store_le(out + IQR_OFF_BITS, h.bits, 2);
    store_le(out + IQR_OFF_BASE_I, uint16_t(h.base[0]), 2);
    store_le(out + IQR_OFF_BASE_Q, uint16_t(h.base[1]), 2);
    store_le(out + IQR_OFF_INDEX, h.index, 8);
    store_le(out + IQR_OFF_CRC, crc32_pairs(pairs, count), 4);
    store_le(out + IQR_OFF_HEADER_CRC, crc32(out, IQR_OFF_HEADER_CRC), 4);

    BitWriter writer(out + IQR_HEADER_BYTES);
    int previous[2] = {h.base[0], h.base[1]};
    for (size_t j = 0; j < count; ++j) {
        for (unsigned c = 0; c < 2; ++c) {
            int x = component(pairs[j], c);
            switch (mode) {
            case IQR_RAW: writer.put(unsigned(x) & 1023, 10); break;
            case IQR_MIN: writer.put(unsigned(x - h.base[c]), h.k[c]); break;
            case IQR_CENTER: writer.rice(zigzag(wrap10(x - h.base[c])), h.k[c]); break;
            case IQR_DELTA:
                writer.rice(zigzag(wrap10(x - previous[c])), h.k[c]);
                previous[c] = x;
                break;
            }
        }
    }
    writer.flush();
    return size;
}

size_t inspect(const uint8_t *p, size_t available, Header *out) {
    if (available < IQR_HEADER_BYTES || load_le(p + IQR_OFF_MAGIC, 4) != IQR_MAGIC ||
        p[IQR_OFF_VERSION] != IQR_VERSION ||
        crc32(p, IQR_OFF_HEADER_CRC) != load_le(p + IQR_OFF_HEADER_CRC, 4))
        return 0;

    Header h;
    h.mode = p[IQR_OFF_MODE];
    h.k[0] = p[IQR_OFF_K_I];
    h.k[1] = p[IQR_OFF_K_Q];
    h.count = uint16_t(load_le(p + IQR_OFF_COUNT, 2));
    h.bits = uint16_t(load_le(p + IQR_OFF_BITS, 2));
    h.base[0] = int16_t(load_le(p + IQR_OFF_BASE_I, 2));
    h.base[1] = int16_t(load_le(p + IQR_OFF_BASE_Q, 2));
    h.index = load_le(p + IQR_OFF_INDEX, 8);
    h.crc = uint32_t(load_le(p + IQR_OFF_CRC, 4));

    if (h.mode > IQR_END || h.count > IQR_BLOCK_PAIRS || h.bits > 52u * h.count) return 0;
    if (h.mode == IQR_END) {
        if (h.count || h.bits || h.k[0] || h.k[1] || h.base[0] || h.base[1]) return 0;
        if (out) *out = h;
        return IQR_HEADER_BYTES;
    }
    if (h.count == 0) return 0;
    for (unsigned c = 0; c < 2; ++c) {
        unsigned max_k = h.mode >= IQR_CENTER ? 9 : 10;
        if (h.base[c] < -512 || h.base[c] > 511 || h.k[c] > max_k) return 0;
    }
    if (h.mode == IQR_RAW &&
        (h.bits != 20u * h.count || h.k[0] || h.k[1] || h.base[0] || h.base[1]))
        return 0;
    if (h.mode == IQR_MIN && h.bits != unsigned(h.count) * (h.k[0] + h.k[1])) return 0;

    if (out) *out = h;
    return IQR_HEADER_BYTES + 2 * ((h.bits + 15u) / 16u);
}

bool decode(const uint8_t *p, size_t size, uint32_t *pairs, size_t capacity, Header *out) {
    Header h;
    size_t expected = inspect(p, size, &h);
    if (!expected || expected != size || capacity < h.count) return false;
    if (h.mode == IQR_END) {
        if (out) *out = h;
        return true;
    }

    BitReader reader(p + IQR_HEADER_BYTES, p + size, h.bits);
    int previous[2] = {h.base[0], h.base[1]};
    for (unsigned j = 0; j < h.count; ++j) {
        uint32_t pair = 0;
        for (unsigned c = 0; c < 2; ++c) {
            int x;
            switch (h.mode) {
            case IQR_RAW: x = component(reader.take(10), 0); break;
            case IQR_MIN:
                x = h.base[c] + int(reader.take(h.k[c]));
                if (x > 511) return false;
                break;
            case IQR_CENTER: x = wrap10(h.base[c] + unzigzag(reader.rice(h.k[c]))); break;
            default:
                x = wrap10(previous[c] + unzigzag(reader.rice(h.k[c])));
                previous[c] = x;
                break;
            }
            pair |= (unsigned(x) & 1023) << (10 * c);
        }
        pairs[j] = pair;
    }
    if (!reader.good() || reader.position() != h.bits) return false;

    // Padding after the last payload bit must be zero.
    for (unsigned bit = h.bits; bit < (size - IQR_HEADER_BYTES) * 8; ++bit)
        if ((p[IQR_HEADER_BYTES + bit / 8] >> (bit % 8)) & 1) return false;
    if (crc32_pairs(pairs, h.count) != h.crc) return false;

    if (out) *out = h;
    return true;
}

}  // namespace iqr
