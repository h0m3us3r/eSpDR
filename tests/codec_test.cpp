// Codec tests.
//   codec_test                        self-test of the reference codec
//   codec_test pattern PAIRS          write encoder test input (every mode wins
//                                     somewhere, plus a partial final block)
//   codec_test check PAIRS RECORDS [--end]
//       RECORDS (RTL output) must decode to PAIRS (LE u32 each) and equal the
//       reference encoding record for record. A record index that jumps ahead
//       is a gap: the pairs it skips must not appear. --end also requires the
//       END record (span and CRC of the delivered pairs) and zero padding.
#include "iq_record.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string &what)
{
    if (!condition) throw std::runtime_error(what);
}

uint32_t random_state = 1;
uint32_t random_pair()
{
    random_state ^= random_state << 13;
    random_state ^= random_state >> 17;
    random_state ^= random_state << 5;
    return random_state & 0xFFFFF;
}

uint32_t pattern_pair(unsigned pattern, unsigned j)
{
    switch (pattern) {
    case 0: return 0;
    case 1: return 0xFFFFF;
    case 2: return (j & 1) ? 0x80200 : 0x7FDFF;                 // extremes
    case 3: return (j & 1023) | ((1023 - (j & 1023)) << 10);    // ramps
    case 4: return random_pair();
    case 5: return (j % 7) | ((j % 11) << 10);
    case 6: return random_pair() & 0x1C07;
    default: return (j & 1) ? 0x7FFFF : 0x80000;
    }
}

void self_test()
{
    std::array<uint32_t, IQR_BLOCK_PAIRS> in{}, out{};
    std::array<uint8_t, 8192> record{};
    unsigned cases = 0;

    // Every mode, several block lengths and patterns; truncation, capacity and
    // single-bit corruption must all be rejected.
    for (unsigned pattern = 0; pattern < 8; ++pattern) {
        for (unsigned n : {1u, 2u, 15u, 16u, 17u, 511u, 1023u, 1024u}) {
            for (unsigned j = 0; j < n; ++j) in[j] = pattern_pair(pattern, j);
            for (int mode = -1; mode <= IQR_DELTA; ++mode) {
                uint64_t index = UINT64_MAX - n;
                size_t size = iqr::encode(in.data(), n, index, mode, record.data(), record.size());
                require(size, "encode");
                iqr::Header h;
                require(iqr::decode(record.data(), size, out.data(), out.size(), &h), "decode");
                require(std::equal(in.begin(), in.begin() + n, out.begin()), "round trip");
                require(h.index == index, "index");
                require(!iqr::decode(record.data(), size - 1, out.data(), out.size(), nullptr), "truncation");
                require(!iqr::decode(record.data(), size, out.data(), n - 1, nullptr), "capacity");
                for (size_t byte = 0; byte < size; ++byte) {
                    for (unsigned bit = 0; bit < 8; ++bit) {
                        record[byte] ^= uint8_t(1u << bit);
                        require(!iqr::decode(record.data(), size, out.data(), out.size(), nullptr),
                                "corruption accepted");
                        record[byte] ^= uint8_t(1u << bit);
                    }
                }
                ++cases;
            }
        }
    }

    // A hand-built CENTER record covering every 10-bit residual at each k,
    // including every escape boundary.
    for (unsigned k = 0; k < 10; ++k) {
        record.fill(0);
        unsigned bits = 0;
        auto put_bits = [&](unsigned value, unsigned count) {
            for (unsigned b = 0; b < count; ++b, ++bits)
                record[IQR_HEADER_BYTES + bits / 8] |= uint8_t(((value >> b) & 1) << (bits % 8));
        };
        auto put_field = [&](unsigned at, uint64_t value, unsigned bytes) {
            for (unsigned b = 0; b < bytes; ++b) record[at + b] = uint8_t(value >> (8 * b));
        };
        for (unsigned j = 0; j < 1024; ++j) {
            in[j] = 0;
            for (unsigned c = 0; c < 2; ++c) {
                unsigned z = c ? 1023 - j : j, q = z >> k;
                int x = (z & 1) ? -int((z + 1) / 2) : int(z / 2);
                in[j] |= (unsigned(x) & 1023) << (10 * c);
                put_bits(0, std::min(q, 15u));
                put_bits(1, 1);
                if (q >= 15) put_bits(z, 10);
                else put_bits(z & ((1u << k) - 1), k);
            }
        }
        put_field(IQR_OFF_MAGIC, IQR_MAGIC, 4);
        put_field(IQR_OFF_VERSION, IQR_VERSION, 1);
        put_field(IQR_OFF_MODE, IQR_CENTER, 1);
        put_field(IQR_OFF_K_I, k, 1);
        put_field(IQR_OFF_K_Q, k, 1);
        put_field(IQR_OFF_COUNT, 1024, 2);
        put_field(IQR_OFF_BITS, bits, 2);
        put_field(IQR_OFF_CRC, iqr::crc32_pairs(in.data(), 1024), 4);
        put_field(IQR_OFF_HEADER_CRC, iqr::crc32(record.data(), IQR_OFF_HEADER_CRC), 4);
        require(iqr::decode(record.data(), IQR_HEADER_BYTES + 2 * ((bits + 15) / 16), out.data(),
                            out.size(), nullptr),
                "hand-built Rice record");
        require(in == out, "hand-built Rice record contents");
    }

    require(!iqr::encode(in.data(), 0, 0, -1, record.data(), record.size()), "empty block");
    std::printf("PASS codec self-test: %u encode/decode cases, all single-bit corruptions rejected\n",
                cases);
}

std::vector<uint8_t> read_file(const char *path)
{
    std::ifstream file(path, std::ios::binary);
    require(bool(file), std::string("open ") + path);
    return {std::istreambuf_iterator<char>(file), {}};
}

void check(const char *pairs_path, const char *records_path, bool expect_end)
{
    std::vector<uint8_t> raw = read_file(pairs_path);
    std::vector<uint8_t> data = read_file(records_path);
    require(raw.size() % 4 == 0, "pair file length");
    std::vector<uint32_t> pairs(raw.size() / 4);
    for (size_t i = 0; i < pairs.size(); ++i)
        pairs[i] = raw[4 * i] | raw[4 * i + 1] << 8 | raw[4 * i + 2] << 16 | uint32_t(raw[4 * i + 3]) << 24;

    std::array<uint32_t, IQR_BLOCK_PAIRS> decoded{};
    std::array<uint8_t, IQR_MAX_RECORD_BYTES> reference{};
    std::vector<uint32_t> delivered;
    size_t at = 0, records = 0, gaps = 0;
    uint64_t index = 0, skipped = 0;
    unsigned modes[4] = {};
    bool ended = false;
    while (at < data.size()) {
        iqr::Header h;
        size_t size = iqr::inspect(data.data() + at, data.size() - at, &h);
        require(size && size <= data.size() - at, "record header at byte " + std::to_string(at));
        require(iqr::decode(data.data() + at, size, decoded.data(), decoded.size(), &h),
                "record " + std::to_string(records));
        require(h.index >= index, "record index went backwards");
        if (h.index > index) {
            ++gaps;
            skipped += h.index - index;
            index = h.index;
        }
        if (h.mode == IQR_END) {
            require(h.index == pairs.size(), "END span");
            require(h.crc == iqr::crc32_pairs(delivered.data(), delivered.size()), "END stream CRC");
            for (size_t i = at + size; i < data.size(); ++i) require(data[i] == 0, "padding");
            std::printf("END verified, %zu padding bytes\n", data.size() - at - size);
            ended = true;
            break;
        }
        require(index + h.count <= pairs.size(), "more pairs than the input");
        require(std::equal(decoded.begin(), decoded.begin() + h.count, pairs.begin() + index),
                "decoded pairs differ from the input");
        size_t expected = iqr::encode(pairs.data() + index, h.count, index, -1, reference.data(),
                                      reference.size());
        require(expected == size && std::equal(reference.begin(), reference.begin() + size,
                                               data.begin() + at),
                "record " + std::to_string(records) + " differs from the reference encoding");
        delivered.insert(delivered.end(), pairs.begin() + index, pairs.begin() + index + h.count);
        index += h.count;
        at += size;
        ++records;
        ++modes[h.mode];
    }
    require(index == pairs.size(), "missing pairs at the end");
    require(ended || !expect_end, "no END record");
    std::printf("PASS RTL records match the reference: %zu pairs, %zu records, modes %u/%u/%u/%u",
                delivered.size(), records, modes[0], modes[1], modes[2], modes[3]);
    if (gaps) std::printf(", %zu gaps of %llu pairs", gaps, (unsigned long long)skipped);
    std::printf("\n");
}

void write_pattern(const char *path)
{
    std::mt19937 rng(123);
    std::ofstream file(path, std::ios::binary);
    for (unsigned j = 0; j < 1024 * 64 + 17; ++j) {
        uint32_t pair;
        switch (j / 1024 % 9) {
        case 0: pair = 0; break;
        case 1: pair = 0xFFFFF; break;
        case 2: pair = (j & 1) ? 0x80200 : 0x7FDFF; break;
        case 3: pair = rng() & 0xFFFFF; break;                          // RAW
        case 4: pair = (rng() & 7) | (rng() & 15) << 10; break;         // MIN
        case 5: pair = (j & 1023) | (1023 - (j & 1023)) << 10; break;   // DELTA
        case 6: pair = (rng() & 127) | (rng() & 127) << 10; break;
        case 7: pair = 0x80000; break;
        default: {                                                      // CENTER burst
            int x = (128 + int(j % 32 / 2) * 7) * ((j & 1) ? -1 : 1);
            pair = j % 1024 < 32 ? (uint32_t(x) & 1023) * 1025 : 0;
        }
        }
        for (int b = 0; b < 4; ++b) file.put(char(pair >> (8 * b)));
    }
}

}  // namespace

int main(int argc, char **argv)
{
    try {
        if (argc == 1) {
            self_test();
        } else if (argc == 3 && std::string(argv[1]) == "pattern") {
            write_pattern(argv[2]);
        } else if ((argc == 4 || argc == 5) && std::string(argv[1]) == "check") {
            check(argv[2], argv[3], argc == 5 && std::string(argv[4]) == "--end");
        } else {
            std::fprintf(stderr, "usage: codec_test [pattern PAIRS | check PAIRS RECORDS [--end]]\n");
            return 2;
        }
        return 0;
    } catch (const std::exception &e) {
        std::fprintf(stderr, "FAIL %s\n", e.what());
        return 1;
    }
}
