// Generates a lane waveform as the ESP transmit kernel produces it, plus the
// pairs the receiver must recover.
//   link_wave WAVE PAIRS UNITS [SEED] [--faults]
// The lines change τ into each 240 MHz cycle; the FPGA samples them a
// quarter and three quarters into it (rising and falling clock edge). WAVE
// holds the two samples of every cycle. Each unit has its own τ, cycling
// through 0.1, 0.5, 0.75 (the falling-edge sample lands on the marker edge
// and reads a mix of old and new bits) and 0.9 (the edge is found one sample
// later). Units use random ring offsets and lengths, so head, tail and
// wrapped fragments all occur, and start at random even or odd cycles.
// --faults also drops every fifth unit's marker and corrupts every seventh
// unit's header; the receiver must lose those units and resynchronise. PAIRS
// then holds only the pairs that must be received; the counts of accepted
// and rejected units are printed as "accepted A rejected R".
#include "link.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <random>
#include <string>
#include <vector>

namespace {

// Cycle schedule of esp32s3/src/transmit.S, relative to the marker.
constexpr uint32_t kHeaderCycle = 242;
constexpr uint32_t kFirstPayloadOneGroup = 270;
constexpr uint32_t kFirstPayloadMultiGroup = 271;
constexpr uint32_t kSkipEmptyFragment[4] = {0, 21, 18, 0};
constexpr uint32_t kFragmentStart[4] = {0, 42, 39, 41};
constexpr uint32_t kTrailerGap[4] = {67, 46, 28, 8};

struct Unit {
    std::vector<uint8_t> bytes;    // header, payload, trailer
    std::vector<uint32_t> cycles;  // write cycle of each byte
    std::vector<uint32_t> pairs;   // valid pairs in order
};

Unit make_unit(uint32_t sequence, uint32_t first, uint32_t count, std::mt19937 &rng)
{
    Unit unit;
    std::vector<uint32_t> ring(LINK_RING_PAIRS);
    for (auto &pair : ring) pair = rng() & 0xFFFFF;
    for (uint32_t i = 0; i < count; ++i) unit.pairs.push_back(ring[(first + i) % LINK_RING_PAIRS]);

    uint32_t head = (-first) & 15;
    if (head > count) head = count;
    uint32_t tail = (count - head) & 15;
    uint32_t bulk = count - head - tail;
    uint32_t bulk_at = (first + head) % LINK_RING_PAIRS;
    uint32_t before_wrap = std::min(LINK_RING_PAIRS - bulk_at, bulk);
    uint32_t groups[4] = {head != 0, before_wrap / 16, (bulk - before_wrap) / 16, tail != 0};

    // Groups in transmission order, padding pairs zero.
    std::vector<uint32_t> padded;
    auto add = [&](uint32_t at, uint32_t n) {
        for (uint32_t i = 0; i < n; ++i) padded.push_back(ring[(at + i) % LINK_RING_PAIRS]);
        while (padded.size() % 16) padded.push_back(0);
    };
    if (head) add(first, head);
    add(bulk_at, bulk);
    if (tail) add((first + head + bulk) % LINK_RING_PAIRS, tail);

    // The ESP captures unit n into bank n % 4.
    uint32_t layout = first | count << 14 | (sequence & 3) << 28 | uint32_t(LINK_VERSION) << 30;
    for (int i = 0; i < 4; ++i) unit.bytes.push_back(uint8_t(sequence >> (8 * i)));
    for (int i = 0; i < 4; ++i) unit.bytes.push_back(uint8_t(layout >> (8 * i)));
    for (size_t g = 0; g < padded.size(); g += 16) {
        for (size_t i = 0; i < 16; ++i) {
            unit.bytes.push_back(uint8_t(padded[g + i]));
            unit.bytes.push_back(uint8_t(padded[g + i] >> 8));
        }
        for (size_t i = 0; i < 16; i += 2)
            unit.bytes.push_back(uint8_t((padded[g + i] >> 16) | ((padded[g + i + 1] >> 16) << 4)));
    }
    uint32_t checksum = 0;
    for (size_t i = 0; i < unit.bytes.size(); i += 2) checksum += unit.bytes[i] | unit.bytes[i + 1] << 8;
    for (int i = 0; i < 4; ++i) unit.bytes.push_back(uint8_t(checksum >> (8 * i)));
    for (int i = 0; i < 4; ++i) unit.bytes.push_back(uint8_t(uint32_t(LINK_END_MARKER) >> (8 * i)));

    for (uint32_t i = 0; i < LINK_HEADER_BYTES; ++i) unit.cycles.push_back(kHeaderCycle + 2 * i);
    uint32_t cycle = 0, final_cycle = 0;
    int last_fragment = 0;
    bool first_fragment = true;
    for (int f = 0; f < 4; ++f) {
        uint32_t n = groups[f];
        if (!n) {
            if (!first_fragment) cycle += kSkipEmptyFragment[f];
            continue;
        }
        uint32_t start = first_fragment ? (n > 1 ? kFirstPayloadMultiGroup : kFirstPayloadOneGroup)
                                        : cycle + kFragmentStart[f] + (n > 1 ? 3 : 0);
        first_fragment = false;
        for (uint32_t g = 0; g < n; ++g)
            for (uint32_t b = 0; b < LINK_GROUP_BYTES; ++b)
                unit.cycles.push_back(start + LINK_GROUP_CYCLES * g + ((g && n >= 3) ? 1 : 0) + 2 * b);
        final_cycle = start + LINK_GROUP_CYCLES * (n - 1) + (n >= 3 ? 1 : 0) + 78;
        cycle = final_cycle + 8;
        last_fragment = f;
    }
    for (uint32_t i = 0; i < LINK_TRAILER_BYTES; ++i)
        unit.cycles.push_back(final_cycle + kTrailerGap[last_fragment] + 2 * i);
    return unit;
}

}  // namespace

int main(int argc, char **argv)
{
    bool faults = argc == 6 && std::string(argv[5]) == "--faults";
    if (argc < 4 || argc > 6 || (argc == 6 && !faults)) {
        std::fprintf(stderr, "usage: link_wave WAVE PAIRS UNITS [SEED] [--faults]\n");
        return 2;
    }
    unsigned units = std::stoul(argv[3]);
    std::mt19937 rng(argc >= 5 ? std::stoul(argv[4]) : 1);
    unsigned accepted = 0, rejected = 0;
    std::vector<uint8_t> wave(1000, 0);       // line levels, one per cycle
    std::vector<double> tau(1000, 0.0);       // transition time within each cycle
    const double kTau[4] = {0.1, 0.5, 0.75, 0.9};
    std::ofstream pairs_file(argv[2], std::ios::binary);

    for (unsigned u = 0; u < units; ++u) {
        uint32_t first = rng() % LINK_RING_PAIRS;
        uint32_t count = LINK_MIN_PAIRS + rng() % (LINK_MAX_PAIRS - LINK_MIN_PAIRS + 1);
        if (u == 0) first = 0;                            // aligned, no head
        if (u == 1) first = LINK_RING_PAIRS - 5;          // head, then wrap
        if (u == 2) count = LINK_MAX_PAIRS;
        if (u == 3) count = LINK_MIN_PAIRS;
        Unit unit = make_unit(2 * u, first, count, rng);

        size_t origin = wave.size() + (rng() & 1);        // either sample phase
        wave.resize(origin, wave.back());
        wave.resize(origin + unit.cycles.back() + 2 + 200 + rng() % 2000, unit.bytes.back());
        tau.resize(wave.size(), kTau[u % 4]);
        bool missing_marker = faults && u % 5 == 4;
        bool bad_header = faults && u % 7 == 6 && !missing_marker;
        if (bad_header) unit.bytes[7] ^= 0x80;          // layout version 3 -> 2
        for (uint32_t c = 0; c < 240; ++c) wave[origin + c] = (missing_marker && c > 100) ? 0x00 : 0xFF;
        for (uint32_t c = 240; c < unit.cycles[0]; ++c) wave[origin + c] = 0x00;
        for (size_t i = 0; i < unit.bytes.size(); ++i) {
            size_t end = i + 1 < unit.bytes.size() ? unit.cycles[i + 1] : unit.cycles[i] + 2;
            for (size_t c = unit.cycles[i]; c < end; ++c) wave[origin + c] = unit.bytes[i];
        }
        if (missing_marker || bad_header) {
            rejected += bad_header;
            continue;
        }
        ++accepted;
        for (uint32_t pair : unit.pairs)
            for (int b = 0; b < 4; ++b) pairs_file.put(char(pair >> (8 * b)));
    }
    if (wave.size() & 1) {
        wave.push_back(wave.back());
        tau.push_back(tau.back());
    }

    // Sample every cycle at its rising (+0.25) and falling (+0.75) clock edge.
    // The lines hold wave[k] from k + tau to k + 1 + tau; a sample exactly at a
    // change reads each line's old or new level at random.
    std::vector<uint8_t> samples;
    for (size_t c = 0; c < wave.size(); ++c) {
        for (double at : {0.25, 0.75}) {
            double t = double(c) + at - tau[c];
            long k = long(std::floor(t));
            uint8_t value = wave[std::max(0L, k)];
            if (t == double(k) && k > 0 && wave[k - 1] != value) {
                uint8_t pick = uint8_t(rng());
                value = uint8_t((value & pick) | (wave[k - 1] & ~pick));
            }
            samples.push_back(value);
        }
    }
    std::ofstream(argv[1], std::ios::binary).write(reinterpret_cast<const char *>(samples.data()), samples.size());
    std::printf("accepted %u rejected %u\n", accepted, rejected);
    return 0;
}
