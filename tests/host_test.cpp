// Host-side tests for serve: the spectrum engine and the web server.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <zlib.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "spectrum.h"
#include "web.h"

namespace {

int failures = 0;
void check(bool ok, const std::string &what)
{
    std::printf("%s %s\n", ok ? "PASS" : "FAIL", what.c_str());
    if (!ok) ++failures;
}

// ---- spectrum -----------------------------------------------------------------------------------------

struct Collected {
    std::mutex mutex;
    std::vector<std::shared_ptr<const SpectrumFrame>> frames;
};

// Feeds `pairs` of a complex tone at RF offset `rf_hz` (the ESP's I + jQ is
// LO minus RF, so the samples turn the other way) in batches of `batch`,
// skipping `gap` pairs after the first half if asked.
// `on` says which pairs carry the tone.
void feed(SpectrumEngine &engine, double rate, double rf_hz, uint64_t pairs, uint64_t batch, uint64_t gap,
          const std::function<bool(uint64_t)> &on)
{
    uint64_t index = 0;
    while (index < pairs) {
        uint64_t n = std::min(batch, pairs - index);
        std::vector<int16_t> samples(2 * n);
        for (uint64_t i = 0; i < n; ++i) {
            double phase = -2 * M_PI * rf_hz * double(index + i) / rate, amplitude = on(index + i) ? 500 : 0;
            samples[2 * i] = int16_t(std::lround(amplitude * std::cos(phase)));
            samples[2 * i + 1] = int16_t(std::lround(amplitude * std::sin(phase)));
        }
        engine.add(index, std::move(samples));
        index += n;
        if (gap && index >= pairs / 2 && index < pairs / 2 + batch) index += gap;
        std::this_thread::sleep_for(std::chrono::microseconds(500));  // the engine drops batches it cannot take
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

std::vector<std::shared_ptr<const SpectrumFrame>> run(const SpectrumSettings &settings, double rate, double rf_hz,
                                                      uint64_t pairs, uint64_t batch, uint64_t gap = 0,
                                                      SpectrumEngine::Stats *stats = nullptr,
                                                      std::function<bool(uint64_t)> on = [](uint64_t) { return true; })
{
    Collected out;
    SpectrumEngine engine([&](std::shared_ptr<const SpectrumFrame> f) {
        std::lock_guard<std::mutex> lock(out.mutex);
        out.frames.push_back(std::move(f));
    });
    engine.configure(settings);
    engine.begin_run(1, 2.44e9, rate);
    feed(engine, rate, rf_hz, pairs, batch, gap, on);
    if (stats) *stats = engine.stats();
    std::lock_guard<std::mutex> lock(out.mutex);
    return out.frames;
}

// Display bin of an RF offset: bin 0 is the lowest frequency, size/2 the LO.
size_t bin_of(double rf_hz, double rate, unsigned size)
{
    return size_t(std::lround(rf_hz / (rate / size)) + long(size / 2));
}

size_t peak_bin(const SpectrumFrame &f)
{
    size_t best = 0;
    for (size_t i = 1; i < f.db.size(); ++i)
        if (f.db[i] > f.db[best]) best = i;
    return best;
}

void spectrum_tests()
{
    SpectrumSettings s;  // SDR#'s defaults: 4096 points, Blackman-Harris 4, 40 frames/s, no averaging
    s.validate();
    // FFTW plans each size once, slowly; the engine drops what arrives meanwhile.
    for (unsigned size : {512u, 4096u, 16384u}) {
        SpectrumSettings warm = s;
        warm.size = size;
        run(warm, 1e6, 0, 20000, 20000);
    }
    const double rate = 80e6;

    // A tone on a bin centre lands in the right bin, the right way round, at
    // its level in dBFS (500 of a 512 full scale is -0.21 dB).
    for (double rf : {10e6, -10e6, 30e6}) {
        auto frames = run(s, rate, rf, 3 * rate / s.rate, 250000);
        size_t expected = bin_of(rf, rate, s.size);
        bool ok = !frames.empty();
        double level = 0;
        for (const auto &f : frames) {
            ok = ok && peak_bin(*f) == expected;
            level = f->db[expected];
        }
        check(ok && std::fabs(level - 20 * std::log10(500.0 / 512)) < 0.1,
              "tone at " + std::to_string(int(rf / 1e6)) + " MHz in bin " + std::to_string(expected) +
                  " at " + std::to_string(level) + " dBFS, " + std::to_string(frames.size()) + " frames");
    }

    // Frames follow sample time: 40 per second of samples, whatever the batches.
    SpectrumSettings small = s;
    small.size = 512;
    const double slow = 1e6;
    SpectrumEngine::Stats stats;
    auto frames = run(small, slow, 100e3, uint64_t(slow), 7919, 0, &stats);
    check(frames.size() >= 39 && frames.size() <= 40 && stats.pairs_dropped == 0,
          "40 frames from one second of samples (" + std::to_string(frames.size()) + ")");

    // Averaging: that many blocks per frame; 0 takes every sample.
    small.averaging = 8;
    frames = run(small, slow, 100e3, uint64_t(slow / 2), 7919);
    bool eight = !frames.empty();
    for (const auto &f : frames) eight = eight && f->blocks == 8;
    check(eight, "averaging 8: 8 blocks in every frame");
    small.averaging = 0;
    frames = run(small, slow, 100e3, uint64_t(slow / 2), 7919);
    bool every = !frames.empty();
    for (const auto &f : frames) every = every && f->blocks == 48;  // 25000 pairs per frame / 512
    check(every, "averaging 0: every sample (48 blocks a frame)");

    // Blocks longer than a batch are built from the batches before it.
    SpectrumSettings large = s;
    large.size = 16384;
    frames = run(large, slow, -200e3, uint64_t(slow / 2), 1000);
    size_t expected = bin_of(-200e3, slow, large.size);
    bool joined = frames.size() >= 19;
    for (const auto &f : frames) joined = joined && peak_bin(*f) == expected;
    check(joined, "16384-point blocks from 1000-pair batches (" + std::to_string(frames.size()) + " frames)");

    // A gap in the stream costs the frames it covers and no more.
    small.averaging = 1;
    frames = run(small, slow, 100e3, uint64_t(slow), 7919, 60000);
    check(frames.size() >= 36 && frames.size() <= 39, "a 60 ms gap drops 2 of 40 frames (" +
                                                           std::to_string(frames.size()) + " left)");

    // A tone in only the first of eight blocks per frame: the peak detector
    // shows it at full level, averaging at an eighth of its power (-9 dB).
    SpectrumSettings burst = small;
    burst.averaging = 8;
    auto first_block = [](uint64_t i) { return i % 25000 < 3000; };
    size_t tone = bin_of(100e3, slow, burst.size);
    double full = 20 * std::log10(500.0 / 512);
    frames = run(burst, slow, 100e3, 250000, 7919, 0, nullptr, first_block);
    double averaged = frames.empty() ? 0 : frames.back()->db[tone];
    burst.peak = true;
    frames = run(burst, slow, 100e3, 250000, 7919, 0, nullptr, first_block);
    double held = frames.empty() ? 0 : frames.back()->db[tone];
    check(std::fabs(held - full) < 0.2 && std::fabs(averaged - (full - 9.03)) < 0.3,
          "a tone in one block of eight: peak " + std::to_string(held) + " dBFS, average " +
              std::to_string(averaged) + " dBFS");
}

// ---- web server -------------------------------------------------------------------------------------------

int connect_to(int port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(uint16_t(port));
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (connect(fd, reinterpret_cast<sockaddr *>(&a), sizeof(a)) != 0) return -1;
    timeval timeout{2, 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    return fd;
}

std::string exchange(int fd, const std::string &request, const std::string &until, size_t at_least = 0)
{
    if (!request.empty() && write(fd, request.data(), request.size()) < 0) return "";
    std::string reply;
    char buffer[4096];
    while (reply.find(until) == std::string::npos || reply.size() < at_least) {
        ssize_t n = read(fd, buffer, sizeof(buffer));
        if (n <= 0) break;
        reply.append(buffer, size_t(n));
    }
    return reply;
}

std::string masked_text(const std::string &text)
{
    std::string frame = {char(0x81), char(0x80 | text.size()), 1, 2, 3, 4};
    for (size_t i = 0; i < text.size(); ++i) frame += char(text[i] ^ "\x01\x02\x03\x04"[i % 4]);
    return frame;
}

void web_tests()
{
    WebServer *server = nullptr;
    WebServer::Handler handler;
    handler.text = [&](unsigned client, const std::string &message) {
        std::string upper = message;
        for (char &c : upper) c = char(std::toupper(uint8_t(c)));
        server->send_text(client, upper == "LONG" ? std::string(300, 'x') : upper);
    };
    WebServer web("127.0.0.1:0", "", handler);
    server = &web;
    int port = std::atoi(web.address().substr(web.address().rfind(':') + 1).c_str());
    std::atomic<bool> stop{false};
    std::thread loop([&] { web.run(stop); });
    const std::string upgrade = "GET /ws HTTP/1.1\r\nHost: 127.0.0.1\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                                "Sec-WebSocket-Version: 13\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n";

    int fd = connect_to(port);
    std::string reply = exchange(fd, "GET / HTTP/1.1\r\nHost: x\r\nAccept-Encoding: gzip\r\n\r\n", "\r\n\r\n");
    check(reply.find("200 OK") != std::string::npos && reply.find("Content-Encoding: gzip") != std::string::npos,
          "the page is served, compressed");
    close(fd);

    fd = connect_to(port);
    reply = exchange(fd, upgrade + "\r\n", "\r\n\r\n");
    check(reply.find("101 Switching") != std::string::npos &&
              reply.find("Sec-WebSocket-Accept: s3pPLMBiTxaQ9kYGzzhZRbK+xOo=") != std::string::npos,
          "WebSocket handshake (RFC 6455 example key)");
    reply = exchange(fd, masked_text("hello"), "HELLO");
    check(reply == std::string("\x81\x05HELLO", 7), "a masked text message is answered");
    close(fd);

    fd = connect_to(port);
    reply = exchange(fd, upgrade + "Sec-WebSocket-Extensions: permessage-deflate; client_max_window_bits\r\n\r\n",
                     "\r\n\r\n");
    check(reply.find("permessage-deflate") != std::string::npos, "permessage-deflate is negotiated");
    std::string frame = exchange(fd, masked_text("long"), "", 4);
    frame += exchange(fd, "", "", size_t(uint8_t(frame[1]) & 127) + 2 - frame.size());
    bool compressed = frame.size() >= 2 && (uint8_t(frame[0]) & 0x40);
    std::string payload = frame.substr(2) + std::string("\x00\x00\xff\xff", 4), text(400, '\0');
    z_stream z{};
    inflateInit2(&z, -15);
    z.next_in = (Bytef *)payload.data();
    z.avail_in = uInt(payload.size());
    z.next_out = (Bytef *)&text[0];
    z.avail_out = uInt(text.size());
    inflate(&z, Z_SYNC_FLUSH);
    text.resize(z.total_out);
    inflateEnd(&z);
    check(compressed && text == std::string(300, 'x'), "a long message arrives compressed and inflates");
    close(fd);

    fd = connect_to(port);
    reply = exchange(fd, upgrade + "Origin: http://elsewhere.example\r\n\r\n", "\r\n\r\n");
    check(reply.find("403") != std::string::npos, "a page from another site cannot connect");
    close(fd);

    stop = true;
    loop.join();
}

}  // namespace

// The web UI files the server would carry; here a stand-in page.
static const unsigned char page[] = "<!doctype html><title>test</title>";
const EmbeddedFile kWebFiles[] = {{"/index.html", page, sizeof(page) - 1}};
const size_t kWebFileCount = 1;

int main()
{
    spectrum_tests();
    web_tests();
    std::printf(failures ? "%d FAILED\n" : "all host tests passed\n", failures);
    return failures ? 1 : 0;
}
