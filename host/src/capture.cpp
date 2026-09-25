// capture:   stream IQ from the ESP32-S3 through the FPGA to a file, checking
//            every record and reconciling the counts of all three stages.
// calibrate: sweep the FPGA's link sampling phase and centre it.
// status:    identities, readiness and the statistics of the last run.
#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "capture.h"
#include "control.h"
#include "link.h"
#include "receiver.h"

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

namespace {

std::atomic<bool> interrupted{false};

void on_signal(int)
{
    interrupted = true;
    std::signal(SIGINT, SIG_DFL);  // a second Ctrl-C kills
}

uint64_t join64(uint32_t low, uint32_t high) { return uint64_t(high) << 32 | low; }

}  // namespace

std::string describe_flags(uint32_t flags)
{
    std::string text;
    auto add = [&](uint32_t bit, const char *name) {
        text += (flags & bit) ? " " : " !";
        text += name;
    };
    add(FPGA_FLAG_REFERENCE_ON, "reference");
    add(FPGA_FLAG_PHASE_READY, "phase");
    add(FPGA_FLAG_DESKEW_READY, "deskew");
    add(FPGA_FLAG_DDR_READY, "ddr");
    add(FPGA_FLAG_ARMED, "armed");
    add(FPGA_FLAG_STREAM_ENDED, "ended");
    add(FPGA_FLAG_STREAM_OPEN, "open");
    return text;
}

const char *esp_failure(uint32_t status)
{
    switch (status) {
    case 0: return "none";
    case ESP_FAIL_OWNER: return "capture bank ownership changed";
    case ESP_FAIL_LATE_POLL: return "bank overran while waiting";
    case ESP_FAIL_LATE_SWITCH: return "bank overran at the switch";
    case ESP_FAIL_END: return "unit end not found";
    case ESP_FAIL_START: return "unit start mismatch";
    case ESP_FAIL_LENGTH: return "unit length out of range";
    case ESP_FAIL_REUSE: return "bank still transmitting";
    case ESP_FAIL_LATE_PREPARE: return "next bank prepared too late";
    default: return "unknown";
    }
}

namespace {

// Receives USB transfers into the decoder on its own thread.
class Receiver {
public:
    Receiver(Ft600 &usb, StreamDecoder &decoder) : usb_(usb), decoder_(decoder)
    {
        thread_ = std::thread([this] { run(); });
    }
    ~Receiver() { stop(); }

    // Ends reception: wakes the thread if it is waiting on USB and joins it.
    void stop()
    {
        if (!thread_.joinable()) return;
        cancelled_ = true;
        if (!done_) usb_.abort();
        thread_.join();
    }
    bool done() const { return done_; }
    uint64_t bytes() const { return bytes_; }
    std::string error() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return error_;
    }

private:
    void run()
    {
        try {
            std::vector<uint8_t> data;
            while (!decoder_.ended() && !cancelled_) {
                if (!usb_.next(data)) continue;
                bytes_ += data.size();
                decoder_.add(data.data(), data.size());
            }
        } catch (const std::exception &e) {
            if (!cancelled_) {
                std::lock_guard<std::mutex> lock(mutex_);
                error_ = e.what();
            }
        }
        done_ = true;
    }

    Ft600 &usb_;
    StreamDecoder &decoder_;
    std::thread thread_;
    std::atomic<bool> done_{false}, cancelled_{false};
    std::atomic<uint64_t> bytes_{0};
    mutable std::mutex mutex_;
    std::string error_;
};

// Ctrl-C and SIGTERM request a clean stop while installed; a second Ctrl-C kills.
class SignalGuard {
public:
    SignalGuard()
    {
        interrupted = false;
        std::signal(SIGINT, on_signal);
        std::signal(SIGTERM, on_signal);
    }
    ~SignalGuard()
    {
        std::signal(SIGINT, SIG_DFL);
        std::signal(SIGTERM, SIG_DFL);
    }
};

// Runs a step, keeping the first error.
template <typename Step>
void attempt(std::string &failure, Step step)
{
    try {
        step();
    } catch (const std::exception &e) {
        if (failure.empty()) failure = e.what();
    }
}

// The FPGA's single stream: opened by ARM, released again however the run
// ends, so the next ARM is accepted.
class OpenStream {
public:
    explicit OpenStream(ControlPort &fpga) : fpga_(fpga) { fpga_.command(FPGA_ARM); }
    ~OpenStream()
    {
        try {
            fpga_.command(FPGA_RELEASE);
        } catch (const std::exception &) {
            // Still open: the next run finds it and closes it.
        }
    }
    OpenStream(const OpenStream &) = delete;
    OpenStream &operator=(const OpenStream &) = delete;

private:
    ControlPort &fpga_;
};

}  // namespace

Devices::Devices(const Options &options, bool require_radio)
    : esp(find_esp_port(options), CTL_NODE_ESP), fpga(find_fpga_port(options), CTL_NODE_FPGA),
      usb(options.ft600_serial)
{
    if (esp.command(CTL_INFO, 0) != CTL_ESP_FIRMWARE_ID)
        throw std::runtime_error(esp.path() + " is not running this iqstream's ESP firmware");
    if (fpga.command(CTL_INFO, 0) != CTL_FPGA_FIRMWARE_ID)
        throw std::runtime_error(fpga.path() + " is not running this iqstream's FPGA image");
    if (uint32_t radio = esp.command(CTL_STATUS, ESP_STAT_RADIO); require_radio && radio != ESP_RADIO_OK)
        throw std::runtime_error("ESP radio initialisation failed (code " + std::to_string(radio) + ")");
    uint32_t required = FPGA_FLAG_REFERENCE_ON | FPGA_FLAG_DESKEW_READY | FPGA_FLAG_DDR_READY;
    if (uint32_t flags = fpga.command(CTL_STATUS, FPGA_STAT_FLAGS); (flags & required) != required)
        throw std::runtime_error("FPGA not ready:" + describe_flags(flags));
}

uint32_t RunResult::link_errors() const
{
    return fpga[FPGA_STAT_FRAMING0] + fpga[FPGA_STAT_FRAMING1] + fpga[FPGA_STAT_CHECKSUM0] +
           fpga[FPGA_STAT_CHECKSUM1] + fpga[FPGA_STAT_END_MARK0] + fpga[FPGA_STAT_END_MARK1];
}

RunResult run_capture(Devices &devices, const RunRequest &request)
{
    const unsigned seconds = request.seconds;
    const bool show_progress = request.show_progress;
    ControlPort &esp = devices.esp;
    ControlPort &fpga = devices.fpga;
    Ft600 &usb = devices.usb;
    RunResult result;
    std::string failure;

    // This process holds the control ports exclusively, so a stream that is
    // still open belongs to an earlier process that did not finish.
    if (fpga.command(CTL_STATUS, FPGA_STAT_FLAGS) & FPGA_FLAG_STREAM_OPEN) {
        std::fprintf(stderr, "closing a stream left open by an earlier iqstream process\n");
        fpga.command(FPGA_RELEASE);
    }

    // Link lines driven low before the receiver runs; then a clean pipeline.
    esp.command(ESP_OUTPUTS, 1);
    OpenStream stream(fpga);
    if (size_t stale = usb.drain())
        std::fprintf(stderr, "discarded %zu stale bytes from the FT600\n", stale);

    auto decoder = std::make_unique<StreamDecoder>(request.format, request.output, request.threads,
                                                   request.samples);
    decoder->limit_to_elapsed(Clock::now(), LINK_PAIRS_PER_SECOND, 2.0);
    usb.start();
    {
        Receiver receiver(usb, *decoder);
        std::optional<SignalGuard> signals;
        if (request.catch_signals) signals.emplace();
        bool run_started = false;
        try {
            uint16_t run = esp.send(ESP_RUN, seconds);
            run_started = true;
            auto started = Clock::now(), last_report = started, stop_sent_at = started;
            uint64_t last_bytes = 0;
            bool stop_sent = false;
            uint16_t stop_sequence = 0;
            ControlPort::Reply run_reply;

            for (;;) {
                if (esp.receive(ESP_RUN, run, run_reply, request.progress ? 100ms : 200ms)) break;
                double elapsed = std::chrono::duration<double>(Clock::now() - started).count();
                bool receiver_stopped = !receiver.error().empty() || receiver.done();
                if (request.progress) request.progress({elapsed, receiver.bytes(), decoder->progress()});
                bool stop_wanted = (request.catch_signals && interrupted) || (request.stop && request.stop());
                if ((stop_wanted || receiver_stopped) && !stop_sent) {
                    stop_sequence = esp.send(ESP_STOP, 0);
                    stop_sent = true;
                    stop_sent_at = Clock::now();
                }
                if ((seconds && elapsed > seconds + 30.0) || (stop_sent && Clock::now() - stop_sent_at > 10s))
                    throw std::runtime_error("the ESP did not finish its run");
                if (show_progress && Clock::now() - last_report >= 1s) {
                    double interval = std::chrono::duration<double>(Clock::now() - last_report).count();
                    uint64_t bytes = receiver.bytes();
                    std::fprintf(stderr, "\r%7.1f s  %7.2f MB/s  %9.3f Mpairs", elapsed,
                                 (bytes - last_bytes) / interval / 1e6, decoder->progress() / 1e6);
                    std::fflush(stderr);
                    last_report = Clock::now();
                    last_bytes = bytes;
                }
            }
            if (show_progress) std::fprintf(stderr, "\n");
            if (run_reply.status != CTL_OK)
                failure = "ESP run failed: " + control_status_name(run_reply.status);

            // Stop the receiver; the FPGA then flushes and ends the stream.
            fpga.command(FPGA_STOP);
            if (stop_sent) {
                ControlPort::Reply ignored;
                esp.receive(ESP_STOP, stop_sequence, ignored, 2000ms);
            }
            esp.command(ESP_OUTPUTS, 0);

            // Wait for the END record while data keeps arriving (a slow
            // consumer may still be draining a large DDR backlog).
            auto last_progress = Clock::now();
            uint64_t seen = receiver.bytes();
            while (!receiver.done()) {
                std::this_thread::sleep_for(20ms);
                if (receiver.bytes() != seen) {
                    seen = receiver.bytes();
                    last_progress = Clock::now();
                } else if (Clock::now() - last_progress > 30s) {
                    throw std::runtime_error("the stream did not end");
                }
            }
        } catch (const std::exception &e) {
            if (failure.empty()) failure = e.what();
            if (show_progress) std::fprintf(stderr, "\n");
            // Leave both devices stopped and safe.
            attempt(failure, [&] { fpga.command(FPGA_STOP); });
            if (run_started)
                attempt(failure, [&] {
                    ControlPort::Reply ignored;
                    uint16_t stop = esp.send(ESP_STOP, 0);
                    esp.receive(ESP_STOP, stop, ignored, 2000ms);
                });
            attempt(failure, [&] { esp.command(ESP_OUTPUTS, 0); });
        }
        receiver.stop();
        if (failure.empty() && !receiver.error().empty()) failure = receiver.error();
    }
    usb.stop();

    if (failure.empty()) attempt(failure, [&] { result.totals = decoder->finish(); });
    decoder.reset();  // waits for the writer before the caller closes the output
    result.require(failure.empty(), failure);

    std::string stats_error;
    attempt(stats_error, [&] {
        for (unsigned i = 0; i < ESP_STAT_COUNT; ++i) result.esp[i] = esp.command(CTL_STATUS, i);
        for (unsigned i = 0; i < FPGA_STAT_COUNT; ++i) result.fpga[i] = fpga.command(CTL_STATUS, i);
    });
    if (!stats_error.empty()) {
        result.require(false, "cannot read statistics: " + stats_error);
        return result;
    }

    const auto &e = result.esp;
    const auto &f = result.fpga;
    uint64_t esp_pairs = join64(e[ESP_STAT_PAIRS0_LO], e[ESP_STAT_PAIRS0_HI]) +
                         join64(e[ESP_STAT_PAIRS1_LO], e[ESP_STAT_PAIRS1_HI]);
    uint64_t fpga_pairs = join64(f[FPGA_STAT_PAIRS_LO], f[FPGA_STAT_PAIRS_HI]);
    uint64_t fpga_lost = join64(f[FPGA_STAT_LOST_PAIRS_LO], f[FPGA_STAT_LOST_PAIRS_HI]);
    result.seconds = join64(e[ESP_STAT_TICKS_LO], e[ESP_STAT_TICKS_HI]) / 16e6;

    result.require(e[ESP_STAT_STATUS] == 0,
                   std::string("ESP acquisition failure: ") + esp_failure(e[ESP_STAT_STATUS]) + " (lane " +
                       std::to_string(e[ESP_STAT_FAIL_LANE]) + ", detail " +
                       std::to_string(e[ESP_STAT_DETAIL]) + ")");
    result.require(result.totals.ended, "no END record received");
    // Every pair the ESP sent was either delivered or accounted for as lost.
    result.require(fpga_pairs + fpga_lost == esp_pairs,
                   "FPGA pairs " + std::to_string(fpga_pairs) + " + lost " + std::to_string(fpga_lost) +
                       " != ESP pairs " + std::to_string(esp_pairs));
    result.require(result.totals.pairs == fpga_pairs, "received pairs " + std::to_string(result.totals.pairs) +
                                                          " != FPGA pairs " + std::to_string(fpga_pairs));
    result.require(result.totals.lost() == fpga_lost, "gaps in the stream differ from the FPGA's lost pairs");
    result.require(result.totals.crc == f[FPGA_STAT_STREAM_CRC], "stream CRC differs from the FPGA's");
    result.require(result.totals.records == f[FPGA_STAT_RECORDS], "record count differs from the FPGA's");
    result.require(uint64_t(f[FPGA_STAT_UNITS0]) + f[FPGA_STAT_UNITS1] - f[FPGA_STAT_DISCARDED_UNITS] +
                           f[FPGA_STAT_LOST_UNITS] ==
                       uint64_t(e[ESP_STAT_UNITS0]) + e[ESP_STAT_UNITS1],
                   "units received and lost by the FPGA differ from units sent");
    // Rejected headers cost whole units, which are reported as gaps; these
    // errors mean corrupt data was delivered.
    const std::pair<unsigned, const char *> error_counters[] = {
        {FPGA_STAT_CHECKSUM0, "lane 0 checksum errors"},
        {FPGA_STAT_CHECKSUM1, "lane 1 checksum errors"},
        {FPGA_STAT_END_MARK0, "lane 0 end-marker errors"},
        {FPGA_STAT_END_MARK1, "lane 1 end-marker errors"},
        {FPGA_STAT_SAMPLE_OVERFLOW, "capture FIFO overflows"},
        {FPGA_STAT_REORDER_OVERFLOW0, "lane 0 reorder overflows"},
        {FPGA_STAT_REORDER_OVERFLOW1, "lane 1 reorder overflows"},
        {FPGA_STAT_RING_INPUT_OVERFLOW, "DDR input overflows"},
        {FPGA_STAT_RING_OUTPUT_OVERFLOW, "DDR output overflows"},
        {FPGA_STAT_RING_ERRORS, "DDR ring errors"},
        {FPGA_STAT_USB_UNDERRUNS, "USB underruns"},
    };
    for (const auto &[index, name] : error_counters)
        result.require(f[index] == 0, std::to_string(f[index]) + " " + name);
    return result;
}

namespace {

// Moves the link sampling phase and waits until the FPGA reports it there.
void set_phase(ControlPort &fpga, unsigned phase)
{
    fpga.command(FPGA_PHASE, phase);
    auto deadline = Clock::now() + 5s;
    while (!(fpga.command(CTL_STATUS, FPGA_STAT_FLAGS) & FPGA_FLAG_PHASE_READY) ||
           fpga.command(CTL_STATUS, FPGA_STAT_PHASE) != phase) {
        if (Clock::now() > deadline) throw std::runtime_error("the sampling phase did not settle");
        std::this_thread::sleep_for(10ms);
    }
}

}  // namespace

int command_capture(const Options &options)
{
    OutputFormat format = OutputFormat::None;
    std::FILE *output = nullptr;
    if (!options.output.empty()) {
        const std::string &name = options.output;
        std::string kind = options.format.empty()
                               ? (name.size() > 5 && name.substr(name.size() - 5) == ".cs16" ? "cs16" : "iqc")
                               : options.format;
        if (kind != "iqc" && kind != "cs16") throw std::runtime_error("--format must be iqc or cs16");
        format = kind == "cs16" ? OutputFormat::Cs16 : OutputFormat::Iqc;
        output = name == "-" ? stdout : std::fopen(name.c_str(), "wb");
        if (!output) throw std::runtime_error("cannot create " + name);
    }

    Devices devices(options);
    if (options.phase >= 0) set_phase(devices.fpga, unsigned(options.phase));
    RunRequest request;
    request.seconds = options.seconds;
    request.format = format;
    request.output = output;
    request.threads = options.threads;
    request.show_progress = true;
    RunResult result = run_capture(devices, request);
    if (output && output != stdout && std::fclose(output) != 0)
        result.require(false, "cannot write " + options.output);

    const auto &e = result.esp;
    const auto &f = result.fpga;
    double megabytes = result.totals.record_bytes / 1e6;
    uint64_t pairs = result.totals.pairs;
    std::fprintf(stderr,
                 "%.3f s, %llu pairs (%.3f Msps), %llu records, %.1f MB (%.1f MB/s, %.2f bytes/pair)%s\n"
                 "ring peak %.1f MiB, ESP service peak %.1f/%.1f us, USB stall peak %.2f ms\n",
                 result.seconds, (unsigned long long)pairs,
                 result.seconds > 0 ? pairs / result.seconds / 1e6 : 0.0,
                 (unsigned long long)result.totals.records, megabytes,
                 result.seconds > 0 ? megabytes / result.seconds : 0.0,
                 pairs ? double(result.totals.record_bytes) / pairs : 0.0,
                 e[ESP_STAT_STOPPED_BY_HOST] ? ", stopped by request" : "",
                 f[FPGA_STAT_RING_PEAK] * 16.0 / (1 << 20), e[ESP_STAT_SERVICE_MAX0] / 240.0,
                 e[ESP_STAT_SERVICE_MAX1] / 240.0, f[FPGA_STAT_USB_MAX_STALL] / 1e5);
    if (!result.failures.empty()) {
        std::fprintf(stderr, "FAILED\n%s", result.failures.c_str());
        return 1;
    }
    if (result.totals.gaps) {
        uint64_t lost = result.totals.lost();
        std::fprintf(stderr,
                     "COMPLETE WITH GAPS: %llu gaps, %llu pairs lost (%.2g%%, %u rejected unit headers);\n"
                     "every delivered pair verified, record indices and cs16 output keep sample time\n",
                     (unsigned long long)result.totals.gaps, (unsigned long long)lost,
                     100.0 * lost / double(result.totals.span),
                     f[FPGA_STAT_FRAMING0] + f[FPGA_STAT_FRAMING1]);
        return 3;
    }
    std::fprintf(stderr, "OK: every pair received and verified\n");
    return 0;
}

int command_calibrate(const Options &options)
{
    Devices devices(options);
    unsigned original = devices.fpga.command(CTL_STATUS, FPGA_STAT_PHASE);
    const unsigned step = options.phase_step;
    std::vector<unsigned> phases;
    std::vector<bool> passed;
    std::fprintf(stderr, "phase  link errors  result\n");
    for (unsigned phase = 0; phase < FPGA_PHASE_STEPS && !interrupted; phase += step) {
        set_phase(devices.fpga, phase);
        RunRequest request;
        request.seconds = 1;
        request.threads = options.threads;
        RunResult result = run_capture(devices, request);
        bool ok = result.failures.empty() && result.totals.gaps == 0;
        std::fprintf(stderr, "%5u  %11u  %s\n", phase, result.link_errors(), ok ? "ok" : "FAILED");
        phases.push_back(phase);
        passed.push_back(ok);
    }

    // Longest run of passing points, treating the phase range as circular.
    size_t n = phases.size(), best_start = 0, best_length = 0;
    for (size_t start = 0; start < n; ++start) {
        if (!passed[start] || passed[(start + n - 1) % n]) continue;
        size_t length = 0;
        while (length < n && passed[(start + length) % n]) ++length;
        if (length > best_length) {
            best_start = start;
            best_length = length;
        }
    }
    if (!best_length) {
        set_phase(devices.fpga, original);
        std::fprintf(stderr, "no bounded passing window found; phase left at %u\n", original);
        return 1;
    }
    unsigned first = phases[best_start], last = phases[(best_start + best_length - 1) % n];
    unsigned width = unsigned(best_length - 1) * step;
    unsigned centre = (first + width / 2) % FPGA_PHASE_STEPS;
    set_phase(devices.fpga, centre);
    std::fprintf(stderr,
                 "passing window %u..%u (%u steps, %.2f ns); phase set to its centre, %u.\n"
                 "It lasts until the FPGA is reloaded; pass `--phase %u` to `iqstream load`, or\n"
                 "set DEFAULT_PHASE in fpga/rtl/iqstream_top.v to make it the built-in value.\n",
                 first, last, width, width * 4.1667 / FPGA_PHASE_STEPS, centre, centre);
    return 0;
}

int command_status(const Options &options)
{
    ControlPort esp(find_esp_port(options), CTL_NODE_ESP);
    ControlPort fpga(find_fpga_port(options), CTL_NODE_FPGA);
    uint32_t esp_id = esp.command(CTL_INFO, 0), fpga_id = fpga.command(CTL_INFO, 0);
    uint32_t mac_low = esp.command(CTL_INFO, 1), mac_high = esp.command(CTL_INFO, 2);
    std::printf("ESP32-S3  %s  firmware %08x%s  MAC %02x:%02x:%02x:%02x:%02x:%02x  radio %u\n",
                esp.path().c_str(), esp_id, esp_id == CTL_ESP_FIRMWARE_ID ? "" : " (unexpected)",
                mac_low & 255, mac_low >> 8 & 255, mac_low >> 16 & 255, mac_low >> 24, mac_high & 255,
                mac_high >> 8 & 255, esp.command(CTL_STATUS, ESP_STAT_RADIO));
    if (esp_id == CTL_ESP_FIRMWARE_ID)
        std::printf("receiver  %s\n", describe_receiver(read_receiver(esp)).c_str());
    std::printf("FPGA      %s  image %08x%s  flags:%s  phase %u\n", fpga.path().c_str(), fpga_id,
                fpga_id == CTL_FPGA_FIRMWARE_ID ? "" : " (unexpected)",
                describe_flags(fpga.command(CTL_STATUS, FPGA_STAT_FLAGS)).c_str(),
                fpga.command(CTL_STATUS, FPGA_STAT_PHASE));

    static const char *esp_names[ESP_STAT_LO_HZ] = {
        "status", "detail", "fail lane", "units 0", "units 1", "pairs 0 lo", "pairs 0 hi",
        "pairs 1 lo", "pairs 1 hi", "ticks lo", "ticks hi", "service max 0", "service max 1",
        "radio", "stopped by host"};
    static const char *fpga_names[FPGA_STAT_COUNT] = {
        "flags", "pairs lo", "pairs hi", "stream crc", "records", "units 0", "units 1",
        "framing 0", "framing 1", "checksum 0", "checksum 1", "end mark 0", "end mark 1",
        "sample overflow", "reorder overflow 0", "reorder overflow 1", "lost units",
        "ring input overflow", "ring output overflow", "ring errors", "ring used", "ring peak",
        "usb underruns", "usb max stall", "reorder peak 0", "reorder peak 1", "phase",
        "lost pairs lo", "lost pairs hi", "discarded units"};
    std::printf("\nlast run, ESP:\n");
    for (unsigned i = 0; i < ESP_STAT_LO_HZ; ++i)
        std::printf("  %-22s %u\n", esp_names[i], esp.command(CTL_STATUS, i));
    std::printf("last run, FPGA:\n");
    for (unsigned i = 0; i < FPGA_STAT_COUNT; ++i)
        std::printf("  %-22s %u\n", fpga_names[i], fpga.command(CTL_STATUS, i));
    return 0;
}

int command_set(const Options &options)
{
    if (options.settings.empty()) throw std::runtime_error("set needs at least one NAME=VALUE");
    std::vector<ReceiverChange> changes;
    for (const auto &text : options.settings) changes.push_back(parse_receiver_setting(text));
    ControlPort esp(find_esp_port(options), CTL_NODE_ESP);
    if (esp.command(CTL_INFO, 0) != CTL_ESP_FIRMWARE_ID)
        throw std::runtime_error(esp.path() + " is not running this iqstream's ESP firmware");
    for (const auto &change : changes) apply_receiver_change(esp, change);
    std::printf("%s\n", describe_receiver(read_receiver(esp)).c_str());
    return 0;
}
