// serve: a live spectrum and waterfall for web browsers, with receiver
// control and live statistics.
//
// Threads:
//   capture   owns the devices: one verified capture run after another,
//             receiver settings applied between runs, live FPGA statistics
//   spectrum  decoded samples to spectrum frames (SpectrumEngine)
//   web       the HTTP and WebSocket loop (the calling thread): the client's
//             view, flow control and status
// plus each run's own USB and decoding threads.
//
// One browser is served at a time. A new connection takes over from the
// previous one, which is closed with code 4001, so a client on a flaky link
// can always reconnect at once instead of waiting for its old connection to
// time out. Streaming pauses once no client has been connected for a while.
//
// Every frame is fitted to what the client asked for (frequency range and
// number of bins) and sent only while fewer than a few frames are
// unacknowledged, so a slow or distant client gets fewer frames rather than
// a growing backlog. The frames it misses are merged into the next one it
// gets (by maximum in peak mode, by averaging dB otherwise), so short
// signals are never lost from its display.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <deque>
#include <mutex>
#include <set>
#include <thread>

#include "capture.h"
#include "json.h"
#include "options.h"
#include "receiver.h"
#include "spectrum.h"
#include "web.h"

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

namespace {

constexpr int kProtocolVersion = 1;
constexpr auto kIdleGrace = 10s;           // keep streaming this long after the client leaves
constexpr uint16_t kReplaced = 4001;       // close code: another client took over
constexpr size_t kSocketBacklog = 32768;   // bytes queued for a client before frames wait
constexpr unsigned kMaxBins = 8192;
constexpr float kStepDb = 0.5f;            // quantisation of the frames sent

std::atomic<bool> stop_requested{false};
void on_signal(int) { stop_requested = true; }

double wall_ms()
{
    return std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
}

const char *const kFpgaNames[FPGA_STAT_COUNT] = {
    "flags", "pairs_lo", "pairs_hi", "stream_crc", "records", "units0", "units1", "framing0", "framing1",
    "checksum0", "checksum1", "end_mark0", "end_mark1", "sample_overflow", "reorder_overflow0",
    "reorder_overflow1", "lost_units", "ring_input_overflow", "ring_output_overflow", "ring_errors",
    "ring_used", "ring_peak", "usb_underruns", "usb_max_stall", "reorder_peak0", "reorder_peak1", "phase",
    "lost_pairs_lo", "lost_pairs_hi", "discarded_units"};

// What the capture thread publishes for the web thread.
struct Published {
    std::string state = "starting";  // waiting, stopped, starting, running, reconfiguring, error
    std::string message;             // why it is waiting, or the last error
    std::string esp_port, fpga_port, ft600;
    bool receiver_valid = false;
    ReceiverState receiver;
    bool fpga_valid = false;
    std::array<uint32_t, FPGA_STAT_COUNT> fpga{};

    uint64_t run = 0;                 // current (or last) run
    double run_seconds = 0, usb_rate = 0, pair_rate = 0;
    uint64_t usb_bytes = 0, pairs = 0;

    bool last_valid = false;          // the last completed run
    std::string last_result, last_failures;
    double last_seconds = 0;
    uint64_t last_pairs = 0, last_gaps = 0, last_lost = 0, last_bytes = 0;
    std::array<uint32_t, ESP_STAT_COUNT> last_esp{};
    uint64_t runs_ok = 0, runs_gaps = 0, runs_failed = 0;
};

// The browser being served.
struct Viewer {
    double start_hz = 0, stop_hz = 0;  // requested range; stop <= start: the whole span
    unsigned bins = 1024;
    bool auto_bins = true;             // fewer bins when the link cannot carry these
    unsigned bins_cap = 1024;          // that limit (0: none); links start modestly
    double max_fps = 0;                // 0: every frame produced
    double max_kbps = 0;               // 0: no limit

    // Frames sent and not yet acknowledged, and the window that bounds
    // them: it grows while round trips stay near those of an empty link and
    // shrinks when they show a queue, so frames are never late for long.
    struct Sent {
        uint32_t sequence;
        Clock::time_point time;
        uint64_t bytes;
        bool alone;                    // nothing else was in flight
    };
    uint32_t sequence = 0;
    std::deque<Sent> in_flight;
    uint64_t in_flight_bytes = 0, delivered = 0, last_frame_bytes = 0;
    double window = 0;                 // bytes allowed in flight; 0 until the first frame
    Clock::time_point cut_at;          // last reduction
    double rtt = 0, rtt_min = 0;       // seconds
    Clock::time_point rtt_min_at = Clock::now();
    bool probing = false;              // one frame at a time, to measure rtt_min again
    Clock::time_point last_send;
    double tokens = 0;                 // allowance under max_kbps, bytes
    Clock::time_point tokens_at = Clock::now();

    std::shared_ptr<const SpectrumFrame> pending;  // next frame to send
    std::shared_ptr<SpectrumFrame> merged;         // `pending`, when it is this viewer's own merge
    unsigned pending_frames = 0;                   // frames combined into it

    Clock::time_point mark = Clock::now();         // per-second statistics
    unsigned frames_sent = 0, frames_merged = 0;
    uint64_t bytes_mark = 0, delivered_mark = 0;
    double fps = 0, kbps = 0, merged_rate = 0;
    double delivered_rate = 0;         // frame bytes acknowledged per second
    Clock::time_point last_status;
    size_t last_status_bytes = 0;
    std::map<std::string, std::string> sections;  // status sections as last sent
    Clock::time_point sections_at;                // when they were all last sent
};

class Service {
public:
    explicit Service(const Options &options)
        : options_(options),
          web_(options.listen, options.web_dir,
               {[this](unsigned c) { on_open(c); }, [this](unsigned c, const std::string &m) { on_text(c, m); },
                [this](unsigned c) { on_close(c); }, [this] { on_wake(); }, [this] { on_tick(); }}),
          engine_([this](std::shared_ptr<const SpectrumFrame> frame) {
              {
                  std::lock_guard<std::mutex> lock(frames_mutex_);
                  frames_.push_back(std::move(frame));
                  while (frames_.size() > 64) frames_.pop_front();
              }
              web_.wake();
          })
    {
        capture_ = std::thread([this] { capture_loop(); });
    }

    ~Service()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            shutdown_ = true;
        }
        changed_.notify_all();
        capture_.join();
    }

    const std::string &address() const { return web_.address(); }
    void run(const std::atomic<bool> &stop) { web_.run(stop); }

private:
    // ---- capture thread ------------------------------------------------------------------------

    // Updates what the web thread sees; an urgent change (state, settings,
    // a finished run) goes to the clients at once, the rest with the next
    // periodic status.
    template <typename Change>
    void publish(Change change, bool urgent = true)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            change(published_);
            published_changed_ = published_changed_ || urgent;
        }
        if (urgent) web_.wake();
    }

    void notice(const std::string &text)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            notices_.push_back(text);
        }
        web_.wake();
    }

    bool want_run_locked() const
    {
        return running_ && (connected_ || Clock::now() - last_client_ < kIdleGrace);
    }

    // Waits until a request changes (or the timeout ends).
    void wait_for_change(std::chrono::milliseconds timeout)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        changed_.wait_for(lock, timeout, [this] { return shutdown_ || !settings_.empty() || request_changed_; });
        request_changed_ = false;
    }

    bool shutting_down()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return shutdown_;
    }

    // Reads every FPGA statistic. A two-word value is read high, low, high,
    // and its low word again if the high word moved in between.
    void poll_fpga(Devices &devices)
    {
        std::array<uint32_t, FPGA_STAT_COUNT> values{};
        for (unsigned i = 0; i < FPGA_STAT_COUNT; ++i) values[i] = devices.fpga.command(CTL_STATUS, i);
        for (unsigned low : {FPGA_STAT_PAIRS_LO, FPGA_STAT_LOST_PAIRS_LO}) {
            uint32_t high = devices.fpga.command(CTL_STATUS, low + 1);
            values[low] = devices.fpga.command(CTL_STATUS, low);
            values[low + 1] = devices.fpga.command(CTL_STATUS, low + 1);
            if (values[low + 1] != high) values[low] = devices.fpga.command(CTL_STATUS, low);
        }
        publish(
            [&](Published &p) {
                p.fpga = values;
                p.fpga_valid = true;
            },
            false);
    }

    void apply_settings(Devices &devices)
    {
        std::vector<std::pair<std::string, std::string>> batch;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            batch.swap(settings_);
        }
        if (batch.empty()) return;
        publish([](Published &p) { p.state = "reconfiguring"; });
        for (const auto &[name, setting] : batch) {
            try {
                apply_receiver_change(devices.esp, parse_receiver_setting(setting));
            } catch (const ControlError &) {
                throw;  // the ESP stopped answering
            } catch (const std::exception &e) {
                notice(e.what());
            }
        }
        ReceiverState receiver = read_receiver(devices.esp);
        publish([&](Published &p) {
            p.receiver = receiver;
            p.receiver_valid = true;
        });
    }

    // Without the devices there is no receiver to set: settings asked for
    // meanwhile are refused rather than kept.
    void lose_devices(const char *state, const std::string &why)
    {
        std::vector<std::pair<std::string, std::string>> dropped;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            dropped.swap(settings_);
        }
        if (!dropped.empty()) notice("the receiver is not available; settings were not applied");
        publish([&](Published &p) {
            p.state = state;
            p.message = why;
            p.receiver_valid = p.fpga_valid = false;
        });
        wait_for_change(2000ms);
    }

    void capture_loop()
    {
        uint64_t run_id = 0;
        while (!shutting_down()) {
            std::unique_ptr<Devices> devices;
            try {
                devices = std::make_unique<Devices>(options_, false);
            } catch (const std::exception &e) {
                lose_devices("waiting", e.what());
                continue;
            }
            unsigned failures = 0;
            try {
                ReceiverState receiver = read_receiver(devices->esp);
                publish([&](Published &p) {
                    p.esp_port = devices->esp.path();
                    p.fpga_port = devices->fpga.path();
                    p.ft600 = devices->usb.serial();
                    p.receiver = receiver;
                    p.receiver_valid = true;
                    p.message.clear();
                });
                while (!shutting_down()) {
                    apply_settings(*devices);
                    bool want;
                    uint32_t radio;
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        want = want_run_locked();
                        radio = published_.receiver.radio;
                    }
                    if (radio != ESP_RADIO_OK || !want) {
                        // A setting that failed and could not be undone leaves the
                        // receiver unconfigured until another setting works.
                        std::string state = radio != ESP_RADIO_OK ? "error" : "stopped", message;
                        if (radio != ESP_RADIO_OK)
                            message = "the ESP's receiver is not configured (code " + std::to_string(radio) +
                                      "): change a setting, or reload the ESP";
                        bool news;
                        {
                            std::lock_guard<std::mutex> lock(mutex_);
                            news = published_.state != state || (!message.empty() && published_.message != message);
                        }
                        publish(
                            [&](Published &p) {
                                p.state = state;
                                if (!message.empty()) p.message = message;
                            },
                            news);
                        poll_fpga(*devices);
                        wait_for_change(500ms);
                        continue;
                    }
                    run_once(*devices, ++run_id, failures);
                    if (failures) {
                        if (failures >= 3) throw std::runtime_error("three runs in a row failed; reopening the devices");
                        wait_for_change(std::chrono::milliseconds(500 * failures));
                    }
                }
            } catch (const std::exception &e) {
                devices.reset();
                lose_devices("error", e.what());
            }
        }
    }

    void run_once(Devices &devices, uint64_t run_id, unsigned &failures)
    {
        ReceiverState receiver;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            receiver = published_.receiver;
        }
        engine_.begin_run(run_id, receiver.lo_hz, receiver.sample_rate());
        publish([&](Published &p) {
            p.state = "running";
            p.run = run_id;
            p.run_seconds = p.usb_rate = p.pair_rate = 0;
            p.usb_bytes = p.pairs = 0;
            if (!failures) p.message.clear();
        });

        auto last_poll = Clock::now();
        RunProgress rate_mark{0, 0, 0};
        RunRequest request;
        request.threads = options_.threads;
        request.catch_signals = false;
        request.stop = [this] {
            std::lock_guard<std::mutex> lock(mutex_);
            return shutdown_ || !settings_.empty() || !want_run_locked();
        };
        request.progress = [&](const RunProgress &progress) {
            if (Clock::now() - last_poll >= 200ms) {
                poll_fpga(devices);
                last_poll = Clock::now();
            }
            double interval = progress.seconds - rate_mark.seconds;
            publish(
                [&](Published &p) {
                    p.run_seconds = progress.seconds;
                    p.usb_bytes = progress.usb_bytes;
                    p.pairs = progress.pairs;
                    if (interval >= 0.5) {
                        p.usb_rate = (progress.usb_bytes - rate_mark.usb_bytes) / interval;
                        p.pair_rate = (progress.pairs - rate_mark.pairs) / interval;
                    }
                },
                false);
            if (interval >= 0.5) rate_mark = progress;
        };
        request.samples = [this](uint64_t first, std::vector<int16_t> &&samples) {
            engine_.add(first, std::move(samples));
        };
        RunResult result = run_capture(devices, request);
        poll_fpga(devices);

        bool failed = !result.failures.empty();
        failures = failed ? failures + 1 : 0;
        publish([&](Published &p) {
            p.last_valid = true;
            p.last_result = failed ? "failed" : result.totals.gaps ? "gaps" : "ok";
            p.last_failures = result.failures;
            p.last_seconds = result.seconds;
            p.last_pairs = result.totals.pairs;
            p.last_gaps = result.totals.gaps;
            p.last_lost = result.totals.lost();
            p.last_bytes = result.totals.record_bytes;
            p.last_esp = result.esp;
            ++(failed ? p.runs_failed : result.totals.gaps ? p.runs_gaps : p.runs_ok);
            if (failed) {
                p.state = "error";
                p.message = "the last run failed:\n" + result.failures;
            }
        });
    }

    // ---- web thread --------------------------------------------------------------------------------

    void on_open(unsigned client)
    {
        if (client_) web_.close(client_, kReplaced, "another browser took over");
        client_ = client;
        viewer_ = Viewer();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            connected_ = true;
            request_changed_ = true;
        }
        changed_.notify_all();
        std::string windows;
        for (const auto &name : spectrum_windows())
            windows += std::string(windows.empty() ? "" : ",") + "{\"name\":" + json_quote(name) +
                       ",\"bandwidth\":" + json_number(spectrum_window_bandwidth(name)) + "}";
        web_.send_text(client, "{\"t\":\"hello\",\"version\":" + std::to_string(kProtocolVersion) +
                                   ",\"client\":" + std::to_string(client) + ",\"windows\":[" + windows +
                                   "],\"max_bins\":" + std::to_string(kMaxBins) + "}");
        send_status(client, viewer_, status_sections());
    }

    void on_close(unsigned client)
    {
        if (client != client_) return;  // one that was replaced
        client_ = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            connected_ = false;
            last_client_ = Clock::now();
            request_changed_ = true;
        }
        changed_.notify_all();
    }

    void on_text(unsigned client, const std::string &message)
    {
        if (client != client_) return;
        Viewer &v = viewer_;
        try {
            JsonObject m = parse_json_object(message);
            auto number = [&](const char *key, double fallback) {
                auto f = m.find(key);
                return f != m.end() && f->second.kind == JsonValue::Number ? f->second.number : fallback;
            };
            auto text = [&](const char *key) {
                auto f = m.find(key);
                return f != m.end() && f->second.kind == JsonValue::String ? f->second.string : std::string();
            };
            auto flag = [&](const char *key, bool fallback) {
                auto f = m.find(key);
                return f != m.end() && f->second.kind == JsonValue::Boolean ? f->second.boolean : fallback;
            };
            std::string type = text("t");
            if (type == "ack") {
                acknowledge(client, v, uint32_t(number("seq", 0)));
            } else if (type == "ping") {
                web_.send_text(client, "{\"t\":\"pong\",\"c\":" + json_number(number("c", 0)) +
                                           ",\"s\":" + json_number(wall_ms()) + "}");
            } else if (type == "view") {
                v.start_hz = number("start", 0);
                v.stop_hz = number("stop", 0);
                v.bins = unsigned(std::clamp(number("bins", 1024), 16.0, double(kMaxBins)));
                v.auto_bins = flag("auto", true);
                if (!v.auto_bins) v.bins_cap = 0;
                v.max_fps = std::clamp(number("fps", 0), 0.0, 100.0);
                v.max_kbps = std::clamp(number("kbps", 0), 0.0, 1e6);
                try_send(client, v);
            } else if (type == "set") {
                std::string setting = text("v");
                parse_receiver_setting(setting);  // refuse bad input at once
                std::string name = setting.substr(0, setting.find('='));
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (!published_.receiver_valid) throw std::runtime_error("the receiver is not available");
                    auto same = std::find_if(settings_.begin(), settings_.end(),
                                             [&](const auto &s) { return s.first == name; });
                    if (same != settings_.end()) same->second = setting;
                    else settings_.push_back({name, setting});
                }
                changed_.notify_all();
            } else if (type == "run") {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    running_ = flag("v", running_);
                    request_changed_ = true;
                }
                changed_.notify_all();
                broadcast_status();
            } else if (type == "fft") {
                SpectrumSettings s = engine_.settings();
                s.size = unsigned(number("size", s.size));
                if (std::string w = text("window"); !w.empty()) s.window = w;
                s.rate = number("rate", s.rate);
                s.averaging = unsigned(std::max(0.0, number("averaging", s.averaging)));
                s.peak = flag("peak", s.peak);
                s.remove_dc = flag("dc", s.remove_dc);
                s.validate();
                engine_.configure(s);
                broadcast_status();
            } else {
                throw std::runtime_error("unknown message type " + type);
            }
        } catch (const std::exception &e) {
            web_.send_text(client, "{\"t\":\"notice\",\"text\":" + json_quote(e.what()) + "}");
        }
    }

    void on_wake()
    {
        std::deque<std::shared_ptr<const SpectrumFrame>> frames;
        {
            std::lock_guard<std::mutex> lock(frames_mutex_);
            frames.swap(frames_);
        }
        if (client_)
            for (const auto &frame : frames) offer(client_, viewer_, frame);

        std::vector<std::string> notices;
        bool changed;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            notices.swap(notices_);
            changed = published_changed_;
            published_changed_ = false;
        }
        if (!client_) return;
        for (const auto &text : notices)
            web_.send_text(client_, "{\"t\":\"notice\",\"text\":" + json_quote(text) + "}");
        if (changed) broadcast_status();
    }

    void on_tick()
    {
        if (!client_) return;
        auto now = Clock::now();
        Viewer &v = viewer_;
        // A client that stops acknowledging (a bug, or a frozen page) is not
        // waited for forever; a merely slow link acknowledges well within this.
        while (!v.in_flight.empty() && now - v.in_flight.front().time > 30s) {
            v.in_flight_bytes -= v.in_flight.front().bytes;
            v.in_flight.pop_front();
        }
        try_send(client_, v);
        double interval = std::chrono::duration<double>(now - v.mark).count();
        if (interval >= 1) {
            uint64_t bytes = web_.sent(client_);
            v.fps = v.frames_sent / interval;
            v.delivered_rate = (v.delivered - v.delivered_mark) / interval;
            v.delivered_mark = v.delivered;
            v.merged_rate = v.frames_merged / interval;
            v.kbps = (bytes - v.bytes_mark) * 8 / 1000.0 / interval;
            v.frames_sent = v.frames_merged = 0;
            v.bytes_mark = bytes;
            v.mark = now;
            adapt_resolution(v);
        }
        // Status takes at most about a fifth of what the link is seen to
        // deliver (every second when no frames flow), and waits while the
        // link is backed up.
        auto status_interval = std::chrono::duration<double>(
            v.delivered_rate > 0 ? std::clamp(4.0 * v.last_status_bytes / v.delivered_rate, 0.25, 4.0) : 1.0);
        if (now - v.last_status >= status_interval && web_.queued(client_) < kSocketBacklog)
            send_status(client_, v, status_sections());
    }

    // In automatic resolution, halves or doubles the bins per frame so that
    // the link carries about 15 frames a second (fewer when fewer are made),
    // judged only by the frames it really delivered at the present size:
    // halved when frames are being merged and fewer than 70% of that get
    // through, doubled when more than 1.6 times it do (so half as many would
    // still be enough). The factor-of-two steps keep the picture steady.
    void adapt_resolution(Viewer &v)
    {
        if (!v.auto_bins) {
            v.bins_cap = 0;
            return;
        }
        double target = std::min(engine_.settings().rate, 15.0);
        if (v.max_fps > 0) target = std::min(target, v.max_fps);
        unsigned cap = v.bins_cap ? v.bins_cap : v.bins;
        if (v.merged_rate > 0 && v.fps < 0.7 * target && cap > 256) cap /= 2;
        else if (v.fps > 1.6 * target && cap < v.bins) cap = std::min(v.bins, cap * 2);
        v.bins_cap = cap >= v.bins ? 0 : cap;
    }

    // Merges a new frame into what the viewer has waiting, then tries to send.
    void offer(unsigned client, Viewer &v, const std::shared_ptr<const SpectrumFrame> &frame)
    {
        const SpectrumFrame *p = v.pending.get();
        bool same = p && p->run == frame->run && p->center_hz == frame->center_hz &&
                    p->sample_rate == frame->sample_rate && p->db.size() == frame->db.size() &&
                    p->peak == frame->peak;
        if (!same) {
            v.pending = frame;
            v.merged.reset();
            v.pending_frames = 1;
        } else {
            if (!v.merged) {
                v.merged = std::make_shared<SpectrumFrame>(*v.pending);
                v.pending = v.merged;
            }
            SpectrumFrame *merged = v.merged.get();
            float n = float(v.pending_frames);
            for (size_t i = 0; i < merged->db.size(); ++i)
                merged->db[i] = frame->peak ? std::max(merged->db[i], frame->db[i])
                                            : (merged->db[i] * n + frame->db[i]) / (n + 1);
            merged->blocks += frame->blocks;
            merged->index = frame->index;
            ++v.pending_frames;
            ++v.frames_merged;
        }
        try_send(client, v);
    }

    // Bytes allowed in flight; a probe for the empty-link round trip allows
    // just one frame.
    uint64_t window_bytes(const Viewer &v, size_t frame) const
    {
        return v.probing ? 1 : std::max<uint64_t>(frame, uint64_t(v.window));
    }

    // Charges bytes just queued for the client against its bandwidth cap.
    void charge(unsigned client, Viewer &v, uint64_t before)
    {
        v.tokens -= double(web_.sent(client) + web_.queued(client) - before);
    }

    void try_send(unsigned client, Viewer &v)
    {
        if (!v.pending || web_.queued(client) > kSocketBacklog || v.in_flight.size() >= 32) return;
        auto now = Clock::now();
        if (v.max_fps > 0 && now - v.last_send < std::chrono::duration<double>(1 / v.max_fps)) return;
        if (v.max_kbps > 0) {
            double budget = v.max_kbps * 1000 / 8;
            v.tokens = std::min(budget / 4, v.tokens + budget * std::chrono::duration<double>(now - v.tokens_at).count());
            v.tokens_at = now;
            if (v.tokens < 0) return;
        }
        // The empty-link round trip is measured again every 10 s, with one
        // frame alone in flight: queueing would otherwise inflate it for good.
        if (!v.probing && now - v.rtt_min_at > 10s) v.probing = true;
        // The next frame is judged by the last one's size on the wire
        // (compressed, if the link compresses).
        uint64_t estimate = v.last_frame_bytes ? v.last_frame_bytes : 64;
        if (!v.in_flight.empty() && v.in_flight_bytes + estimate > window_bytes(v, estimate)) return;

        std::string message = encode(*v.pending, v, v.sequence + 1);
        uint64_t before = web_.sent(client) + web_.queued(client);
        web_.send_binary(client, message);
        uint64_t bytes = web_.sent(client) + web_.queued(client) - before;
        v.last_frame_bytes = bytes;
        charge(client, v, before);
        if (!v.window) v.window = double(bytes);
        v.in_flight.push_back({++v.sequence, now, bytes, v.in_flight.empty()});
        v.in_flight_bytes += bytes;
        v.last_send = now;
        v.pending.reset();
        v.merged.reset();
        ++v.frames_sent;
    }

    void acknowledge(unsigned client, Viewer &v, uint32_t sequence)
    {
        auto now = Clock::now();
        while (!v.in_flight.empty() && int32_t(sequence - v.in_flight.front().sequence) >= 0) {
            Viewer::Sent sent = v.in_flight.front();
            v.in_flight.pop_front();
            v.in_flight_bytes -= sent.bytes;
            v.delivered += sent.bytes;
            if (sent.sequence != sequence) continue;
            double rtt = std::chrono::duration<double>(now - sent.time).count();
            v.rtt = v.rtt ? 0.8 * v.rtt + 0.2 * rtt : rtt;
            if (!v.rtt_min || rtt < v.rtt_min || (v.probing && sent.alone)) {
                v.rtt_min = rtt;
                v.rtt_min_at = now;
                v.probing = false;
            }
            // Queueing delay: how much longer this round trip took than one
            // over the empty link. Well below the target the window grows by
            // what was delivered (doubling each round trip), near it by about
            // a frame per round trip; above it, it shrinks by 30%, at most
            // once per round trip.
            double queueing = rtt - v.rtt_min, target = std::max(0.03, 0.25 * v.rtt_min);
            if (queueing > target) {
                if (now - v.cut_at > std::chrono::duration<double>(rtt)) {
                    v.window = std::max(double(sent.bytes), 0.7 * v.window);
                    v.cut_at = now;
                }
            } else if (queueing < target / 2) {
                v.window += double(sent.bytes);
            } else {
                v.window += double(sent.bytes) * double(sent.bytes) / v.window;
            }
            v.window = std::min(v.window, 4e6);
        }
        try_send(client, v);
    }

    // The frame fitted to the viewer's range and resolution: at most
    // v.bins bins, each the maximum of the FFT bins it covers, as 8-bit
    // steps of kStepDb above a base level.
    static std::string encode(const SpectrumFrame &f, const Viewer &v, uint32_t sequence)
    {
        // Bin i is centred on center + (i - n/2) * width, so it spans half a bin either side.
        const size_t n = f.db.size();
        const double width = f.sample_rate / double(n), left = f.center_hz - f.sample_rate / 2 - width / 2;
        size_t first = 0, last = n;
        if (v.stop_hz > v.start_hz) {
            first = size_t(std::clamp(std::floor((v.start_hz - left) / width), 0.0, double(n - 1)));
            last = size_t(std::clamp(std::ceil((v.stop_hz - left) / width), double(first + 1), double(n)));
        }
        const size_t span = last - first, bins = std::min<size_t>(span, v.bins_cap ? std::min(v.bins_cap, v.bins) : v.bins);
        std::vector<float> values(bins);
        float low = 1e9f, high = -1e9f;
        for (size_t o = 0; o < bins; ++o) {
            size_t from = first + o * span / bins, to = std::max(from + 1, first + (o + 1) * span / bins);
            float peak = f.db[from];
            for (size_t i = from + 1; i < to; ++i) peak = std::max(peak, f.db[i]);
            values[o] = peak;
            low = std::min(low, peak);
            high = std::max(high, peak);
        }
        float base = std::max(std::floor(low / kStepDb) * kStepDb, std::ceil(high / kStepDb) * kStepDb - 255 * kStepDb);

        std::string out(48 + bins, '\0');
        auto put = [&](size_t at, const void *data, size_t size) { std::memcpy(&out[at], data, size); };
        uint8_t kind = 1, flags = f.peak ? 1 : 0;
        uint16_t count = uint16_t(bins);
        double start = left + double(first) * width, stop = left + double(last) * width, time = wall_ms();
        float step = kStepDb;
        uint32_t frames = v.pending_frames, blocks = f.blocks;
        put(0, &kind, 1);
        put(1, &flags, 1);
        put(2, &count, 2);
        put(4, &sequence, 4);
        put(8, &start, 8);
        put(16, &stop, 8);
        put(24, &base, 4);
        put(28, &step, 4);
        put(32, &frames, 4);
        put(36, &blocks, 4);
        put(40, &time, 8);
        for (size_t o = 0; o < bins; ++o)
            out[48 + o] = char(uint8_t(std::clamp(std::lround((values[o] - base) / kStepDb), 0L, 255L)));
        return out;
    }

    void broadcast_status()
    {
        if (client_ && web_.queued(client_) < kSocketBacklog) send_status(client_, viewer_, status_sections());
    }

    // Sends the status, leaving out the sections that rarely change when
    // they are as last sent (all of them go again every 10 s).
    void send_status(unsigned client, Viewer &v, const std::vector<std::pair<std::string, std::string>> &sections)
    {
        static const std::set<std::string> steady = {"devices", "receiver", "fft", "last"};
        auto now = Clock::now();
        bool everything = now - v.sections_at > 10s;
        if (everything) v.sections_at = now;
        std::string text = "{\"t\":\"status\"";
        for (const auto &[name, json] : sections) {
            if (steady.count(name) && !everything && v.sections[name] == json) continue;
            v.sections[name] = json;
            text += ",\"" + name + "\":" + json;
        }
        v.last_status = now;
        char link[480];
        std::snprintf(link, sizeof(link),
                      ",\"link\":{\"rtt\":%.4f,\"rtt_min\":%.4f,\"delivered\":%.0f,\"window\":%llu,"
                      "\"in_flight\":%zu,\"in_flight_bytes\":%llu,\"fps\":%.1f,\"merged\":%.1f,\"kbps\":%.1f,"
                      "\"queued\":%zu,\"compressed\":%s,\"bins_cap\":%u,\"peer\":%s}}",
                      v.rtt, v.rtt_min, v.delivered_rate * 8 / 1000, (unsigned long long)window_bytes(v, 0),
                      v.in_flight.size(), (unsigned long long)v.in_flight_bytes, v.fps, v.merged_rate, v.kbps,
                      web_.queued(client), web_.compressing(client) ? "true" : "false", v.bins_cap,
                      json_quote(web_.peer(client)).c_str());
        text += link;
        uint64_t before = web_.sent(client) + web_.queued(client);
        web_.send_text(client, text);
        v.last_status_bytes = web_.sent(client) + web_.queued(client) - before;
        charge(client, v, before);
    }

    // The status as named JSON sections.
    std::vector<std::pair<std::string, std::string>> status_sections()
    {
        Published p;
        bool running;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            p = published_;
            running = running_;
        }
        SpectrumSettings fft = engine_.settings();
        SpectrumEngine::Stats spectrum = engine_.stats();
        auto now = Clock::now();
        double interval = std::chrono::duration<double>(now - spectrum_mark_).count();
        if (interval >= 1) {
            spectrum_fps_ = (spectrum.frames - spectrum_mark_stats_.frames) / interval;
            spectrum_used_ = (spectrum.pairs_used - spectrum_mark_stats_.pairs_used) / interval;
            spectrum_dropped_ = (spectrum.pairs_dropped - spectrum_mark_stats_.pairs_dropped) / interval;
            spectrum_mark_ = now;
            spectrum_mark_stats_ = spectrum;
        }

        std::vector<std::pair<std::string, std::string>> sections = {
            {"state", json_quote(p.state)},
            {"message", json_quote(p.message)},
            {"running", running ? "true" : "false"},
            {"time", json_number(wall_ms())},
            {"devices", "{\"esp\":" + json_quote(p.esp_port) + ",\"fpga\":" + json_quote(p.fpga_port) +
                            ",\"ft600\":" + json_quote(p.ft600) + "}"},
        };
        std::string s;
        auto section = [&](const char *name) {
            sections.push_back({name, s});
            s.clear();
        };
        // A section that is not known goes as null, so the page drops what it had.
        if (p.receiver_valid) {
            const ReceiverState &r = p.receiver;
            s += "{\"lo\":" + std::to_string(r.lo_hz) + ",\"rate\":" + (r.rate == ESP_RATE_16M ? "16" : "80") +
                 ",\"width\":" + std::to_string(r.width) + ",\"filter\":[" + std::to_string(r.filter_first()) + "," +
                 std::to_string(r.filter_second()) + "],\"gain\":" + std::to_string(r.gain) +
                 ",\"rf\":" + std::to_string(r.rf_gain) + ",\"bb\":" + std::to_string(r.bb_gain) + ",\"dc\":[" +
                 std::to_string(r.dc[0]) + "," + std::to_string(r.dc[1]) + "," + std::to_string(r.dc[2]) + "," +
                 std::to_string(r.dc[3]) + "],\"iq\":[" + std::to_string(r.iq_amplitude()) + "," +
                 std::to_string(r.iq_phase()) + "],\"auto\":" + std::to_string(r.automatic) + ",\"pll\":[" +
                 std::to_string(r.pll_cap()) + "," + std::to_string(r.pll_first()) + "," +
                 std::to_string(r.pll_length()) + "],\"sample_rate\":" + json_number(r.sample_rate()) + "}";
        } else {
            s = "null";
        }
        section("receiver");
        s += "{\"size\":" + std::to_string(fft.size) + ",\"window\":" + json_quote(fft.window) +
             ",\"rate\":" + json_number(fft.rate) + ",\"averaging\":" + std::to_string(fft.averaging) +
             ",\"peak\":" + (fft.peak ? "true" : "false") + ",\"dc\":" + (fft.remove_dc ? "true" : "false") + "}";
        section("fft");
        s += "{\"fps\":" + json_number(std::round(spectrum_fps_ * 10) / 10) +
             ",\"pairs_used\":" + json_number(std::round(spectrum_used_)) +
             ",\"pairs_dropped\":" + json_number(std::round(spectrum_dropped_)) +
             ",\"coverage\":" + json_number(p.pair_rate > 0 ? std::min(1.0, spectrum_used_ / p.pair_rate) : 0) + "}";
        section("spectrum");
        s += "{\"id\":" + std::to_string(p.run) + ",\"seconds\":" + json_number(p.run_seconds) +
             ",\"usb_bytes\":" + std::to_string(p.usb_bytes) + ",\"pairs\":" + std::to_string(p.pairs) +
             ",\"usb_rate\":" + json_number(std::round(p.usb_rate)) + ",\"pair_rate\":" + json_number(std::round(p.pair_rate)) +
             ",\"ok\":" + std::to_string(p.runs_ok) + ",\"gaps\":" + std::to_string(p.runs_gaps) +
             ",\"failed\":" + std::to_string(p.runs_failed) + "}";
        section("run");
        if (p.fpga_valid) {
            s += "{\"capacity\":" + std::to_string(FPGA_RING_BEATS) + ",\"beat_bytes\":16";
            for (unsigned i = 0; i < FPGA_STAT_COUNT; ++i)
                s += ",\"" + std::string(kFpgaNames[i]) + "\":" + std::to_string(p.fpga[i]);
            s += "}";
        } else {
            s = "null";
        }
        section("fpga");
        if (p.last_valid) {
            const auto &e = p.last_esp;
            s += "{\"result\":" + json_quote(p.last_result) + ",\"failures\":" + json_quote(p.last_failures) +
                 ",\"seconds\":" + json_number(p.last_seconds) + ",\"pairs\":" + std::to_string(p.last_pairs) +
                 ",\"gaps\":" + std::to_string(p.last_gaps) + ",\"lost\":" + std::to_string(p.last_lost) +
                 ",\"bytes\":" + std::to_string(p.last_bytes) + ",\"esp\":{\"status\":" + std::to_string(e[ESP_STAT_STATUS]) +
                 ",\"failure\":" + json_quote(esp_failure(e[ESP_STAT_STATUS])) +
                 ",\"detail\":" + std::to_string(e[ESP_STAT_DETAIL]) + ",\"lane\":" + std::to_string(e[ESP_STAT_FAIL_LANE]) +
                 ",\"units0\":" + std::to_string(e[ESP_STAT_UNITS0]) + ",\"units1\":" + std::to_string(e[ESP_STAT_UNITS1]) +
                 ",\"service0_us\":" + json_number(e[ESP_STAT_SERVICE_MAX0] / 240.0) +
                 ",\"service1_us\":" + json_number(e[ESP_STAT_SERVICE_MAX1] / 240.0) +
                 ",\"stopped_by_host\":" + std::to_string(e[ESP_STAT_STOPPED_BY_HOST]) + "}}";
        } else {
            s = "null";
        }
        section("last");
        return sections;
    }

    const Options &options_;
    WebServer web_;

    // Shared between the capture and web threads.
    std::mutex mutex_;
    std::condition_variable changed_;
    Published published_;
    bool published_changed_ = false;
    std::vector<std::string> notices_;                               // for the client
    std::vector<std::pair<std::string, std::string>> settings_;      // name, NAME=VALUE
    bool running_ = true, request_changed_ = false, shutdown_ = false;
    bool connected_ = false;
    Clock::time_point last_client_ = Clock::now() - kIdleGrace;

    std::mutex frames_mutex_;
    std::deque<std::shared_ptr<const SpectrumFrame>> frames_;
    SpectrumEngine engine_;  // after the queue its thread fills: destroyed (and joined) before it

    // Web thread only.
    unsigned client_ = 0;  // the WebSocket being served; 0: none
    Viewer viewer_;
    Clock::time_point spectrum_mark_ = Clock::now();
    SpectrumEngine::Stats spectrum_mark_stats_;
    double spectrum_fps_ = 0, spectrum_used_ = 0, spectrum_dropped_ = 0;

    std::thread capture_;
};

}  // namespace

int command_serve(const Options &options)
{
    Service service(options);
    std::fprintf(stderr, "serving on http://%s/  (Ctrl-C to stop)\n", service.address().c_str());
    stop_requested = false;
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    service.run(stop_requested);
    std::fprintf(stderr, "stopping\n");
    return 0;
}
