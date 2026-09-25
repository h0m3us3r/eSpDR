// One verified capture run, shared by capture, calibrate and serve.
#pragma once

#include <array>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <string>

#include "control.h"
#include "decoder.h"
#include "ft600.h"
#include "options.h"
#include "serial.h"

// The three devices, identified and checked for readiness. `require_radio`:
// refuse an ESP whose receiver is not configured (serve instead lets the
// user change the setting that failed).
struct Devices {
    ControlPort esp;
    ControlPort fpga;
    Ft600 usb;

    explicit Devices(const Options &options, bool require_radio = true);
};

struct RunProgress {
    double seconds;      // since the run started
    uint64_t usb_bytes;  // received
    uint64_t pairs;      // decoded and verified
};

struct RunRequest {
    unsigned seconds = 0;  // 0: until stopped
    OutputFormat format = OutputFormat::None;
    std::FILE *output = nullptr;
    unsigned threads = 4;
    bool show_progress = false;  // a progress line on stderr
    bool catch_signals = true;   // Ctrl-C and SIGTERM end the run
    // Polled while streaming, several times a second, on the calling thread
    // (which owns the devices): `stop` ends the run when it returns true.
    std::function<bool()> stop;
    std::function<void(const RunProgress &)> progress;
    SampleSink samples;  // decoded pairs, as they are verified
};

struct RunResult {
    StreamTotals totals;
    std::array<uint32_t, ESP_STAT_COUNT> esp{};
    std::array<uint32_t, FPGA_STAT_COUNT> fpga{};
    std::string failures;  // one line each; empty when every pair was verified
    double seconds = 0;

    void require(bool condition, const std::string &what)
    {
        if (!condition) failures += "  " + what + "\n";
    }
    uint32_t link_errors() const;
};

// Arms, streams, stops and drains, then reconciles the ESP's, the FPGA's and
// the host's counts. Leaves both devices stopped whatever happens.
RunResult run_capture(Devices &devices, const RunRequest &request);

std::string describe_flags(uint32_t flags);  // FPGA_FLAG_* as " name" / " !name"
const char *esp_failure(uint32_t status);    // ESP_FAIL_*
