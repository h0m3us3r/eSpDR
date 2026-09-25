// load: bring-up of both devices (volatile: FPGA SRAM and ESP RAM; nothing is
// written to flash).
//
// The ESP32-S3 runs from the FPGA's 40 MHz reference, so the ESP is held in
// reset while the FPGA is (re)configured and its reference started, then
// released into its ROM loader, which receives the RAM image.
//
// decode: converts a captured .iqc file into interleaved int16 IQ.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "control.h"
#include "decoder.h"
#include "options.h"
#include "serial.h"

using namespace std::chrono_literals;

namespace {

// The ESP32-S3 board's USB-UART bridge drives EN from RTS and GPIO0 (BOOT)
// from DTR through the usual auto-reset transistors.
class ResetLines {
public:
    explicit ResetLines(const std::string &path)
    {
        fd_ = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) throw std::runtime_error("cannot open " + path);
        termios tty{};
        tcgetattr(fd_, &tty);
        cfmakeraw(&tty);
        tty.c_cflag |= CLOCAL | CREAD;
        tty.c_cflag &= ~(CRTSCTS | HUPCL);  // keep the lines as set when closing
        tcsetattr(fd_, TCSANOW, &tty);
    }
    ~ResetLines() { close(fd_); }

    void hold_in_reset() { set(false, true); }
    void boot_to_rom()
    {
        set(true, false);  // EN released while BOOT is held low
        std::this_thread::sleep_for(100ms);
        set(false, false);
    }

private:
    void set(bool dtr, bool rts)
    {
        int lines = 0;
        ioctl(fd_, TIOCMGET, &lines);
        lines &= ~(TIOCM_DTR | TIOCM_RTS);
        if (dtr) lines |= TIOCM_DTR;
        if (rts) lines |= TIOCM_RTS;
        if (ioctl(fd_, TIOCMSET, &lines) != 0) throw std::runtime_error("cannot set RTS/DTR");
    }
    int fd_;
};

std::string quote(const std::string &text)
{
    std::string quoted = "'";
    for (char c : text) quoted += c == '\'' ? std::string("'\\''") : std::string(1, c);
    return quoted + "'";
}

void run(const std::string &command)
{
    std::fprintf(stderr, "$ %s\n", command.c_str());
    if (std::system(command.c_str()) != 0) throw std::runtime_error("command failed: " + command);
}

bool readable(const std::string &path)
{
    std::FILE *file = std::fopen(path.c_str(), "rb");
    if (!file) return false;
    bool has_data = std::fgetc(file) != EOF;
    std::fclose(file);
    return has_data;
}

std::string tool(const char *variable, const char *fallback)
{
    const char *value = std::getenv(variable);
    return value && *value ? value : fallback;
}

// Retries a command until the device answers (it may still be booting).
uint32_t wait_for_answer(const std::string &path, unsigned node, unsigned op, unsigned arg, double seconds)
{
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    for (;;) {
        try {
            ControlPort port(path, node);
            return port.command(op, arg, 500ms);
        } catch (const std::exception &) {
            if (std::chrono::steady_clock::now() > deadline) throw;
            std::this_thread::sleep_for(200ms);
        }
    }
}

}  // namespace

int command_load(const Options &options)
{
    if (options.esp_image.empty())
        throw std::runtime_error("load needs --esp IMAGE (an FPGA reload also restarts the ESP)");
    for (const std::string &file : {options.esp_image, options.bitstream})
        if (!file.empty() && !readable(file)) throw std::runtime_error("cannot read " + file);
    std::string esptool = tool("ESPTOOL", "esptool.py");
    std::string before = "usb_reset";

    if (!options.bitstream.empty()) {
        std::string fpga_port = find_fpga_port(options);
        ResetLines reset(find_com_port(options));
        reset.hold_in_reset();
        std::this_thread::sleep_for(100ms);

        run(tool("OPENFPGALOADER", "openFPGALoader") + " -b alchitry_au --ftdi-serial " +
            quote(find_ftdi_serial(options)) + " --write-sram " + quote(options.bitstream));
        uint32_t id = wait_for_answer(fpga_port, CTL_NODE_FPGA, CTL_INFO, 0, 10);
        if (id != CTL_FPGA_FIRMWARE_ID) throw std::runtime_error("unexpected FPGA image after loading");

        // A freshly configured FPGA starts with the reference off; if it is
        // on, the previous image is still running and the load did not happen.
        ControlPort fpga(fpga_port, CTL_NODE_FPGA);
        if (fpga.command(CTL_STATUS, FPGA_STAT_FLAGS) & FPGA_FLAG_REFERENCE_ON)
            throw std::runtime_error("the FPGA was not reconfigured; check the openFPGALoader output");
        fpga.command(FPGA_REFERENCE, FPGA_REFERENCE_KEY);
        uint32_t required = FPGA_FLAG_REFERENCE_ON | FPGA_FLAG_PHASE_READY | FPGA_FLAG_DESKEW_READY |
                            FPGA_FLAG_DDR_READY;
        auto deadline = std::chrono::steady_clock::now() + 10s;
        uint32_t flags = 0;
        while (((flags = fpga.command(CTL_STATUS, FPGA_STAT_FLAGS)) & required) != required) {
            if (std::chrono::steady_clock::now() > deadline)
                throw std::runtime_error("FPGA did not become ready (flags " + std::to_string(flags) + ")");
            std::this_thread::sleep_for(100ms);
        }
        if (options.phase >= 0) {
            fpga.command(FPGA_PHASE, unsigned(options.phase));
            while ((fpga.command(CTL_STATUS, FPGA_STAT_FLAGS) & FPGA_FLAG_PHASE_READY) == 0 ||
                   fpga.command(CTL_STATUS, FPGA_STAT_PHASE) != unsigned(options.phase)) {
                if (std::chrono::steady_clock::now() > deadline)
                    throw std::runtime_error("the sampling phase did not settle");
                std::this_thread::sleep_for(10ms);
            }
        }
        std::fprintf(stderr, "FPGA ready, 40 MHz reference running, sampling phase %u\n",
                     fpga.command(CTL_STATUS, FPGA_STAT_PHASE));

        reset.boot_to_rom();
        before = "no_reset";
    }

    // Without the reference clock (after a power cycle) the ESP has no USB
    // port until it is booted above, so look it up only now.
    std::string esp_port;
    auto deadline = std::chrono::steady_clock::now() + 10s;
    for (;;) {
        try {
            esp_port = find_esp_port(options);
            if (wait_for_path(esp_port, 0)) break;
        } catch (const std::exception &) {
            if (std::chrono::steady_clock::now() > deadline) throw;
        }
        std::this_thread::sleep_for(100ms);
    }
    if (!options.bitstream.empty()) std::this_thread::sleep_for(500ms);

    run(esptool + " --chip esp32s3 --port " + quote(esp_port) + " --before " + before +
        " --after no_reset --no-stub load_ram " + quote(options.esp_image));

    // The firmware calibrates the radio before answering.
    std::this_thread::sleep_for(500ms);
    if (!wait_for_path(esp_port, 10)) throw std::runtime_error(esp_port + " did not come back");
    uint32_t id = wait_for_answer(esp_port, CTL_NODE_ESP, CTL_INFO, 0, 10);
    if (id != CTL_ESP_FIRMWARE_ID) throw std::runtime_error("unexpected ESP firmware after loading");
    ControlPort esp(esp_port, CTL_NODE_ESP);
    uint32_t radio = esp.command(CTL_STATUS, ESP_STAT_RADIO);
    if (radio != ESP_RADIO_OK)
        throw std::runtime_error("ESP radio initialisation failed (code " + std::to_string(radio) + ")");
    std::fprintf(stderr, "ESP32-S3 firmware running, radio ready\n");
    return 0;
}

int command_decode(const Options &options)
{
    if (options.input.empty() || options.output.empty())
        throw std::runtime_error("usage: iqstream decode INPUT.iqc OUTPUT.cs16");
    std::FILE *input = options.input == "-" ? stdin : std::fopen(options.input.c_str(), "rb");
    if (!input) throw std::runtime_error("cannot open " + options.input);
    std::FILE *output = options.output == "-" ? stdout : std::fopen(options.output.c_str(), "wb");
    if (!output) throw std::runtime_error("cannot create " + options.output);

    StreamTotals totals;
    {
        StreamDecoder decoder(OutputFormat::Cs16, output, options.threads);
        std::vector<uint8_t> buffer(1 << 20);
        while (size_t n = std::fread(buffer.data(), 1, buffer.size(), input)) decoder.add(buffer.data(), n);
        if (std::ferror(input)) throw std::runtime_error("read error on " + options.input);
        totals = decoder.finish();
    }
    if (input != stdin) std::fclose(input);
    if (output != stdout && std::fclose(output) != 0) throw std::runtime_error("write error");
    std::fprintf(stderr, "%llu pairs decoded%s", (unsigned long long)totals.pairs,
                 totals.ended ? ", END record verified" : " (no END record: capture incomplete)");
    if (totals.gaps)
        std::fprintf(stderr, "; %llu gaps, %llu pairs lost, written as zero pairs",
                     (unsigned long long)totals.gaps, (unsigned long long)totals.lost());
    std::fprintf(stderr, "\n");
    return totals.ended ? (totals.gaps ? 3 : 0) : 1;
}
