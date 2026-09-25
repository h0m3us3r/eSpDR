// iqstream: host tool for the ESP32-S3 -> FPGA -> USB IQ streamer.
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>

#include "options.h"

namespace {

const char *kUsage = R"(usage: iqstream COMMAND [options]

commands:
  load --fpga BITSTREAM --esp IMAGE [--phase P]
                                      load both devices (RAM only) and start the reference
  load --esp IMAGE                    reload only the ESP32-S3
  capture [--seconds N] [--output FILE|-] [--format iqc|cs16] [--phase P]
                                      stream and verify; N = 0 runs until Ctrl-C
  calibrate [--step S]                sweep the FPGA link sampling phase, set its centre
  decode INPUT.iqc OUTPUT.cs16        convert a captured .iqc file ("-" for stdin/stdout)
  status                              identities, readiness, receiver settings, last-run statistics
  set NAME=VALUE...                   change receiver settings (kept until the ESP is reloaded):
                                        lo=2402M rate=80|16 width=40|20 filter=N[,M] gain=N
                                        rf=N|auto bb=N|auto dc0..dc3=N|auto iq=A,P|auto
  serve [--listen ADDR:PORT]          stream a live spectrum and waterfall to web browsers, with
                                      receiver control and live statistics (default 127.0.0.1:8073)

device options (default: the only matching device):
  --esp-port PORT   ESP32-S3 USB serial port
  --fpga-port PORT  Alchitry Au UART (FT2232 channel B)
  --com-port PORT   ESP32-S3 board USB-UART port, whose RTS/DTR drive EN/BOOT
  --ftdi-serial SN  Alchitry FT2232 serial number (for openFPGALoader)
  --ft600 SN        FT600 serial number

other options:
  --threads N       decoding threads (default 4)
  --web DIR         serve: the web UI from DIR instead of the built-in copy

environment: OPENFPGALOADER, ESPTOOL override the tool commands.
)";

unsigned parse_number(const char *text, const char *option)
{
    char *end = nullptr;
    unsigned long value = std::strtoul(text, &end, 10);
    if (!*text || *end) throw std::runtime_error(std::string(option) + " needs a number");
    return unsigned(value);
}

}  // namespace

int main(int argc, char **argv)
{
    if (argc < 2 || !std::strcmp(argv[1], "-h") || !std::strcmp(argv[1], "--help")) {
        std::fputs(kUsage, argc < 2 ? stderr : stdout);
        return argc < 2 ? 2 : 0;
    }
    try {
        std::string command = argv[1];
        Options options;
        int positional = 0;
        for (int i = 2; i < argc; ++i) {
            std::string arg = argv[i];
            auto value = [&]() -> const char * {
                if (i + 1 >= argc) throw std::runtime_error(arg + " needs a value");
                return argv[++i];
            };
            if (arg == "--esp" && command == "load") options.esp_image = value();
            else if (arg == "--fpga" && command == "load") options.bitstream = value();
            else if (arg == "--esp-port") options.esp_port = value();
            else if (arg == "--fpga-port") options.fpga_port = value();
            else if (arg == "--com-port") options.com_port = value();
            else if (arg == "--ftdi-serial") options.ftdi_serial = value();
            else if (arg == "--ft600") options.ft600_serial = value();
            else if (arg == "--seconds") options.seconds = parse_number(value(), "--seconds");
            else if (arg == "--output" || arg == "-o") options.output = value();
            else if (arg == "--format") options.format = value();
            else if (arg == "--threads") options.threads = parse_number(value(), "--threads");
            else if (arg == "--phase") options.phase = int(parse_number(value(), "--phase"));
            else if (arg == "--step") options.phase_step = parse_number(value(), "--step");
            else if (arg == "--listen" && command == "serve") options.listen = value();
            else if (arg == "--web" && command == "serve") options.web_dir = value();
            else if (command == "set" && arg.find('=') != std::string::npos && arg[0] != '-')
                options.settings.push_back(arg);
            else if (command == "decode" && positional == 0 && (arg == "-" || arg[0] != '-')) {
                options.input = arg;
                ++positional;
            } else if (command == "decode" && positional == 1 && (arg == "-" || arg[0] != '-')) {
                options.output = arg;
                ++positional;
            } else {
                throw std::runtime_error("unknown option " + arg);
            }
        }
        if (options.seconds > 65535) throw std::runtime_error("--seconds must be at most 65535");
        if (options.threads == 0 || options.threads > 64) throw std::runtime_error("--threads must be 1..64");
        if (options.phase >= 280) throw std::runtime_error("--phase must be 0..279");
        if (options.phase_step == 0 || options.phase_step > 70) throw std::runtime_error("--step must be 1..70");

        if (command == "capture") return command_capture(options);
        if (command == "calibrate") return command_calibrate(options);
        if (command == "status") return command_status(options);
        if (command == "load") return command_load(options);
        if (command == "decode") return command_decode(options);
        if (command == "set") return command_set(options);
        if (command == "serve") return command_serve(options);
        std::fputs(kUsage, stderr);
        return 2;
    } catch (const std::exception &e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
