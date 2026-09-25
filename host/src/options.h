// Command-line options shared by the commands, and device discovery.
#pragma once

#include <string>
#include <vector>

struct Options {
    // Devices. Empty means: find the only matching device.
    std::string esp_port;     // ESP32-S3 USB Serial/JTAG
    std::string fpga_port;    // Alchitry Au FT2232 channel B UART
    std::string com_port;     // USB-UART bridge whose RTS/DTR drive the ESP's EN/BOOT
    std::string ftdi_serial;  // Alchitry FT2232 serial, for openFPGALoader
    std::string ft600_serial; // FT600 on the Ft board

    // capture
    unsigned seconds = 10;    // 0: until Ctrl-C
    std::string output;       // file, "-" for stdout, empty: check only
    std::string format;       // "iqc" or "cs16"; default from the output name
    unsigned threads = 4;
    int phase = -1;           // link sampling phase to set first; -1: leave as is
    unsigned phase_step = 4;  // calibrate: phase sweep step

    // load
    std::string bitstream;
    std::string esp_image;

    // decode
    std::string input;

    // set: receiver settings, NAME=VALUE
    std::vector<std::string> settings;

    // serve
    std::string listen = "127.0.0.1:8073";  // HTTP address:port
    std::string web_dir;                    // serve the UI from here instead of the built-in copy
};

// Resolves any empty device field that the command needs, from
// /dev/serial/by-id. Throws if a device is missing or ambiguous.
std::string find_esp_port(const Options &options);
std::string find_fpga_port(const Options &options);
std::string find_com_port(const Options &options);
std::string find_ftdi_serial(const Options &options);

// Waits up to `seconds` for a path to exist.
bool wait_for_path(const std::string &path, double seconds);

int command_capture(const Options &options);
int command_calibrate(const Options &options);
int command_status(const Options &options);
int command_load(const Options &options);
int command_decode(const Options &options);
int command_set(const Options &options);
int command_serve(const Options &options);
