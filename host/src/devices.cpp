#include <chrono>
#include <filesystem>
#include <fnmatch.h>
#include <stdexcept>
#include <thread>
#include <vector>

#include "options.h"

namespace fs = std::filesystem;

namespace {

const char *kById = "/dev/serial/by-id";

std::string find_unique(const std::string &pattern, const std::string &what, const std::string &flag)
{
    std::vector<std::string> matches;
    std::error_code error;
    for (const auto &entry : fs::directory_iterator(kById, error))
        if (fnmatch(pattern.c_str(), entry.path().filename().c_str(), 0) == 0)
            matches.push_back(entry.path().string());
    if (matches.size() == 1) return matches.front();
    throw std::runtime_error(matches.empty() ? "no " + what + " found in " + kById
                                             : "several " + what + "s found; choose one with " + flag);
}

}  // namespace

std::string find_esp_port(const Options &options)
{
    if (!options.esp_port.empty()) return options.esp_port;
    return find_unique("usb-Espressif_USB_JTAG_serial_debug_unit_*-if00", "ESP32-S3 USB serial port", "--esp-port");
}

std::string find_fpga_port(const Options &options)
{
    if (!options.fpga_port.empty()) return options.fpga_port;
    return find_unique("usb-Alchitry_Alchitry_Au_*-if01-port0", "Alchitry Au UART", "--fpga-port");
}

std::string find_com_port(const Options &options)
{
    if (!options.com_port.empty()) return options.com_port;
    return find_unique("usb-1a86_*-if00", "ESP32-S3 COM (USB-UART) port", "--com-port");
}

std::string find_ftdi_serial(const Options &options)
{
    if (!options.ftdi_serial.empty()) return options.ftdi_serial;
    // .../usb-Alchitry_Alchitry_Au_<SERIAL>-if01-port0
    std::string name = fs::path(find_fpga_port(options)).filename().string();
    const std::string prefix = "usb-Alchitry_Alchitry_Au_";
    size_t end = name.find("-if");
    if (name.compare(0, prefix.size(), prefix) != 0 || end == std::string::npos)
        throw std::runtime_error("cannot derive the Alchitry serial number; use --ftdi-serial");
    return name.substr(prefix.size(), end - prefix.size());
}

bool wait_for_path(const std::string &path, double seconds)
{
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (!fs::exists(path)) {
        if (std::chrono::steady_clock::now() > deadline) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return true;
}
