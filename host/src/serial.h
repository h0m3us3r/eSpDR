// Control protocol client (protocol/control.h) over a serial port.
#pragma once

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

class ControlError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class ControlPort {
public:
    // node: CTL_NODE_ESP or CTL_NODE_FPGA. Opens the port exclusively.
    ControlPort(const std::string &path, unsigned node);
    ~ControlPort();
    ControlPort(const ControlPort &) = delete;
    ControlPort &operator=(const ControlPort &) = delete;

    struct Reply {
        uint8_t status = 0;
        uint32_t value = 0;
    };

    // Sends a request and waits for its response; throws unless status is
    // CTL_OK (or allow_failure is set).
    uint32_t command(unsigned op, unsigned arg = 0,
                     std::chrono::milliseconds timeout = std::chrono::milliseconds(2000));
    Reply request(unsigned op, unsigned arg, std::chrono::milliseconds timeout);

    // Split form, for commands whose reply comes much later (ESP_RUN).
    uint16_t send(unsigned op, unsigned arg);
    // Returns false on timeout; throws on a malformed reply.
    bool receive(unsigned op, uint16_t sequence, Reply &reply, std::chrono::milliseconds timeout);

    const std::string &path() const { return path_; }

private:
    using Clock = std::chrono::steady_clock;
    bool read_frame(uint8_t *frame, Clock::time_point deadline);

    int fd_ = -1;
    unsigned node_;
    std::vector<uint8_t> buffer_;  // received bytes not yet consumed
    uint16_t sequence_ = 0;
    std::string path_;
};

std::string control_status_name(unsigned status);
