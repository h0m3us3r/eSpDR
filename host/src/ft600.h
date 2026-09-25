// FT600 bulk-IN reception with several transfers kept in flight.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

class UsbError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

class Ft600 {
public:
    static constexpr size_t kTransferBytes = 1 << 20;  // matches the FPGA's transfer size
    static constexpr size_t kInFlight = 8;

    // serial: FT600 serial number, or empty for the only FT600 present.
    // Requires SuperSpeed and the 245 / 1 IN channel / 100 MHz configuration.
    explicit Ft600(const std::string &serial);
    ~Ft600();
    Ft600(const Ft600 &) = delete;
    Ft600 &operator=(const Ft600 &) = delete;

    // Discards data left in the FT600 by an interrupted run.
    size_t drain();
    // Submits all transfers.
    void start();
    // Waits for the oldest transfer, hands its bytes to `data` and resubmits
    // it. Returns false if it timed out or completed empty.
    bool next(std::vector<uint8_t> &data);
    // Wakes a thread blocked in next(); may be called from another thread.
    void abort();
    // Cancels every outstanding transfer. Not concurrent with next().
    void stop();

    const std::string &serial() const { return serial_; }

private:
    struct Slot;
    void submit(Slot &slot);

    void *handle_ = nullptr;
    std::string serial_;
    std::array<std::unique_ptr<Slot>, kInFlight> slots_;
    size_t next_ = 0;
    bool streaming_ = false;
};
