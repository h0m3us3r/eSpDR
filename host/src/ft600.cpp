#include "ft600.h"

#include <cstring>

#include <ftd3xx.h>

namespace {

constexpr UCHAR kInPipe = 0x82;
constexpr UCHAR kInFifo = 0;
constexpr DWORD kPipeTimeoutMs = 5000;

void check(FT_STATUS status, const char *what)
{
    if (status != FT_OK) throw UsbError(std::string("FT600 ") + what + " failed (status " +
                                        std::to_string(status) + ")");
}

}  // namespace

struct Ft600::Slot {
    std::vector<uint8_t> buffer = std::vector<uint8_t>(kTransferBytes);
    OVERLAPPED overlapped{};
    ULONG transferred = 0;
    bool initialized = false;
    bool pending = false;
};

Ft600::Ft600(const std::string &serial)
{
    DWORD count = 0;
    check(FT_CreateDeviceInfoList(&count), "enumeration");
    std::vector<FT_DEVICE_LIST_INFO_NODE> devices(count);
    if (count) check(FT_GetDeviceInfoList(devices.data(), &count), "enumeration");
    const FT_DEVICE_LIST_INFO_NODE *found = nullptr;
    unsigned matches = 0;
    for (const auto &device : devices) {
        if (device.Type != FT_DEVICE_600) continue;
        if (!serial.empty() && std::strncmp(device.SerialNumber, serial.c_str(), sizeof(device.SerialNumber)))
            continue;
        found = &device;
        ++matches;
    }
    if (matches != 1)
        throw UsbError(matches ? "several FT600 devices; choose one with --ft600" : "no FT600 found");
    if (!(found->Flags & FT_FLAGS_SUPERSPEED))
        throw UsbError("the FT600 is not connected at USB 3 SuperSpeed");
    serial_ = found->SerialNumber;

    FT_HANDLE handle = nullptr;
    check(FT_Create((PVOID)serial_.c_str(), FT_OPEN_BY_SERIAL_NUMBER, &handle), "open");
    handle_ = handle;
    FT_60XCONFIGURATION config{};
    check(FT_GetChipConfiguration(handle, &config), "configuration read");
    if (config.FIFOMode != CONFIGURATION_FIFO_MODE_245 ||
        config.ChannelConfig != CONFIGURATION_CHANNEL_CONFIG_1_INPIPE ||
        config.FIFOClock != CONFIGURATION_FIFO_CLK_100) {
        FT_Close(handle);
        handle_ = nullptr;
        throw UsbError("FT600 must be configured for 245 mode, one IN channel, 100 MHz");
    }
    check(FT_SetStreamPipe(handle, FALSE, FALSE, kInPipe, kTransferBytes), "stream setup");
    streaming_ = true;
    check(FT_SetPipeTimeout(handle, kInPipe, kPipeTimeoutMs), "timeout setup");
    for (auto &slot : slots_) {
        slot = std::make_unique<Slot>();
        check(FT_InitializeOverlapped(handle, &slot->overlapped), "overlapped setup");
        slot->initialized = true;
    }
}

Ft600::~Ft600()
{
    if (!handle_) return;
    FT_HANDLE handle = handle_;
    stop();
    for (auto &slot : slots_)
        if (slot && slot->initialized) FT_ReleaseOverlapped(handle, &slot->overlapped);
    if (streaming_) FT_ClearStreamPipe(handle, FALSE, FALSE, kInPipe);
    FT_Close(handle);
}

size_t Ft600::drain()
{
    std::vector<uint8_t> buffer(kTransferBytes);
    size_t total = 0;
    for (;;) {
        ULONG got = 0;
        FT_STATUS status = FT_ReadPipeEx(handle_, kInFifo, buffer.data(), ULONG(buffer.size()), &got, 200);
        total += got;
        if (status != FT_OK || got == 0) break;
    }
    return total;
}

void Ft600::submit(Slot &slot)
{
    slot.transferred = 0;
    FT_STATUS status = FT_ReadPipeAsync(handle_, kInFifo, slot.buffer.data(), ULONG(slot.buffer.size()),
                                        &slot.transferred, &slot.overlapped);
    if (status != FT_IO_PENDING) check(status, "read submission");
    slot.pending = true;
}

void Ft600::start()
{
    for (auto &slot : slots_) submit(*slot);
    next_ = 0;
}

bool Ft600::next(std::vector<uint8_t> &data)
{
    Slot &slot = *slots_[next_];
    next_ = (next_ + 1) % kInFlight;
    // The completed length is kept apart from the submission's output
    // variable, which the library may still be writing.
    ULONG completed = 0;
    FT_STATUS status = FT_GetOverlappedResult(handle_, &slot.overlapped, &completed, TRUE);
    slot.pending = false;
    if (status != FT_TIMEOUT) check(status, "read");
    if (completed > slot.buffer.size()) throw UsbError("FT600 returned an oversized transfer");
    data.swap(slot.buffer);
    data.resize(completed);
    slot.buffer.resize(kTransferBytes);
    submit(slot);
    return completed != 0;
}

void Ft600::abort()
{
    if (handle_) FT_AbortPipe(handle_, kInPipe);
}

void Ft600::stop()
{
    if (!handle_) return;
    FT_AbortPipe(handle_, kInPipe);
    for (auto &slot : slots_) {
        if (slot && slot->pending) {
            ULONG ignored = 0;
            FT_GetOverlappedResult(handle_, &slot->overlapped, &ignored, TRUE);
            slot->pending = false;
        }
    }
}
