#include "serial.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/serial.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "control.h"
#include "iq_record.h"

namespace {

void store_le(uint8_t *p, uint32_t value, unsigned bytes)
{
    for (unsigned i = 0; i < bytes; ++i) p[i] = uint8_t(value >> (8 * i));
}

uint32_t load_le(const uint8_t *p, unsigned bytes)
{
    uint32_t value = 0;
    for (unsigned i = 0; i < bytes; ++i) value |= uint32_t(p[i]) << (8 * i);
    return value;
}

}  // namespace

std::string control_status_name(unsigned status)
{
    switch (status) {
    case CTL_OK: return "ok";
    case CTL_UNKNOWN_OP: return "unknown operation";
    case CTL_BAD_ARGUMENT: return "bad argument";
    case CTL_BUSY: return "busy";
    case CTL_NOT_READY: return "not ready";
    case CTL_RUN_FAILED: return "run failed";
    case CTL_FAILED: return "failed";
    default: return "status " + std::to_string(status);
    }
}

ControlPort::ControlPort(const std::string &path, unsigned node) : node_(node), path_(path)
{
    fd_ = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) throw ControlError("cannot open " + path + ": " + std::strerror(errno));
    if (ioctl(fd_, TIOCEXCL) != 0) {
        close(fd_);
        throw ControlError(path + " is in use");
    }
    termios tty{};
    tcgetattr(fd_, &tty);
    cfmakeraw(&tty);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(CRTSCTS | HUPCL);
    // The FPGA UART runs at 1 Mbaud; the ESP's USB serial ignores the rate.
    speed_t speed = node == CTL_NODE_FPGA ? B1000000 : B115200;
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        close(fd_);
        throw ControlError("cannot configure " + path);
    }
    tcflush(fd_, TCIOFLUSH);
    // FTDI bridges otherwise hold a short reply for up to 16 ms; this makes
    // each request about 1 ms, so live statistics can be polled. Best effort.
    serial_struct serial{};
    if (ioctl(fd_, TIOCGSERIAL, &serial) == 0 && !(serial.flags & ASYNC_LOW_LATENCY)) {
        serial.flags |= ASYNC_LOW_LATENCY;
        ioctl(fd_, TIOCSSERIAL, &serial);
    }
}

ControlPort::~ControlPort()
{
    if (fd_ >= 0) close(fd_);
}

uint16_t ControlPort::send(unsigned op, unsigned arg)
{
    uint8_t request[CTL_REQUEST_BYTES] = {CTL_REQUEST_MAGIC, uint8_t(op)};
    store_le(request + 2, arg, 2);
    store_le(request + 4, ++sequence_, 2);
    store_le(request + 6, iqr::crc32(request, 6), 4);
    size_t written = 0;
    while (written < sizeof(request)) {
        ssize_t n = write(fd_, request + written, sizeof(request) - written);
        if (n > 0) {
            written += size_t(n);
        } else if (n < 0 && errno != EAGAIN) {
            throw ControlError(path_ + ": write failed");
        } else {
            pollfd p{fd_, POLLOUT, 0};
            poll(&p, 1, 100);
        }
    }
    return sequence_;
}

bool ControlPort::read_frame(uint8_t *frame, Clock::time_point deadline)
{
    for (;;) {
        // Drop bytes until a frame with a valid CRC starts the buffer. Bytes
        // of an incomplete frame stay buffered for the next call.
        while (!buffer_.empty()) {
            auto magic = std::find(buffer_.begin(), buffer_.end(), uint8_t(CTL_RESPONSE_MAGIC));
            buffer_.erase(buffer_.begin(), magic);
            if (buffer_.size() < CTL_RESPONSE_BYTES) break;
            if (load_le(buffer_.data() + 12, 4) != iqr::crc32(buffer_.data(), 12)) {
                buffer_.erase(buffer_.begin());
                continue;
            }
            std::memcpy(frame, buffer_.data(), CTL_RESPONSE_BYTES);
            buffer_.erase(buffer_.begin(), buffer_.begin() + CTL_RESPONSE_BYTES);
            if (frame[1] != node_) throw ControlError(path_ + ": response from the wrong device");
            return true;
        }

        auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
        if (left.count() <= 0) return false;
        pollfd p{fd_, POLLIN, 0};
        if (poll(&p, 1, int(std::min<long long>(left.count(), 100))) < 0 && errno != EINTR)
            throw ControlError(path_ + ": poll failed");
        if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) throw ControlError(path_ + " disconnected");
        if (!(p.revents & POLLIN)) continue;
        uint8_t bytes[256];
        ssize_t n = read(fd_, bytes, sizeof(bytes));
        if (n > 0) buffer_.insert(buffer_.end(), bytes, bytes + n);
    }
}

bool ControlPort::receive(unsigned op, uint16_t sequence, Reply &reply,
                          std::chrono::milliseconds timeout)
{
    auto deadline = Clock::now() + timeout;
    uint8_t frame[CTL_RESPONSE_BYTES];
    // Replies to other requests, such as a run left behind by a previous
    // host process, are skipped.
    while (read_frame(frame, deadline)) {
        if (frame[2] == op && load_le(frame + 4, 2) == sequence) {
            reply.status = frame[3];
            reply.value = load_le(frame + 8, 4);
            return true;
        }
    }
    return false;
}

ControlPort::Reply ControlPort::request(unsigned op, unsigned arg, std::chrono::milliseconds timeout)
{
    uint16_t sequence = send(op, arg);
    Reply reply;
    if (!receive(op, sequence, reply, timeout))
        throw ControlError(path_ + ": no response to operation " + std::to_string(op));
    return reply;
}

uint32_t ControlPort::command(unsigned op, unsigned arg, std::chrono::milliseconds timeout)
{
    Reply reply = request(op, arg, timeout);
    if (reply.status != CTL_OK)
        throw ControlError(path_ + ": operation " + std::to_string(op) + ": " +
                           control_status_name(reply.status));
    return reply.value;
}
