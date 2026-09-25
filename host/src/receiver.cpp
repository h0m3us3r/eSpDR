#include "receiver.h"

#include <cmath>
#include <cstdlib>
#include <stdexcept>

#include "control.h"

namespace {

[[noreturn]] void bad(const std::string &text, const char *why)
{
    throw std::runtime_error("receiver setting " + text + ": " + why);
}

long parse_integer(const std::string &text, const std::string &setting, long low, long high)
{
    char *end = nullptr;
    long value = std::strtol(text.c_str(), &end, 10);
    if (text.empty() || *end || value < low || value > high)
        bad(setting, ("needs a whole number from " + std::to_string(low) + " to " + std::to_string(high)).c_str());
    return value;
}

// A number, or ESP_AUTO for "auto".
uint32_t parse_code(const std::string &text, const std::string &setting, long high)
{
    return text == "auto" ? ESP_AUTO : uint32_t(parse_integer(text, setting, 0, high));
}

int32_t sign_extend(uint32_t value, unsigned bits)
{
    uint32_t sign = 1u << (bits - 1);
    return int32_t((value & ((sign << 1) - 1)) ^ sign) - int32_t(sign);
}

}  // namespace

double ReceiverState::sample_rate() const { return rate == ESP_RATE_16M ? 16e6 : 80e6; }
int ReceiverState::iq_amplitude() const { return sign_extend(iq, 5); }
int ReceiverState::iq_phase() const { return sign_extend(iq >> 8, 6); }

ReceiverState read_receiver(ControlPort &esp)
{
    ReceiverState s;
    s.radio = esp.command(CTL_STATUS, ESP_STAT_RADIO);
    s.lo_hz = esp.command(CTL_STATUS, ESP_STAT_LO_HZ);
    s.rate = esp.command(CTL_STATUS, ESP_STAT_RATE);
    s.width = esp.command(CTL_STATUS, ESP_STAT_WIDTH);
    s.filter = esp.command(CTL_STATUS, ESP_STAT_FILTER);
    s.gain = esp.command(CTL_STATUS, ESP_STAT_GAIN);
    s.rf_gain = esp.command(CTL_STATUS, ESP_STAT_RF_GAIN);
    s.bb_gain = esp.command(CTL_STATUS, ESP_STAT_BB_GAIN);
    for (unsigned r = 0; r < 4; ++r) s.dc[r] = esp.command(CTL_STATUS, ESP_STAT_DC0 + r);
    s.iq = esp.command(CTL_STATUS, ESP_STAT_IQ);
    s.automatic = esp.command(CTL_STATUS, ESP_STAT_AUTOMATIC);
    s.pll = esp.command(CTL_STATUS, ESP_STAT_PLL);
    return s;
}

ReceiverChange parse_receiver_setting(const std::string &text)
{
    size_t equals = text.find('=');
    if (equals == std::string::npos) bad(text, "expected NAME=VALUE");
    std::string name = text.substr(0, equals), value = text.substr(equals + 1);
    size_t comma = value.find(',');
    std::string first = value.substr(0, comma);
    std::string second = comma == std::string::npos ? "" : value.substr(comma + 1);

    ReceiverChange change;
    if (name == "lo") {
        char *end = nullptr;
        double hz = std::strtod(value.c_str(), &end);
        std::string suffix = end;
        if (suffix == "k") hz *= 1e3;
        else if (suffix == "M") hz *= 1e6;
        else if (suffix == "G") hz *= 1e9;
        else if (!suffix.empty() || value.empty()) bad(text, "needs a frequency such as 2402M");
        if (!(hz >= ESP_LO_MIN_HZ && hz <= ESP_LO_MAX_HZ))
            bad(text, "the LO must be from 2210 to 2790 MHz");
        change = {ESP_SET_LO, uint32_t(std::llround(hz))};
    } else if (name == "rate") {
        if (value == "80") change = {ESP_SET_RATE, ESP_RATE_80M};
        else if (value == "16") change = {ESP_SET_RATE, ESP_RATE_16M};
        else bad(text, "the rate is 80 or 16 (Msps)");
    } else if (name == "width") {
        if (value != "20" && value != "40") bad(text, "the width is 20 or 40 (MHz)");
        change = {ESP_SET_WIDTH, uint32_t(std::atoi(value.c_str()))};
    } else if (name == "filter") {
        long a = parse_integer(first, text, 0, 63);
        long b = second.empty() ? a : parse_integer(second, text, 0, 63);
        change = {ESP_SET_FILTER, uint32_t(a | b << 8)};
    } else if (name == "gain") {
        change = {ESP_SET_GAIN, uint32_t(parse_integer(value, text, 0, 127))};
    } else if (name == "rf") {
        change = {ESP_SET_RF_GAIN, parse_code(value, text, 511)};
    } else if (name == "bb") {
        change = {ESP_SET_BB_GAIN, parse_code(value, text, 127)};
    } else if (name.size() == 3 && name.compare(0, 2, "dc") == 0 && name[2] >= '0' && name[2] <= '3') {
        uint32_t code = value == "auto" ? ESP_DC_AUTO : uint32_t(parse_integer(value, text, 0, 511));
        change = {ESP_SET_DC, uint32_t(name[2] - '0') << 12 | code};
    } else if (name == "iq") {
        if (value == "auto") {
            change = {ESP_SET_IQ, ESP_AUTO};
        } else {
            if (second.empty()) bad(text, "needs AMPLITUDE,PHASE or auto");
            long amplitude = parse_integer(first, text, -16, 15), phase = parse_integer(second, text, -32, 31);
            change = {ESP_SET_IQ, uint32_t(amplitude & 31) | uint32_t(phase & 63) << 8};
        }
    } else {
        bad(text, "unknown name (lo, rate, width, filter, gain, rf, bb, dc0..dc3, iq)");
    }
    return change;
}

uint32_t apply_receiver_change(ControlPort &esp, const ReceiverChange &change)
{
    uint32_t value = change.value;
    if (value > 0xFFFF) esp.command(ESP_ARG_HIGH, value >> 16);
    ControlPort::Reply reply = esp.request(change.op, value & 0xFFFF, std::chrono::milliseconds(2000));
    if (reply.status == CTL_BAD_ARGUMENT) throw std::runtime_error("the ESP refused the setting");
    if (reply.status == CTL_FAILED)
        throw std::runtime_error(change.op == ESP_SET_LO ? "the RF PLL did not lock; the previous LO is kept"
                                                         : "the receiver could not be configured; kept as it was");
    if (reply.status != CTL_OK) throw std::runtime_error("receiver setting: " + control_status_name(reply.status));
    return reply.value;
}

std::string describe_receiver(const ReceiverState &s)
{
    auto code = [&](uint32_t value, unsigned automatic_bit) {
        return std::to_string(value) + (s.automatic & automatic_bit ? "(auto)" : "");
    };
    std::string text = "lo=" + std::to_string(s.lo_hz) + " rate=" + (s.rate == ESP_RATE_16M ? "16" : "80") +
                       " width=" + std::to_string(s.width) + " filter=" + std::to_string(s.filter_first()) + "," +
                       std::to_string(s.filter_second()) + " gain=" + std::to_string(s.gain) +
                       " rf=" + code(s.rf_gain, 1) + " bb=" + code(s.bb_gain, 2);
    for (unsigned r = 0; r < 4; ++r) text += " dc" + std::to_string(r) + "=" + code(s.dc[r], 4u << r);
    text += " iq=" + std::to_string(s.iq_amplitude()) + "," + std::to_string(s.iq_phase()) +
            (s.automatic & 64 ? "(auto)" : "");
    text += " pll=" + std::to_string(s.pll_cap()) + "[" + std::to_string(s.pll_first()) + "+" +
            std::to_string(s.pll_length()) + "]";
    if (s.radio != ESP_RADIO_OK) text += " radio=FAILED(" + std::to_string(s.radio) + ")";
    return text;
}
