// ESP32-S3 receiver settings (control.h ESP_SET_*): parsing, applying and
// reading them back. The ESP keeps them until it is reloaded.
#pragma once

#include <cstdint>
#include <string>

#include "serial.h"

// The settings in effect, as the ESP reports them (ESP_STAT_LO_HZ..PLL).
struct ReceiverState {
    uint32_t radio = 0;  // ESP_RADIO_*: not OK after a failed setting could not be undone
    uint32_t lo_hz = 0, rate = 0, width = 0, filter = 0, gain = 0, rf_gain = 0, bb_gain = 0;
    uint32_t dc[4] = {0, 0, 0, 0}, iq = 0, automatic = 0, pll = 0;

    double sample_rate() const;  // pairs per second
    unsigned filter_first() const { return filter & 63; }
    unsigned filter_second() const { return filter >> 8 & 63; }
    int iq_amplitude() const;    // signed codes
    int iq_phase() const;
    unsigned pll_cap() const { return pll & 511; }
    unsigned pll_first() const { return pll >> 9 & 511; }   // first capacitor code that locks
    unsigned pll_length() const { return pll >> 18; }        // codes that lock
};

ReceiverState read_receiver(ControlPort &esp);

// One change: an ESP_SET_* operation and its argument.
struct ReceiverChange {
    unsigned op = 0;
    uint32_t value = 0;
};

// Parses NAME=VALUE:
//   lo=2402M        LO frequency: Hz, or with a k, M or G suffix
//   rate=80|16      sample rate, Msps
//   width=40|20     analog channel width, MHz
//   filter=N[,M]    baseband RC filter codes 0..63 (M defaults to N)
//   gain=N          gain selector 0..127
//   rf=N|auto       RF gain stage word 0..511
//   bb=N|auto       baseband gain stage 0..127
//   dcR=N|auto      DC offset register R (0, 1: I; 2, 3: Q), code 0..511
//   iq=A,P|auto     I/Q correction: amplitude -16..15, phase -32..31
// Throws std::runtime_error on anything else.
ReceiverChange parse_receiver_setting(const std::string &text);

// Applies a change and returns the value now in effect. Throws on refusal.
uint32_t apply_receiver_change(ControlPort &esp, const ReceiverChange &change);

// The settings as NAME=VALUE words, in parse_receiver_setting's syntax.
std::string describe_receiver(const ReceiverState &state);
