#include <cassert>
#include <cstdio>
#include <stdexcept>
#include "control.h"
#include "receiver.h"

int main()
{
    for (const char *text : {"lo=1841.666667M", "lo=1.9G", "lo=2000000000", "lo=2210M", "lo=2790M"})
        assert(parse_receiver_setting(text).op == ESP_SET_LO);
    for (const char *text : {"lo=1841.666666M", "lo=1840M", "lo=2790000001", "lo=nan", "lo=inf", "lo=-1"}) {
        bool refused = false;
        try { parse_receiver_setting(text); } catch (const std::runtime_error &) { refused = true; }
        assert(refused);
    }
    ReceiverState s;
    s.lo_hz = 2000000000u; s.pll_hz = 2400000000u; s.lo_mode = 2; s.sdm_word = 0x300000;
    const auto text = describe_receiver(s);
    assert(text.find("lo=2000000000") != std::string::npos);
    assert(text.find("lo_mode=5/6 pll_hz=2400000000 sdm=3145728") != std::string::npos);
    puts("PASS receiver parsing and effective-LO status");
}
