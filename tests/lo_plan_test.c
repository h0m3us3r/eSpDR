#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "control.h"
#include "lo_plan.h"

static void check(uint32_t hz, enum esp32s3_lo_mode mode)
{
    struct esp32s3_lo_plan p;
    assert(esp32s3_plan_lo(hz, mode, &p));
    double pll = 960e6 + 30e6 * p.sdm_word / 65536.0;
    double lo = p.mode == ESP32S3_LO_5_6 ? pll * 5.0 / 6.0 : pll;
    assert(fabs(lo - p.lo_hz) <= 0.5);
    assert(fabs(pll - p.pll_hz) <= 0.5);
    assert(fabs(lo - hz) <= (p.mode == ESP32S3_LO_5_6 ? 25e6 : 30e6) / 131072.0);
    assert(pll >= ESP32S3_PLL_MIN_HZ - 229.0 && pll <= ESP32S3_PLL_MAX_HZ + 229.0);
    if (p.mode == ESP32S3_LO_NORMAL) {
        /* The original production formula must give identical normal words. */
        uint64_t scaled = ((uint64_t)hz * 4 * 65536 + 3ull * 40000000 / 2) / (3ull * 40000000);
        assert(p.sdm_word == scaled - 32 * 65536);
    }
}

int main(void)
{
    assert(ESP_LO_MIN_HZ == ESP32S3_LO_MIN_HZ);
    assert(ESP_LO_MAX_HZ == ESP32S3_PLL_MAX_HZ);
    struct esp32s3_lo_plan p;
    assert(esp32s3_plan_lo(2000000000u, ESP32S3_LO_AUTO, &p));
    assert(p.sdm_word == 0x300000 && p.pll_hz == 2400000000u && p.lo_hz == 2000000000u);
    assert(p.mode == ESP32S3_LO_5_6);
    assert(esp32s3_plan_lo(2440000000u, ESP32S3_LO_AUTO, &p));
    assert(p.sdm_word == 0x315555 && p.lo_hz == 2439999847u && p.mode == ESP32S3_LO_NORMAL);
    uint32_t word = p.sdm_word;
    const uint32_t invalid[] = {0, ESP_LO_MIN_HZ - 1, ESP_LO_MAX_HZ + 1, UINT32_MAX};
    for (unsigned n = 0; n < sizeof(invalid) / sizeof(*invalid); ++n) {
        assert(!esp32s3_plan_lo(invalid[n], ESP32S3_LO_AUTO, &p));
        assert(p.sdm_word == word);
    }
    assert(!esp32s3_plan_lo(2000000000u, ESP32S3_LO_NORMAL, &p));
    assert(!esp32s3_plan_lo(2440000000u, ESP32S3_LO_5_6, &p));
    assert(!esp32s3_plan_lo(2440000000u, (enum esp32s3_lo_mode)3, &p));
    assert(!esp32s3_plan_lo(2440000000u, ESP32S3_LO_AUTO, 0));
    const uint32_t edges[] = {ESP_LO_MIN_HZ, ESP_LO_MIN_HZ + 1, ESP32S3_PLL_MIN_HZ - 1,
                             ESP32S3_PLL_MIN_HZ, ESP32S3_PLL_MIN_HZ + 1, ESP_LO_MAX_HZ};
    for (unsigned n = 0; n < sizeof(edges) / sizeof(*edges); ++n) check(edges[n], ESP32S3_LO_AUTO);
    for (uint32_t hz = ESP_LO_MIN_HZ; hz < ESP_LO_MAX_HZ; hz += 1009) check(hz, ESP32S3_LO_AUTO);
    for (uint32_t hz = ESP32S3_PLL_MIN_HZ; hz <= ESP32S3_LO_5_6_MAX_HZ; hz += 1000) {
        check(hz, ESP32S3_LO_NORMAL);
        check(hz, ESP32S3_LO_5_6);
    }
    puts("PASS LO words, rounding, bounds, overlap and normal-mode compatibility");
    return 0;
}
