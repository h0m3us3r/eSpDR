/* SPDX-License-Identifier: 0BSD
 * ESP32-S3 receive-LO planning, nominal 40 MHz crystal.
 * Standalone C/C++: no SDK, FPGA, allocation or floating point required.
 * See docs/LO-EXTENSION.md for the register sequence and measured limits.
 */
#ifndef ESP32S3_LO_PLAN_H
#define ESP32S3_LO_PLAN_H

#include <stdbool.h>
#include <stdint.h>

/* Software request limits; each board must still pass PLL calibration. */
#define ESP32S3_PLL_MIN_HZ 2210000000u
#define ESP32S3_PLL_MAX_HZ 2790000000u
#define ESP32S3_LO_MIN_HZ 1841666667u /* ceil(PLL_MIN * 5/6) */
#define ESP32S3_LO_5_6_MAX_HZ 2325000000u

#define ESP32S3_CKGEN_BLOCK 0x65
#define ESP32S3_CKGEN_REG 0
#define ESP32S3_CKGEN_5_6_BIT 0x10

enum esp32s3_lo_mode {
    ESP32S3_LO_AUTO = 0,
    ESP32S3_LO_NORMAL = 1,
    ESP32S3_LO_5_6 = 2,
};

struct esp32s3_lo_plan {
    uint32_t sdm_word;
    uint32_t pll_hz; /* normal-mode coordinate, rounded to integer Hz */
    uint32_t lo_hz;  /* effective receive coordinate, rounded to integer Hz */
    enum esp32s3_lo_mode mode;
};

/* AUTO keeps normal conversion from 2210 MHz up, uses 5/6 below it.
 * Explicit modes let other projects compare both paths in their overlap.
 * Does not touch hardware or modify *out on an invalid request.
 * Frequencies are nominal; crystal error is not corrected here.
 */
static inline bool esp32s3_plan_lo(uint32_t hz, enum esp32s3_lo_mode mode,
                                  struct esp32s3_lo_plan *out)
{
    if (!out)
        return false;
    if (mode == ESP32S3_LO_AUTO)
        mode = hz < ESP32S3_PLL_MIN_HZ ? ESP32S3_LO_5_6 : ESP32S3_LO_NORMAL;
    uint32_t step_hz;
    if (mode == ESP32S3_LO_NORMAL) {
        if (hz < ESP32S3_PLL_MIN_HZ || hz > ESP32S3_PLL_MAX_HZ)
            return false;
        step_hz = 30000000u;
    } else if (mode == ESP32S3_LO_5_6) {
        if (hz < ESP32S3_LO_MIN_HZ || hz > ESP32S3_LO_5_6_MAX_HZ)
            return false;
        step_hz = 25000000u;
    } else {
        return false;
    }
    /* LO = step_hz * (32 + W/65536). Round once, in the requested mode. */
    uint64_t scaled = ((uint64_t)hz * 65536u + step_hz / 2u) / step_hz;
    struct esp32s3_lo_plan plan;
    plan.sdm_word = (uint32_t)(scaled - 32u * 65536u);
    plan.pll_hz = (uint32_t)((30000000ull * scaled + 32768u) / 65536u);
    plan.lo_hz = (uint32_t)((step_hz * scaled + 32768u) / 65536u);
    plan.mode = mode;
    *out = plan;
    return true;
}

#endif
