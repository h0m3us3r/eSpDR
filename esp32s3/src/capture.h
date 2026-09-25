/* Continuous two-core acquisition and link transmission. */
#pragma once

#include <stdint.h>

/*
 * Streams IQ over both link lanes until `seconds` have elapsed (0: until the
 * host sends a byte). Blocks on core 0 and returns 0 or an ESP_FAIL_* code.
 * The link outputs must already be enabled.
 */
unsigned capture_run(unsigned seconds);

/* Statistics of the last run, indexed by ESP_STAT_*. */
uint32_t capture_stat(unsigned index);
