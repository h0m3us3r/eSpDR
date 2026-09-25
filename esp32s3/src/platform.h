/* Freestanding runtime: clocks, timers, USB serial and the second core. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "board.h"

/* Code placed here runs on core 1 only. It lives in its own instruction SRAM
 * bank so neither core stalls the other's fetches while streaming. */
#define CORE1_CODE __attribute__((section(".core1_text"), noinline))

static inline void memory_barrier(void) { __asm__ volatile("memw" ::: "memory"); }

static inline uint32_t cpu_cycles(void)
{
    uint32_t cycles;
    __asm__ volatile("rsr.ccount %0" : "=a"(cycles));
    return cycles;
}

void platform_init(void);        /* 240 MHz CPU, system timer, watchdogs off */
void start_core1(void);          /* releases core 1 into core1_main() */
uint64_t timer_ticks(void);      /* 16 MHz system timer, 64 bits */
void delay_us(uint32_t us);      /* busy-wait on the cycle counter */

void serial_write(const void *data, size_t size);
int serial_read(void);              /* next received byte, or -1 */
bool serial_rx_pending(void);

void read_mac(uint8_t mac[6]);

extern volatile uint32_t core1_alive;
void core1_main(void);
