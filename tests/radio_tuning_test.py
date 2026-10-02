#!/usr/bin/env python3
"""Run the production tuner/settings code against a simulated analog bus.

Only SDK includes, PHY power-up and Xtensa-only vendor hooks are replaced.
The PLL scan, reconfiguration, settings and rollback functions run unchanged.
This checks sequencing and failure recovery, not RF reception.
"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'esp32s3/src/radio.c').read_text()
source = source[source.index('/* Requested settings'):]
power = source.index('/* ---- PHY power-up')
pll = source.index('/* ---- RF PLL')
source = source[:power] + source[pll:]
source = source[:source.index('/* ---- hooks required')]

prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "radio.h"
#include "lo_plan.h"
#include "control.h"
#include "board.h"
#undef REG
#define PBUS_TIMEOUT_CYCLES 24000u
#define IQ_FIELDS 0x1FFF0000u
#define IQ_MANUAL 0x08000000u
static uint8_t bus[128][256];
static uint32_t fail_word = UINT32_MAX;
static unsigned scans;
static uint32_t *reg_at(uint32_t address) {
    static uint32_t addresses[64], values[64];
    static unsigned count;
    for (unsigned n = 0; n < count; n++) if (addresses[n] == address) return &values[n];
    assert(count < 64); addresses[count] = address; return &values[count++];
}
#define REG(address) (*reg_at(address))
static uint32_t word(void) {
    return (uint32_t)bus[0x63][3] << 16 | (uint32_t)bus[0x63][4] << 8 | bus[0x63][5];
}
static unsigned esp_rom_regi2c_read(unsigned block, unsigned host, unsigned reg) {
    assert(host == 1);
    if (block == 0x62 && reg == 7) return 2;
    if (block == 0x62 && reg == 12) {
        assert(!(bus[0x65][0] & 0x10)); /* all calibration in normal conversion */
        ++scans;
        unsigned cap = bus[0x62][1] | ((bus[0x62][2] & 0x10) << 4);
        return word() != fail_word && cap >= 200 && cap < 210 ? 0 : 4;
    }
    return bus[block][reg];
}
static void esp_rom_regi2c_write(unsigned block, unsigned host, unsigned reg, unsigned value) {
    assert(host == 1);
    if (block == 0x63) assert(!(bus[0x65][0] & 0x10));
    bus[block][reg] = value;
}
static void delay_us(unsigned us) { (void)us; }
static uint32_t cpu_cycles(void) { return 0; }
static unsigned rom_pbus_rd(unsigned block, unsigned index) { (void)block; (void)index; return 0; }
static void power_up_modem(void) {}
static bool calibrate_phy(void) { bus[0x65][0] = 0x63; return true; }
'''

suffix = r'''
static void state(uint32_t lo, unsigned mode, uint8_t ckgen) {
    assert(radio_stat(ESP_STAT_RADIO) == ESP_RADIO_OK);
    assert(radio_stat(ESP_STAT_LO_HZ) == lo);
    assert(radio_stat(ESP_STAT_LO_MODE) == mode);
    assert(bus[0x65][0] == ckgen);
    assert(word() == radio_stat(ESP_STAT_SDM_WORD));
}
int main(void) {
    uint32_t effective = 0;
    assert(radio_init() == ESP_RADIO_OK);
    state(2439999847u, 1, 0x63);
    assert(scans == 512);
    assert(radio_set(ESP_SET_LO, 2000000000u, &effective) == CTL_OK);
    assert(effective == 2000000000u);
    state(2000000000u, 2, 0x73);
    assert(scans == 1024);
    assert(radio_stat(ESP_STAT_PLL_HZ) == 2400000000u);
    assert(radio_set(ESP_SET_GAIN, 30, &effective) == CTL_OK);
    assert(radio_set(ESP_SET_RATE, ESP_RATE_16M, &effective) == CTL_OK);
    state(2000000000u, 2, 0x73);
    assert(scans == 1024); /* unrelated settings preserve the mode without a rescan */
    struct esp32s3_lo_plan bad;
    assert(esp32s3_plan_lo(1900000000u, ESP32S3_LO_AUTO, &bad));
    fail_word = bad.sdm_word;
    assert(radio_set(ESP_SET_LO, 1900000000u, &effective) == CTL_FAILED);
    state(2000000000u, 2, 0x73); /* failed alternate tune restores alternate mode */
    assert(scans == 2048);
    assert(radio_set(ESP_SET_LO, 2440000000u, &effective) == CTL_OK);
    state(2439999847u, 1, 0x63);
    assert(radio_set(ESP_SET_LO, 1900000000u, &effective) == CTL_FAILED);
    state(2439999847u, 1, 0x63); /* failed alternate tune restores normal mode */
    assert(radio_set(ESP_SET_LO, ESP_LO_MIN_HZ - 1, &effective) == CTL_BAD_ARGUMENT);
    state(2439999847u, 1, 0x63);
    bus[0x65][0] = 0xE3; /* unrelated CKGEN bits survive both directions */
    assert(radio_set(ESP_SET_LO, 2000000000u, &effective) == CTL_OK);
    state(2000000000u, 2, 0xF3);
    assert(radio_set(ESP_SET_LO, 2440000000u, &effective) == CTL_OK);
    state(2439999847u, 1, 0xE3);
    puts("PASS firmware calibration order, mode changes, readback and failed-tune rollback");
}
'''

with tempfile.TemporaryDirectory() as tmp:
    c = Path(tmp) / 'radio_test.c'
    exe = Path(tmp) / 'radio_test'
    c.write_text(prefix + source + suffix)
    subprocess.run(['cc', '-std=gnu11', '-O2', '-Wall', '-Wextra', '-Werror',
                    '-I' + str(root / 'esp32s3/src'), '-I' + str(root / 'protocol'),
                    str(c), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
