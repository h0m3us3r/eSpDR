/*
 * Board wiring and the ESP32-S3 registers this firmware uses directly.
 *
 * The radio-side registers are undocumented by Espressif. Their names below
 * describe how this firmware uses them; the values written to them reproduce
 * the vendor PHY library's own receive configuration.
 */
#pragma once

#include <stdint.h>

#define REG(address) (*(volatile uint32_t *)(address))

/* Link GPIOs in lane order: lane 0 (core 0) first, then lane 1 (core 1). */
#define LINK_GPIOS 4, 5, 6, 7, 15, 16, 17, 18, 3, 46, 9, 10, 11, 12, 13, 14
#define LINK_LINES 16

/* IO_MUX drive-strength field for a 10 mA output. GPIO17 and GPIO18 have
 * the 10 mA and 20 mA encodings swapped (ESP32-S3 errata). */
#define LINK_DRIVE_10MA 1
#define LINK_DRIVE_10MA_GPIO17_18 2

/* Four 64 KiB capture banks; the ADC writer owns one at a time. */
#define CAPTURE_BANK_BASE 0x3FCB0000u
#define CAPTURE_BANK_BYTES 0x10000u
#define CAPTURE_BANKS 4
/* The ROM's own data and .bss, at the top of capture bank 3. */
#define ROM_DATA_BASE 0x3FCED710u
#define ROM_DATA_BYTES 0x28F0u

/* ADC sample dump engine. */
#define DUMP_CTRL_REG 0x60033D5Cu       /* bit 31 run, bits 0..15 ring length */
#define DUMP_WRITE_INDEX_REG 0x60033D60u
#define DUMP_CONFIG_REG 0x60033D90u
#define DUMP_BANK_SELECT_REG 0x600C101Cu /* bits 0..3 route the writer to bank 0..3 */
#define DUMP_CTRL_RUN 0x80000000u
#define DUMP_CTRL_CIRCULAR 0x00024000u   /* circular 16384-pair ring, IQ source 0 */
#define DUMP_CTRL_16MSPS 0x00010000u     /* dump clock 16 Msps instead of 80 */
#define DUMP_CONFIG_IQ 0x000C2040u

/* Baseband, AGC and front-end control. */
#define BB_ENABLE_REG 0x6002600Cu        /* bit 1 BB enable, bits 2..3 width, bit 28 MAC BB */
#define AGC_CTRL_REG 0x6001C01Cu
#define AGC_GAIN_FORCE_REG 0x6001C02Cu   /* bits 24..30 gain selector, bit 23 force */
#define AGC_DISABLE_REG 0x6001C034u
#define AGC_RX_FORCE_REG 0x6001C080u
#define IQ_CORRECTION_REG 0x6000607Cu    /* receive I/Q amplitude and phase correction */
#define FE_WIDTH_REG 0x60006100u         /* bits 16..21 analog channel width */
#define PBUS_CTRL_REG 0x60006104u        /* bit 0 software owner, bit 1 write strobe */
#define PBUS_MODE_REG 0x6000610Cu
#define PBUS_STATUS_REG 0x60006110u      /* bit 31 busy, bits 14..15 RX force */
#define PBUS_BB_GAIN_REG 0x60006118u     /* bits 9..17 current baseband gain */
#define PBUS_RF_GAIN_REG 0x6000611Cu     /* bits 18..26 current RF gain */
#define RFPLL_OWNER_REG 0x6000E0C4u      /* bit 25 software RFPLL control */

/* Analog (regi2c) blocks. */
#define I2C_RFPLL 0x62
#define I2C_SDM 0x63
#define I2C_BB_FILTER 0x67
