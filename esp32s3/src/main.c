/*
 * ESP32-S3 IQ source firmware: command loop on core 0.
 *
 * Runs from RAM (loaded by the ROM bootloader), with no RTOS, heap or
 * interrupts. Both cores are polling loops.
 */
#include <stdbool.h>
#include <stdint.h>

#include "board.h"
#include "capture.h"
#include "control.h"
#include "hal/dedic_gpio_cpu_ll.h"
#include "platform.h"
#include "radio.h"
#include "soc/gpio_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#include "soc/system_reg.h"

#define CPU_HZ 240000000u
#define IDLE_RELEASE_CYCLES CPU_HZ /* release the link after 1 s without commands */
#define PARTIAL_REQUEST_CYCLES (CPU_HZ / 10)

static const unsigned link_gpio[LINK_LINES] = {LINK_GPIOS};

/* Dedicated-GPIO output signals: bits 0..7 of each core's port. */
static const unsigned link_signal[LINK_LINES] = {
    PRO_ALONEGPIO_OUT0_IDX, PRO_ALONEGPIO_OUT1_IDX, PRO_ALONEGPIO_OUT2_IDX, PRO_ALONEGPIO_OUT3_IDX,
    PRO_ALONEGPIO_OUT4_IDX, PRO_ALONEGPIO_OUT5_IDX, PRO_ALONEGPIO_OUT6_IDX, PRO_ALONEGPIO_OUT7_IDX,
    CORE1_GPIO_OUT0_IDX,    CORE1_GPIO_OUT1_IDX,    CORE1_GPIO_OUT2_IDX,    CORE1_GPIO_OUT3_IDX,
    CORE1_GPIO_OUT4_IDX,    CORE1_GPIO_OUT5_IDX,    CORE1_GPIO_OUT6_IDX,    CORE1_GPIO_OUT7_IDX,
};

static uint8_t mac[6];
static bool outputs_enabled;

/* ---- link pins ------------------------------------------------------------------ */

static void configure_pads(void)
{
    for (unsigned i = 0; i < LINK_LINES; i++) {
        unsigned gpio = link_gpio[i];
        unsigned drive = (gpio == 17 || gpio == 18) ? LINK_DRIVE_10MA_GPIO17_18 : LINK_DRIVE_10MA;
        REG(PERIPHS_IO_MUX_GPIO0_U + 4 * gpio) = (1u << MCU_SEL_S) | FUN_IE | (drive << FUN_DRV_S);
    }
    memory_barrier();
}

static void set_outputs(bool enable)
{
    uint32_t low = 0, high = 0;
    for (unsigned i = 0; i < LINK_LINES; i++) {
        unsigned gpio = link_gpio[i];
        if (gpio < 32)
            low |= 1u << gpio;
        else
            high |= 1u << (gpio - 32);
    }
    if (enable) {
        configure_pads();
        dedic_gpio_cpu_ll_write_all(0); /* core 1 keeps its port at 0 when idle */
        REG(GPIO_ENABLE_W1TS_REG) = low;
        REG(GPIO_ENABLE1_W1TS_REG) = high;
    } else {
        REG(GPIO_ENABLE_W1TC_REG) = low;
        REG(GPIO_ENABLE1_W1TC_REG) = high;
    }
    memory_barrier();
    outputs_enabled = enable;
}

static void init_link(void)
{
    configure_pads();
    set_outputs(false);
    REG(SYSTEM_CPU_PERI_CLK_EN_REG) |= SYSTEM_CLK_EN_DEDICATED_GPIO;
    for (unsigned i = 0; i < LINK_LINES; i++)
        REG(GPIO_FUNC0_OUT_SEL_CFG_REG + 4 * link_gpio[i]) = link_signal[i] | GPIO_FUNC0_OEN_SEL;
    dedic_gpio_cpu_ll_write_all(0);
}

/* ---- control protocol ---------------------------------------------------------------- */

static uint32_t crc32(const uint8_t *data, unsigned size)
{
    uint32_t crc = 0xFFFFFFFFu;
    while (size--) {
        crc ^= *data++;
        for (int bit = 0; bit < 8; bit++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320u : crc >> 1;
    }
    return ~crc;
}

static uint32_t load_le(const uint8_t *p, unsigned bytes)
{
    uint32_t value = 0;
    for (unsigned i = 0; i < bytes; i++)
        value |= (uint32_t)p[i] << (8 * i);
    return value;
}

static void store_le(uint8_t *p, uint32_t value, unsigned bytes)
{
    for (unsigned i = 0; i < bytes; i++)
        p[i] = (uint8_t)(value >> (8 * i));
}

static void reply(uint8_t op, uint8_t status, uint16_t sequence, uint32_t value)
{
    uint8_t response[CTL_RESPONSE_BYTES] = {CTL_RESPONSE_MAGIC, CTL_NODE_ESP, op, status};
    store_le(response + 4, sequence, 2);
    store_le(response + 8, value, 4);
    store_le(response + 12, crc32(response, 12), 4);
    serial_write(response, sizeof(response));
}

static uint32_t info(unsigned what)
{
    switch (what) {
    case 0: return CTL_ESP_FIRMWARE_ID;
    case 1: return load_le(mac, 4);
    case 2: return load_le(mac + 4, 2);
    default: return 0;
    }
}

static uint8_t execute(uint8_t op, uint32_t arg, uint32_t *value)
{
    switch (op) {
    case CTL_INFO:
        if (arg > 2)
            return CTL_BAD_ARGUMENT;
        *value = info(arg);
        return CTL_OK;
    case CTL_SAFE:
        set_outputs(false);
        return CTL_OK;
    case CTL_STATUS:
        if (arg >= ESP_STAT_COUNT)
            return CTL_BAD_ARGUMENT;
        *value = arg == ESP_STAT_RADIO || arg >= ESP_STAT_LO_HZ ? radio_stat(arg) : capture_stat(arg);
        return CTL_OK;
    case ESP_OUTPUTS:
        if (arg > 1)
            return CTL_BAD_ARGUMENT;
        set_outputs(arg);
        return CTL_OK;
    case ESP_RUN:
        if (arg > 65535)
            return CTL_BAD_ARGUMENT;
        if (radio_stat(ESP_STAT_RADIO) != ESP_RADIO_OK || !outputs_enabled)
            return CTL_NOT_READY;
        *value = capture_run(arg);
        return *value ? CTL_RUN_FAILED : CTL_OK;
    case ESP_STOP: /* the run, if any, has already ended */
    case ESP_ARG_HIGH: /* kept by the command loop */
        return CTL_OK;
    default:
        return radio_set(op, arg, value);
    }
}

void app_main(void)
{
    platform_init();
    init_link();
    start_core1();
    read_mac(mac);
    radio_init();

    uint8_t request[CTL_REQUEST_BYTES];
    unsigned received = 0;
    uint16_t arg_high = 0; /* from ESP_ARG_HIGH, for the next request only */
    uint32_t last_command = cpu_cycles(), last_byte = last_command;
    for (;;) {
        uint32_t now = cpu_cycles();
        if (outputs_enabled && now - last_command > IDLE_RELEASE_CYCLES)
            set_outputs(false);
        if (received && now - last_byte > PARTIAL_REQUEST_CYCLES) {
            received = 0; /* abandon a request whose sender went away */
            arg_high = 0;
        }

        int byte = serial_read();
        if (byte < 0)
            continue;
        last_byte = cpu_cycles();
        if (received == 0 && byte != CTL_REQUEST_MAGIC)
            continue;
        request[received++] = (uint8_t)byte;
        if (received < sizeof(request))
            continue;
        received = 0;
        if (load_le(request + 6, 4) != crc32(request, 6)) {
            /* Misaligned: resume from the next magic byte in what we have. A
             * high argument half belongs only to the request right after it. */
            arg_high = 0;
            for (unsigned i = 1; i < sizeof(request); i++) {
                if (request[i] != CTL_REQUEST_MAGIC)
                    continue;
                received = sizeof(request) - i;
                for (unsigned j = 0; j < received; j++)
                    request[j] = request[i + j];
                break;
            }
            continue;
        }

        uint8_t op = request[1];
        uint16_t arg = (uint16_t)load_le(request + 2, 2);
        uint16_t sequence = (uint16_t)load_le(request + 4, 2);
        uint32_t value = 0;
        uint8_t status = execute(op, (uint32_t)arg_high << 16 | arg, &value);
        arg_high = op == ESP_ARG_HIGH ? arg : 0;
        reply(op, status, sequence, value);
        last_command = cpu_cycles();
    }
}
