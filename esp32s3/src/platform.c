#include "platform.h"

#include "hal/clk_gate_ll.h"
#include "hal/cpu_utility_ll.h"
#include "hal/systimer_ll.h"
#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/timer_group_reg.h"
#include "soc/usb_serial_jtag_reg.h"

#define CPU_MHZ 240
#define WDT_WRITE_KEY 0x50D83AA1u
#define USB_PACKET_BYTES 64
#define USB_WRITE_TIMEOUT_CYCLES (CPU_MHZ * 50000u) /* 50 ms per packet */

extern void ets_efuse_get_mac(uint8_t *mac);
extern void ets_set_appcpu_boot_addr(uint32_t address);
extern void core1_start(void); /* startup.S */

volatile uint32_t core1_alive;

static void disable_watchdogs(void)
{
    for (unsigned group = 0; group < 2; group++) {
        REG(TIMG_WDTWPROTECT_REG(group)) = WDT_WRITE_KEY;
        REG(TIMG_WDTCONFIG0_REG(group)) &= ~TIMG_WDT_EN;
        REG(TIMG_WDTWPROTECT_REG(group)) = 0;
    }
    REG(RTC_CNTL_WDTWPROTECT_REG) = WDT_WRITE_KEY;
    REG(RTC_CNTL_WDTCONFIG0_REG) &= ~RTC_CNTL_WDT_EN;
    REG(RTC_CNTL_WDTWPROTECT_REG) = 0;
}

void platform_init(void)
{
    disable_watchdogs();

    rtc_config_t rtc = RTC_CONFIG_DEFAULT();
    rtc_init(rtc);
    rtc_cpu_freq_config_t cpu;
    if (!rtc_clk_cpu_freq_mhz_to_config(CPU_MHZ, &cpu))
        for (;;) {
        }
    rtc_clk_cpu_freq_set_config(&cpu);

    periph_ll_enable_clk_clear_rst(PERIPH_SYSTIMER_MODULE);
    systimer_ll_enable_clock(&SYSTIMER, true);
    systimer_ll_enable_counter(&SYSTIMER, 0, true);

    /* rtc_init() may re-arm the RTC watchdog. */
    disable_watchdogs();
}

void start_core1(void)
{
    cpu_utility_ll_unstall_cpu(1);
    cpu_utility_ll_enable_clock_and_reset_app_cpu();
    ets_set_appcpu_boot_addr((uint32_t)core1_start);
    while (!core1_alive) {
    }
}

uint64_t timer_ticks(void)
{
    systimer_ll_counter_snapshot(&SYSTIMER, 0);
    while (!systimer_ll_is_counter_value_valid(&SYSTIMER, 0)) {
    }
    return ((uint64_t)systimer_ll_get_counter_value_high(&SYSTIMER, 0) << 32) |
           systimer_ll_get_counter_value_low(&SYSTIMER, 0);
}

/* Cycle-counter delay. The ROM delay routine keeps its calibration in SRAM
 * that the capture ring overwrites, so it must not be used after a capture. */
void delay_us(uint32_t us)
{
    uint32_t start = cpu_cycles();
    while (cpu_cycles() - start < us * CPU_MHZ) {
    }
}

/* Bulk IN packets are flushed every 64 bytes and at the end of each write.
 * A write gives up after 50 ms without host progress, so a disconnected
 * host cannot wedge the firmware. */
void serial_write(const void *data, size_t size)
{
    const uint8_t *bytes = data;
    unsigned in_packet = 0;
    uint32_t waiting_since = cpu_cycles();
    while (size) {
        if (!(REG(USB_SERIAL_JTAG_EP1_CONF_REG) & USB_SERIAL_JTAG_SERIAL_IN_EP_DATA_FREE)) {
            if (cpu_cycles() - waiting_since > USB_WRITE_TIMEOUT_CYCLES)
                return;
            continue;
        }
        REG(USB_SERIAL_JTAG_EP1_REG) = *bytes++;
        size--;
        if (++in_packet == USB_PACKET_BYTES) {
            REG(USB_SERIAL_JTAG_EP1_CONF_REG) = USB_SERIAL_JTAG_WR_DONE;
            in_packet = 0;
            waiting_since = cpu_cycles();
        }
    }
    /* A write ending exactly on a packet boundary needs a zero-length packet
     * to complete the host's transfer; shorter final packets end it anyway. */
    if (!in_packet) {
        waiting_since = cpu_cycles();
        while (!(REG(USB_SERIAL_JTAG_EP1_CONF_REG) & USB_SERIAL_JTAG_SERIAL_IN_EP_DATA_FREE))
            if (cpu_cycles() - waiting_since > USB_WRITE_TIMEOUT_CYCLES)
                return;
    }
    REG(USB_SERIAL_JTAG_EP1_CONF_REG) = USB_SERIAL_JTAG_WR_DONE;
}

bool serial_rx_pending(void)
{
    return REG(USB_SERIAL_JTAG_EP1_CONF_REG) & USB_SERIAL_JTAG_SERIAL_OUT_EP_DATA_AVAIL;
}

int serial_read(void)
{
    if (!serial_rx_pending())
        return -1;
    return REG(USB_SERIAL_JTAG_EP1_REG) & 0xFF;
}

void read_mac(uint8_t mac[6]) { ets_efuse_get_mac(mac); }

/* The toolchain's freestanding code may still reference these. */
void __attribute__((noreturn)) abort(void)
{
    for (;;)
        __asm__ volatile("nop");
}

void __assert_func(const char *file, int line, const char *function, const char *expression)
{
    (void)file;
    (void)line;
    (void)function;
    (void)expression;
    abort();
}
