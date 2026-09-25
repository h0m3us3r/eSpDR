/*
 * Receiver bring-up and settings.
 *
 * The vendor PHY library performs power-up calibration. Everything after it
 * is done here directly: the RF PLL is tuned by software, the Wi-Fi AGC is
 * disabled, the receive gain is forced, and the analog gain stages are owned
 * through the PBUS interface. No transmit path is ever enabled.
 *
 * A setting change hands the gain stages back to the hardware and repeats
 * the whole receive configuration, the sequence every setting was
 * characterised with. It runs between captures only: the ROM's analog
 * register helpers keep their state in capture bank 3, which capture.c
 * restores after every run.
 */
#include "radio.h"

#include <stdbool.h>
#include <string.h>

#include "control.h"
#include "esp_phy_init.h"
#include "esp_rom_regi2c.h"
#include "hal/clk_gate_ll.h"
#include "phy_init_data.h"
#include "platform.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/syscon_reg.h"
#include "soc/system_reg.h"

#define REFERENCE_HZ 40000000u
#define PBUS_TIMEOUT_CYCLES 24000u
#define IQ_FIELDS 0x1FFF0000u  /* I/Q correction: amplitude 20:16, phase 26:21, mode 28:27 */
#define IQ_MANUAL 0x08000000u  /* bit 27 set, bit 28 clear: the fields apply */

extern int register_chipv7_phy(const esp_phy_init_data_t *init, esp_phy_calibration_data_t *cal,
                               esp_phy_calibration_mode_t mode);
extern void phy_bbpll_en_usb(bool enable);
extern void phy_init_param_set(uint8_t param);
extern unsigned rom_pbus_rd(unsigned block, unsigned index);

/* Requested settings (control.h encodings). */
static struct {
    uint32_t lo_hz;
    unsigned rate, width, filter, gain;
    unsigned rf_gain, bb_gain;  /* or ESP_AUTO */
    unsigned dc[4];             /* or ESP_DC_AUTO */
    unsigned iq;                /* amplitude | phase << 8, or ESP_AUTO */
} settings = {
    .lo_hz = RADIO_LO_HZ, .rate = ESP_RATE_80M, .width = 40, .filter = 0, .gain = RADIO_GAIN,
    .rf_gain = ESP_AUTO, .bb_gain = ESP_AUTO,
    .dc = {ESP_DC_AUTO, ESP_DC_AUTO, ESP_DC_AUTO, ESP_DC_AUTO}, .iq = ESP_AUTO,
};

/* The receiver as configured. */
static struct {
    unsigned status;                /* ESP_RADIO_* */
    uint32_t lo_hz;                 /* exact LO */
    unsigned pll_cap, pll_first, pll_length;
    bool owned;                     /* gain stages held through the PBUS */
    uint32_t pbus_ctrl, pbus_mode;  /* before taking them */
    uint32_t iq_register;           /* before a manual I/Q correction */
    unsigned rf_gain, bb_gain;      /* stage words written */
    unsigned dc[4], dc_hardware[4]; /* codes in effect, and as the hardware set them */
} receiver;

/* DC offset registers 0..3 (ESP_SET_DC) as PBUS block and index. */
static const uint8_t dc_block[4] = {3, 3, 2, 2}, dc_index[4] = {1, 2, 1, 2};

/* ---- analog register access ---------------------------------------------- */

static uint8_t analog_read(uint8_t block, uint8_t reg) { return esp_rom_regi2c_read(block, 1, reg); }

static void analog_write(uint8_t block, uint8_t reg, uint8_t value)
{
    esp_rom_regi2c_write(block, 1, reg, value);
}

static void analog_write_bits(uint8_t block, uint8_t reg, uint8_t mask, uint8_t value)
{
    uint8_t old = analog_read(block, reg);
    analog_write(block, reg, (uint8_t)((old & ~mask) | (value & mask)));
}

/* ---- PHY power-up and calibration ------------------------------------------ */

static void power_up_modem(void)
{
    REG(RTC_CNTL_DIG_PWC_REG) &= ~RTC_CNTL_WIFI_FORCE_PD;
    delay_us(10);
    periph_ll_wifi_bt_module_enable_clk();
    REG(SYSCON_WIFI_RST_EN_REG) |= MODEM_RESET_FIELD_WHEN_PU;
    REG(SYSCON_WIFI_RST_EN_REG) &= ~MODEM_RESET_FIELD_WHEN_PU;
    REG(RTC_CNTL_DIG_ISO_REG) &= ~RTC_CNTL_WIFI_FORCE_ISO;
    periph_ll_wifi_bt_module_disable_clk();

    /* The sample dump engine needs the Wi-Fi MAC clock (bit 6), which the
     * public clock-gate mask does not include. */
    periph_ll_enable_clk_clear_rst(PERIPH_WIFI_MODULE);
    REG(SYSTEM_WIFI_CLK_EN_REG) |= 1u << 6;
}

static bool calibrate_phy(void)
{
    periph_ll_wifi_bt_module_enable_clk();
    periph_ll_phy_calibration_module_enable_clk_clear_rst();
    periph_ll_enable_clk_clear_rst(PERIPH_RNG_MODULE);
    periph_ll_wifi_module_enable_clk_clear_rst();
    periph_ll_enable_clk_clear_rst(PERIPH_BT_MODULE);
    phy_init_param_set(1);
    phy_bbpll_en_usb(true);

    static esp_phy_calibration_data_t calibration;
    memset(&calibration, 0, sizeof(calibration));
    read_mac(calibration.mac);
    int result = register_chipv7_phy(&phy_init_data, &calibration, PHY_RF_CAL_FULL);

    /* The PHY and RNG clocks stay on: the dump registers stop responding
     * without them. The Bluetooth clock is not needed. */
    periph_ll_disable_clk_set_rst(PERIPH_BT_MODULE);

    /* A full calibration from an empty record reports 1 (stored data
     * invalid), which the SDK also treats as success. */
    return result == 0 || result == 1;
}

/* ---- RF PLL ------------------------------------------------------------------ */

static void set_pll_capacitor(unsigned cap)
{
    analog_write(I2C_RFPLL, 1, (uint8_t)cap);
    analog_write_bits(I2C_RFPLL, 2, 0x10, (uint8_t)((cap >> 8) << 4));
}

static void set_pll_manual_capacitor(bool manual)
{
    analog_write_bits(I2C_RFPLL, 11, 0x40, manual ? 0x40 : 0);
}

/*
 * Programs the sigma-delta word, waits for the PLL's own calibration, then
 * scans all 512 VCO capacitor codes and pins the capacitor to the middle of
 * the longest run of codes the PLL reports as locked.
 */
static bool tune_pll(uint32_t lo_hz)
{
    /* LO = 3/4 * reference * (32 + word / 2^16), to the nearest word */
    uint64_t scaled = ((uint64_t)lo_hz * 4 * 65536 + 3ull * REFERENCE_HZ / 2) / (3ull * REFERENCE_HZ);
    uint32_t word = (uint32_t)(scaled - 32 * 65536);
    receiver.lo_hz = (uint32_t)((3ull * REFERENCE_HZ * scaled + 2 * 65536) / (4 * 65536));

    REG(RFPLL_OWNER_REG) |= 1u << 25;
    set_pll_manual_capacitor(false);
    analog_write(I2C_SDM, 0, 0x07);
    analog_write(I2C_SDM, 3, (uint8_t)(word >> 16));
    analog_write(I2C_SDM, 4, (uint8_t)(word >> 8));
    analog_write(I2C_SDM, 5, (uint8_t)word);
    analog_write(I2C_SDM, 0, 0x17);

    /* Restart calibration and wait for it to report done. */
    analog_write_bits(I2C_RFPLL, 0, 0x40, 0x40);
    analog_write_bits(I2C_RFPLL, 0, 0x20, 0x00);
    analog_write_bits(I2C_RFPLL, 0, 0x20, 0x20);
    analog_write_bits(I2C_RFPLL, 0, 0x40, 0x00);
    bool calibrated = false;
    for (unsigned poll = 0; poll < 100 && !calibrated; poll++) {
        delay_us(20);
        calibrated = analog_read(I2C_RFPLL, 7) & 2;
    }
    if (!calibrated)
        return false;
    delay_us(5);

    uint8_t saved_low = analog_read(I2C_RFPLL, 1);
    uint8_t saved_high = analog_read(I2C_RFPLL, 2);
    uint8_t saved_mode = analog_read(I2C_RFPLL, 11);
    set_pll_manual_capacitor(true);
    unsigned run_start = 0, run_length = 0, best_start = 0, best_length = 0;
    for (unsigned cap = 0; cap < 512; cap++) {
        set_pll_capacitor(cap);
        delay_us(20);
        bool locked = ((analog_read(I2C_RFPLL, 12) >> 2) & 3) == 0;
        if (!locked) {
            run_length = 0;
            continue;
        }
        if (run_length++ == 0)
            run_start = cap;
        if (run_length > best_length) {
            best_start = run_start;
            best_length = run_length;
        }
    }
    analog_write(I2C_RFPLL, 1, saved_low);
    analog_write_bits(I2C_RFPLL, 2, 0x10, saved_high);
    analog_write_bits(I2C_RFPLL, 11, 0x40, saved_mode);
    if (!best_length)
        return false;

    receiver.pll_first = best_start;
    receiver.pll_length = best_length;
    receiver.pll_cap = best_start + (best_length - 1) / 2;
    set_pll_capacitor(receiver.pll_cap);
    set_pll_manual_capacitor(true);
    return true;
}

/* ---- receive path -------------------------------------------------------------- */

/* Write one analog register through the PBUS interface. */
static bool pbus_write(unsigned block, unsigned index, unsigned value)
{
    uint32_t fields = ((value & 511u) << 6) | ((block & 15u) << 2) | ((index & 3u) << 15);
    REG(PBUS_CTRL_REG) = (REG(PBUS_CTRL_REG) & 0xFFFE0001u) | (fields & 0x1FFFCu) | 2u;
    uint32_t start = cpu_cycles();
    while (REG(PBUS_STATUS_REG) & 0x80000000u) {
        if (cpu_cycles() - start > PBUS_TIMEOUT_CYCLES) {
            REG(PBUS_CTRL_REG) &= ~2u;
            return false;
        }
    }
    REG(PBUS_CTRL_REG) &= ~2u;
    return true;
}

static void set_width(unsigned mhz)
{
    bool wide = mhz == 40;
    REG(FE_WIDTH_REG) = (REG(FE_WIDTH_REG) & ~0x003F0000u) | (wide ? 0x00120000u : 0);
    REG(BB_ENABLE_REG) = (REG(BB_ENABLE_REG) & ~0xCu) | (wide ? 0x4u : 0);
}

/* Dump engine stopped, RX forces and baseband off. */
static void park_receiver(void)
{
    REG(DUMP_CTRL_REG) &= ~DUMP_CTRL_RUN;
    REG(DUMP_BANK_SELECT_REG) &= ~15u;
    REG(AGC_RX_FORCE_REG) &= ~0xC1u;
    REG(PBUS_STATUS_REG) &= ~0xCF00u;
    REG(BB_ENABLE_REG) &= ~2u;
}

/* Parks the receiver and hands the gain stages, DC offsets and I/Q
 * correction back to the hardware as configure_receiver found them. */
static bool release_receiver(void)
{
    park_receiver();
    if (!receiver.owned)
        return true;
    bool ok = true;
    for (unsigned r = 0; r < 4; r++)
        if (receiver.dc[r] != receiver.dc_hardware[r])
            ok = pbus_write(dc_block[r], dc_index[r], receiver.dc_hardware[r]) && ok;
    ok = pbus_write(0, 1, 0) && pbus_write(1, 1, 0) && pbus_write(1, 2, 0) && ok;
    REG(PBUS_CTRL_REG) = receiver.pbus_ctrl;
    REG(PBUS_MODE_REG) = receiver.pbus_mode;
    REG(IQ_CORRECTION_REG) = receiver.iq_register;
    receiver.owned = false;
    return ok;
}

static bool configure_receiver(void)
{
    park_receiver();

    /* The width selects an analog RC bank when the baseband is enabled, so
     * it is programmed before the enable edge. */
    set_width(settings.width);

    /* Enable the baseband, disable the Wi-Fi AGC and force the RX gain. */
    REG(BB_ENABLE_REG) |= 0x10000000u;
    REG(BB_ENABLE_REG) &= ~2u;
    delay_us(1);
    REG(BB_ENABLE_REG) |= 2u;
    REG(AGC_CTRL_REG) = (REG(AGC_CTRL_REG) & 0xFF00FFFFu) | 0x007F0000u;
    REG(AGC_DISABLE_REG) |= 0x80u;
    REG(AGC_RX_FORCE_REG) |= 1u;
    REG(AGC_GAIN_FORCE_REG) =
        (REG(AGC_GAIN_FORCE_REG) & 0x007FFFFFu) | (settings.gain << 24) | 0x00800000u;
    REG(PBUS_STATUS_REG) |= 0xC000u;

    /* Baseband RC filter: registers 6/7 serve 40 MHz, 4/5 serve 20 MHz. */
    set_width(settings.width);
    unsigned filter = settings.width == 40 ? 6 : 4;
    analog_write(I2C_BB_FILTER, filter, settings.filter & 63);
    analog_write(I2C_BB_FILTER, filter + 1, (settings.filter >> 8) & 63);

    /* Let the forced gain settle, capture the gain stages it selected, then
     * take PBUS ownership and hold them there with the baseband disabled. */
    delay_us(100);
    unsigned bb = (REG(PBUS_BB_GAIN_REG) >> 9) & 511;
    unsigned rf = (REG(PBUS_RF_GAIN_REG) >> 18) & 511;
    if (settings.bb_gain != ESP_AUTO)
        bb = 0x180 | settings.bb_gain;
    if (settings.rf_gain != ESP_AUTO)
        rf = settings.rf_gain;
    receiver.pbus_ctrl = REG(PBUS_CTRL_REG);
    receiver.pbus_mode = REG(PBUS_MODE_REG);
    receiver.iq_register = REG(IQ_CORRECTION_REG);
    for (unsigned r = 0; r < 4; r++)
        receiver.dc[r] = receiver.dc_hardware[r] = 0;
    REG(PBUS_MODE_REG) &= ~0x08000000u;
    REG(PBUS_CTRL_REG) |= 1u;
    receiver.owned = true;
    REG(BB_ENABLE_REG) &= ~2u;

    /* Receive-only power configuration: both TX groups stay off. */
    bool ok = pbus_write(4, 1, 0) && pbus_write(5, 1, 0) && pbus_write(0, 1, 0x184) &&
              pbus_write(1, 1, 0x189) && pbus_write(1, 2, rf) && pbus_write(0, 1, bb);
    receiver.rf_gain = rf;
    receiver.bb_gain = bb;
    for (unsigned r = 0; r < 4 && ok; r++) {
        receiver.dc[r] = receiver.dc_hardware[r] = rom_pbus_rd(dc_block[r], dc_index[r]) & 511;
        if (settings.dc[r] != ESP_DC_AUTO && settings.dc[r] != receiver.dc[r]) {
            ok = pbus_write(dc_block[r], dc_index[r], settings.dc[r]);
            receiver.dc[r] = settings.dc[r];
        }
    }
    if (settings.iq != ESP_AUTO)
        REG(IQ_CORRECTION_REG) = (receiver.iq_register & ~IQ_FIELDS) | IQ_MANUAL |
                                 ((settings.iq & 31u) << 16) | (((settings.iq >> 8) & 63u) << 21);
    if (!ok)
        return false;

    REG(DUMP_CONFIG_REG) = DUMP_CONFIG_IQ;
    delay_us(100);
    REG(DUMP_CTRL_REG) = radio_dump_control();
    REG(DUMP_BANK_SELECT_REG) = (REG(DUMP_BANK_SELECT_REG) & ~15u) | 1u;
    return true;
}

static unsigned reconfigure(bool retune)
{
    if (!release_receiver())
        return ESP_RADIO_PBUS_FAILED;
    if (retune && !tune_pll(settings.lo_hz))
        return ESP_RADIO_PLL_FAILED;
    if (!configure_receiver())
        return ESP_RADIO_PBUS_FAILED;
    return ESP_RADIO_OK;
}

unsigned radio_init(void)
{
    power_up_modem();
    if (!calibrate_phy())
        receiver.status = ESP_RADIO_PHY_FAILED;
    else
        receiver.status = reconfigure(true);
    return receiver.status;
}

uint32_t radio_dump_control(void)
{
    return DUMP_CTRL_CIRCULAR | (settings.rate == ESP_RATE_16M ? DUMP_CTRL_16MSPS : 0);
}

unsigned radio_pairs_per_tick(void) { return settings.rate == ESP_RATE_16M ? 1 : 5; }

static bool valid_setting(unsigned op, uint32_t value)
{
    switch (op) {
    case ESP_SET_LO: return value >= ESP_LO_MIN_HZ && value <= ESP_LO_MAX_HZ;
    case ESP_SET_RATE: return value == ESP_RATE_80M || value == ESP_RATE_16M;
    case ESP_SET_WIDTH: return value == 20 || value == 40;
    case ESP_SET_FILTER: return !(value & ~0x3F3Fu);
    case ESP_SET_GAIN: return value <= 127;
    case ESP_SET_RF_GAIN: return value <= 511 || value == ESP_AUTO;
    case ESP_SET_BB_GAIN: return value <= 127 || value == ESP_AUTO;
    case ESP_SET_DC: return value >> 12 <= 3 && ((value & 0xFFF) <= 511 || (value & 0xFFF) == ESP_DC_AUTO);
    case ESP_SET_IQ: return !(value & ~0x3F1Fu) || value == ESP_AUTO;
    default: return false;
    }
}

unsigned radio_set(unsigned op, uint32_t value, uint32_t *effective)
{
    if (!valid_setting(op, value))
        return op >= ESP_SET_LO && op <= ESP_SET_IQ ? CTL_BAD_ARGUMENT : CTL_UNKNOWN_OP;
    if (receiver.status == ESP_RADIO_PHY_FAILED)
        return CTL_NOT_READY;

    typeof(settings) previous = settings;
    unsigned stat = 0;
    switch (op) {
    case ESP_SET_LO: settings.lo_hz = value; stat = ESP_STAT_LO_HZ; break;
    case ESP_SET_RATE: settings.rate = value; stat = ESP_STAT_RATE; break;
    case ESP_SET_WIDTH: settings.width = value; stat = ESP_STAT_WIDTH; break;
    case ESP_SET_FILTER: settings.filter = value; stat = ESP_STAT_FILTER; break;
    case ESP_SET_GAIN: settings.gain = value; stat = ESP_STAT_GAIN; break;
    case ESP_SET_RF_GAIN: settings.rf_gain = value; stat = ESP_STAT_RF_GAIN; break;
    case ESP_SET_BB_GAIN: settings.bb_gain = value; stat = ESP_STAT_BB_GAIN; break;
    case ESP_SET_DC: settings.dc[value >> 12] = value & 0xFFF; stat = ESP_STAT_DC0 + (value >> 12); break;
    case ESP_SET_IQ: settings.iq = value; stat = ESP_STAT_IQ; break;
    }
    /* A retune also follows a failed one, whose PLL state is unknown. */
    bool retune = settings.lo_hz != previous.lo_hz || receiver.status == ESP_RADIO_PLL_FAILED;
    receiver.status = reconfigure(retune);
    if (receiver.status != ESP_RADIO_OK) {
        settings = previous;
        receiver.status = reconfigure(true);
        return CTL_FAILED;
    }
    *effective = radio_stat(stat);
    return CTL_OK;
}

uint32_t radio_stat(unsigned index)
{
    switch (index) {
    case ESP_STAT_RADIO: return receiver.status;
    case ESP_STAT_LO_HZ: return receiver.lo_hz;
    case ESP_STAT_RATE: return settings.rate;
    case ESP_STAT_WIDTH: return settings.width;
    case ESP_STAT_FILTER: return settings.filter;
    case ESP_STAT_GAIN: return settings.gain;
    case ESP_STAT_RF_GAIN: return receiver.rf_gain;
    case ESP_STAT_BB_GAIN: return receiver.bb_gain & 127;
    case ESP_STAT_DC0:
    case ESP_STAT_DC1:
    case ESP_STAT_DC2:
    case ESP_STAT_DC3: return receiver.dc[index - ESP_STAT_DC0];
    case ESP_STAT_IQ: {
        uint32_t iq = REG(IQ_CORRECTION_REG);
        return ((iq >> 16) & 31) | (((iq >> 21) & 63) << 8);
    }
    case ESP_STAT_AUTOMATIC: {
        uint32_t automatic = (settings.rf_gain == ESP_AUTO ? 1 : 0) | (settings.bb_gain == ESP_AUTO ? 2 : 0) |
                             (settings.iq == ESP_AUTO ? 64 : 0);
        for (unsigned r = 0; r < 4; r++)
            if (settings.dc[r] == ESP_DC_AUTO)
                automatic |= 4u << r;
        return automatic;
    }
    case ESP_STAT_PLL: return receiver.pll_cap | (receiver.pll_first << 9) | (receiver.pll_length << 18);
    default: return 0;
    }
}

/* ---- hooks required by the vendor PHY library ---------------------------------- */

uint32_t phy_enter_critical(void)
{
    uint32_t ps;
    __asm__ volatile("rsil %0, 15" : "=a"(ps)::"memory");
    return ps;
}

void phy_exit_critical(uint32_t ps) { __asm__ volatile("wsr %0, ps; rsync" ::"a"(ps) : "memory"); }

int phy_printf(const char *format, ...)
{
    (void)format;
    return 0;
}

void coex_pti_print(void) {}

int64_t esp_timer_get_time(void) { return (int64_t)(timer_ticks() / 16); }
