# Extending the ESP32-S3 receive LO with 5/6 conversion

The S3's ordinary receive-LO control reaches roughly 2.2–2.8 GHz. A second
conversion mode moves the effective receive LO to **5/6 of the normal PLL
coordinate**, giving access to lower frequencies with the same RF PLL.
On the tested PHY baseline, the change is one bit: **analog CKGEN block
`0x65`, register `0`, bit `4`; `0x63` becomes `0x73`**.

Combining the two modes gives an approximate **1.84–2.79 GHz tuning
envelope**. Conducted RF tones verify the 5/6 relationship at discrete
points, and the release's live browser check reaches a nominal 1.841667 GHz
LO. The 2790 MHz request failed to lock on this board and restored the
previous LO. The request envelope's upper edge remains board-dependent.

## Using it in eSpDR

Build and load the updated ESP firmware and host tool together. ESP firmware
ID is now `0x49515306`; the FPGA image and sample format are unchanged.

```sh
iqstream set lo=1900M
iqstream status
iqstream set lo=2440M
```

`lo=` always means the **effective receive LO**. Requests below 2210 MHz use
5/6 conversion; requests from 2210 MHz up retain normal conversion. For
example, `lo=2000M` programs the ordinary PLL coordinate to 2400 MHz and
selects 5/6. Status reports `lo=2000000000 lo_mode=5/6 pll_hz=2400000000`
(among the other settings). The browser's frequency axis uses the effective
LO and its receiver panel shows the selected conversion.

The accepted integer-Hz range is **1,841,666,667–2,790,000,000 Hz**. Each tune
still has to pass the PLL capacitor scan. A failed tune restores the previous
frequency and conversion mode; a failed restoration is visible in radio
status. Gain, width, filter and sample-rate changes retain the selected mode.
Nominal frequency readback includes SDM quantization, not crystal correction.

## The idea to reuse

Separate the PLL's tuning coordinate from the frequency the receive mixer
actually uses. A clock/conversion selector can change that relationship
while the PLL stays inside its existing lock range. Here the RF PLL does
not have to lock at 1900 MHz: it uses a normal-mode coordinate of 2280 MHz,
and the alternate receive conversion puts the effective LO at 1900 MHz.

For a measured conversion ratio `r`:

```text
effective receive LO = r × ordinary PLL coordinate
ordinary PLL request = wanted receive LO / r
new tuning interval  = r × [ordinary PLL lower limit, ordinary PLL upper limit]
```

Take the union of the working modes to obtain the combined tuning envelope.
Keep the requested receive frequency separate from the programmed PLL
coordinate in APIs, status and spectrum labels. Preserve the calibration
sequence that locks the PLL; apply the conversion selector at the point in
initialization where it was tested.

The S3 implementation below is a concrete example of that method. Other ESP
variants need their own frequency-word law, clock selector and calibration
sequence; the S3's addresses and 5/6 ratio are not a family-wide interface.

### S3 frequency planning

[`lo_plan.h`](../esp32s3/src/lo_plan.h) is a standalone, **0BSD** C/C++ header.
It needs only `<stdint.h>` and `<stdbool.h>`. Copy it into another project to
calculate the SDM word and nominal receive frequency. It has no eSpDR
protocol, SDK, FPGA, USB or host dependency.

```c
#include "lo_plan.h"

struct esp32s3_lo_plan plan;
bool ok = esp32s3_plan_lo(2000000000u, ESP32S3_LO_AUTO, &plan);
/* ok=true, sdm_word=0x300000, pll_hz=2400000000, lo_hz=2000000000,
 * mode=ESP32S3_LO_5_6. The function does not access hardware. */
```

Explicit `ESP32S3_LO_NORMAL` and `ESP32S3_LO_5_6` modes allow comparisons in
the overlap. With the current PLL limits, normal accepts 2210–2790 MHz and
5/6 accepts 1841.666667–2325 MHz. eSpDR's automatic policy selects normal in
the overlap to preserve its existing tuning behavior.

For a nominal 40 MHz crystal and 24-bit SDM word `W`:

```text
normal PLL coordinate = 30 MHz × (32 + W / 65536)
5/6 receive LO        = 25 MHz × (32 + W / 65536)

W = round(requested_LO_Hz × 65536 / K) − 32 × 65536
K = 30,000,000 for normal; 25,000,000 for 5/6
```

The nominal steps are **457.763672 Hz** and **381.469727 Hz**, respectively.
Use 64-bit intermediates. The helper rounds directly in the selected mode,
avoiding an intermediate integer-Hz approximation to the PLL coordinate.

### Register sequence

Run vendor PHY power-up calibration first and stop capture before changing
the radio. Serialize access with any other radio user. The following sequence
matches the tested setup and the production
[`tune_pll()` / `reconfigure()` implementation](../esp32s3/src/radio.c):

1. Park the receive path. Clear CKGEN `0x65:0[4]` for normal conversion,
   preserving the other seven bits.
2. Take RFPLL software ownership at `0x6000e0c4[25]`. Clear manual capacitor
   mode at RFPLL `0x62:11[6]`.
3. Write `0x07` to SDM `0x63:0`, then `W`'s high/middle/low bytes to
   `0x63:3`, `:4`, `:5`. Write `0x17` to `0x63:0`.
4. Restart calibration by setting RFPLL `0x62:0[6]`, clearing then setting
   `[5]`, then clearing `[6]`. Poll `0x62:7[1]`, at 20 µs intervals for at
   most 100 polls, then wait 5 µs.
5. Enable manual capacitor mode. Scan all 512 codes using `0x62:1[7:0]`
   and `0x62:2[4]` for the ninth bit. After 20 µs at each code, accept
   `0x62:12[3:2] == 0`. Hold the midpoint of the longest consecutive window.
   Fail if calibration times out or no accepted window exists.
6. Configure the receive path, gain and filters. Apply the final CKGEN
   bit **after** this setup; set it for 5/6, clear it for normal. Wait 3 ms
   and check the bit readback before capture.

The final write using the ESP32-S3 ROM helpers is:

```c
uint8_t old = esp_rom_regi2c_read(0x65, 1, 0);
uint8_t bit = plan.mode == ESP32S3_LO_5_6 ? 0x10 : 0;
esp_rom_regi2c_write(0x65, 1, 0, (old & 0xef) | bit);
```

`1` is the ROM helper's analog-host argument. On the measured baseline,
`old` is `0x63`. Preserve unrelated bits; the other CKGEN encodings have not
been qualified as interchangeable. Restore both PLL programming and CKGEN
mode on failure. Merely changing CKGEN without scaling the frequency word
changes the actual receive center while leaving an old software label behind.

This is an observed conversion ratio. The measurements do not identify the
internal mixer/divider topology or establish the same behavior on other ESP
chips. `I + jQ` retains the **LO minus RF** convention.

## Applying the method to another ESP variant

Start with that chip's working receive setup and PHY library. Identify the
frequency-word writer, its reference-clock arithmetic, and the clock/band
selection helpers that run during channel changes. Look for state changes
that alter the relationship between the programmed word and the requested
RF frequency. A dual-band chip's band-switch sequence is a useful place to
compare those relationships.

1. **Establish the ordinary coordinate.** Recover the word width, fractional
   bits, bias and reference multiplier for that chip. Check several ordinary
   tunes against a known RF source. Record its stable PLL lock interval.
2. **Find a candidate conversion selector.** Follow the clock/band-selection
   writes and compare complete before/after states. Keep the PLL word and
   capacitor initially fixed so a changed response can be attributed to the
   selector. Use the chip's own calibration/receive sequence.
3. **Measure the new receive center.** A low-IF test tone gives
   `F_effective = F_RF + F_IF` for an LO-minus-RF complex-IQ convention.
   Determine the IQ sign and sample cadence on the target chip first.
4. **Change both frequencies independently.** Step the source while holding
   the PLL fixed, then step the ordinary PLL coordinate while holding the
   source fixed. Fit effective LO against PLL coordinate as `aF + b`, without
   assuming a ratio or forcing a zero intercept. Compare slopes with the
   ordinary mode; their ratio largely cancels the common reference error.
5. **Check the interpretation.** Use source on/off, a level change, and a
   second source hardware-LO/DSP split at the same wanted RF. A wanted signal
   follows the source step one-for-one. Source harmonics/spurs and changed
   ADC clocking can otherwise look like a new LO mode.
6. **Integrate actual receive-frequency tuning.** Invert the measured
   relationship, use the target's SDM/calibration sequence, and reapply the
   selector after any setup that overwrites it. Restore both PLL and selector
   after failure. Test the RF input/matching path and lock margins at the
   new band edges before calling those edges usable reception.

The deliverable on another chip is its own conversion formula and ordered
register sequence. A successful S3 bit write is a worked example, not a
register map to copy to C3/C5/C6 or the original ESP32. This release establishes
the S3 result; those other variants have not been hardware-validated here.

## Quick validity check

Independent reconstruction of the existing conducted-tone measurements gives
slopes **1.0000400** (normal) and **0.8333670** (alternate), for a ratio of
**0.833333609**, consistent with 5/6. Five ordinary PLL coordinates—2230,
2320, 2560, 2650 and 2750 MHz—were checked in each mode, with fit residuals
below 2.03 kHz. Source-frequency steps, a level change and an alternate source
hardware LO supported wanted-signal reception.

Those measurements used nominal 80 Msps and a coax/pad connection to isolated
`LNA_IN`. Stock antenna/matching response, sensitivity, endpoint lock margin,
alternate-mode 16 Msps reception and protocol decoding remain separate
measurements. The approximate 1.84–2.79 GHz claim describes tuning reach;
“verified across the entire range” would overstate these tests.

The release candidate was also exercised through the live Chromium web UI:
80 Msps streaming at 1841.666667, 1900, 2000, 2130, 2210 and 2440 MHz;
conducted tones near the low endpoint, 2.0 GHz and 2.44 GHz; filter changes
while retaining 5/6 mode; and stop/restart from the page. The reported receive
streams had zero gaps, lost pairs or transport errors. The rejected 2790 MHz
tune restored the previous 5/6 setting and resumed the spectrum. These are
short functional checks; the original bench firmware and requested settings were restored.

## Bands this puts within tuning reach

The lower mode adds access to these example RF allocations. These are
frequency overlaps for raw I/Q reception; protocol decoders and reception
performance for each system are separate work.

| System | Frequencies within the tuning envelope | Newly reached below 2210 MHz |
|---|---|---|
| DECT | Europe 1880–1900 MHz; US 1920–1930 MHz | Both |
| PCS1900, LTE bands 2/25 | Uplink 1850–1910/1915; downlink 1930–1990/1995 MHz | Both directions |
| LTE band 1 | Uplink 1920–1980; downlink 2110–2170 MHz | Both directions |
| AWS LTE bands 4/66 | Downlink 2110–2155/2200 MHz | Downlink; their 1.7 GHz uplink is outside the envelope |
| LTE TDD bands 34/39 | 2010–2025 / 1880–1920 MHz | Both |
| LTE 7/38/40/41 | 2.3/2.5/2.6 GHz allocations | Already within normal-mode reach |
| 2.4 GHz Wi-Fi, Bluetooth/BLE, Zigbee; 13 cm amateur | Existing 2.3/2.4 GHz coverage | Already within normal-mode reach |

Allocation references: [ETSI DECT](https://www.etsi.org/technologies/dect?highlight=WyJ0cyIsInRzJ3MiXQ%3D%3D),
[FCC DECT/UPCS](https://docs.fcc.gov/public/attachments/FCC-10-77A1.pdf),
[3GPP TS 36.101 table 5.5-1 via ETSI](https://www.etsi.org/deliver/etsi_ts/136100_136199/136101/13.08.00_60/ts_136101v130800p.pdf),
[Bluetooth SIG](https://www.bluetooth.com/learn-about-bluetooth/key-attributes/range/),
[Zigbee Alliance/CSA](https://csa-iot.org/all-solutions/zigbee/zigbee-faq/),
and [ARRL 13 cm band plan](https://www.arrl.org/band-plan).
