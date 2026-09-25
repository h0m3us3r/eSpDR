# Inside the eSpDR receiver

The ESP32-S3 already has most of the pieces of a small SDR: an RF receive
chain, a tunable local oscillator, gain and filter controls, and a hardware
path that writes complex samples into SRAM. eSpDR takes control of those
pieces and streams the samples to the host.

The radio work started in the parent `ble` project, with short captures and
a USRP providing test signals and simultaneous reference recordings. Those
experiments established the sample format, tuning sequence and receiver
controls used here. This guide brings that work together with the current
firmware. Register names such as `DUMP_CONFIG_REG` are names used by this
project for the S3's undocumented radio registers.

## Getting the I/Q samples

The samples come from the radio receive path after RF downconversion. The
hardware supplies I and Q together, as two signed 10-bit values in each
32-bit word:

| Bits | Contents used by eSpDR |
|---|---|
| 0–9 | I, two's complement, −512 to 511 |
| 10–19 | Q, two's complement, −512 to 511 |
| 20–31 | Sideband bits, discarded when packing the stream |

The key to exposing that word was the dump engine's byte selector at
`0x60033d90`. It has four six-bit selectors. Selecting input bytes 0, 1, 2
and 3 gives the complete word:

```c
uint32_t map = 0 | (1u << 6) | (2u << 12) | (3u << 18);
/* map = 0x000c2040 */
```

During the original investigation this register was left at zero, which
selected byte zero four times and produced words such as `0xabababab`.
Following the vendor's MAC initialization revealed the missing write.
Controlled tones then showed both components in quadrature, and moving the
LO across the source reversed the complex frequency sign.

The firmware writes that map as `DUMP_CONFIG_IQ`. To unpack a captured word:

```c
int i = (int)((word & 1023u) ^ 512u) - 512;
int q = (int)(((word >> 10) & 1023u) ^ 512u) - 512;
```

With these definitions, `I + jQ` has the frequency convention **LO minus RF**.
A tone 2 MHz above the LO appears at −2 MHz in the complex samples. The
browser spectrum reverses the FFT-bin order so its RF frequency axis runs
left to right as expected. Exported I/Q keeps the ESP's original convention;
conjugating it changes the sign convention for downstream DSP.

The dump engine runs continuously into internal SRAM. These are radio
samples: reception does not depend on recognizing a Wi-Fi or Bluetooth packet.

The small set of registers that controls extraction is collected in
[`board.h`](../esp32s3/src/board.h):

| Address | Use in this firmware |
|---|---|
| `0x60033d90` | Select the four bytes of the I/Q word |
| `0x60033d5c` | Start/stop, ring configuration and 80/16 Msps selection |
| `0x60033d60` | Read the current write index |
| `0x600c101c[3:0]` | Select one of the four destination SRAM banks |

The stopped control word is `0x00024000` at 80 Msps or `0x00034000` at
16 Msps. Setting bit 31 starts the writer. The low four bank-select bits use
one bit per bank, so the running capture rotates through `1`, `2`, `4`, `8`.

## Bringing up the receiver

Power-up calibration comes from the vendor PHY library. The firmware powers
the modem, enables its clocks and calls `register_chipv7_phy` with full RF
calibration and an empty calibration record. It then takes over tuning and
receive configuration directly in [`radio.c`](../esp32s3/src/radio.c).

Several clocks have to stay enabled for this to work. In particular, the
sample writer needs the Wi-Fi MAC clock, including bit 6 of
`SYSTEM_WIFI_CLK_EN_REG`. The PHY and RNG clocks also stay on because the
dump registers depend on them. The Bluetooth clock is disabled after
calibration.

The receive setup follows a deliberate order:

1. Stop the dump engine and park the receive path.
2. Program the 20 or 40 MHz width while the baseband is disabled, then give
   it an enable edge. This activates the selected analog filter bank.
3. Disable Wi-Fi AGC, force the requested gain-table index and program the
   RC filter pair. Allow 100 µs for the selected gain to settle.
4. Read the RF and baseband gain words chosen by the hardware. Apply any
   explicit stage overrides, then take software ownership through PBUS.
5. Hold the receive stages on, keep both transmit groups off, and disable
   the baseband while retaining the analog configuration.
6. Apply DC and I/Q correction settings, select the I/Q dump format and
   prepare the writer for capture.

PBUS is the internal interface used here to control the analog stages.
The firmware saves the state it takes over so that a later setting change
can release it and repeat the same setup sequence.

## Keeping acquisition continuous

At 80 million pairs per second, the hardware's four-byte words arrive at
320 MB/s. A 64 KiB bank fills in 204.8 µs. The firmware therefore switches
banks while the writer is running and sends completed data in parallel.

Four capture banks start at `0x3fcb0000`, `0x3fcc0000`, `0x3fcd0000` and
`0x3fce0000`. Core 0 handles banks 0 and 2; core 1 handles banks 1 and 3.
The write index continues around a 16,384-pair ring as the destination bank
changes. Each core switches at a threshold of 15,360 pairs, leaving time
before the writer wraps around.

The switch and the final sample arriving in SRAM are separated by a short
pipeline delay. To find the exact boundaries, the firmware plants sentinel
words around the predicted start and end positions before handing over the
next bank. After switching, it lets pending writes settle and finds where
samples have replaced the sentinels. That gives the actual first pair and
pair count for each unit. The next unit begins at the previous unit's end.

The CPU packs the low 20 bits of each word for the GPIO link. The FPGA
reorders the two lanes, removes alignment padding and compresses the samples.
The capture loop, boundary checks and bank handoffs are in
[`capture.c`](../esp32s3/src/capture.c); the wire format is in
[`link.h`](../protocol/link.h).

Capture bank 3 also overlaps working memory used by the ROM's analog-register
helpers. The firmware saves that memory before capture and restores it
afterward. This is one reason receiver settings are applied between runs.

## Tuning the PLL

Tuning has two parts: program a frequency word, then choose a VCO capacitor
setting that works at that frequency. The word sets the PLL's frequency
relationship to the reference. The capacitor setting puts the VCO into the
right operating range.

### The frequency word

For the 40 MHz reference used by eSpDR, the relation implemented by the
firmware is:

```text
LO = (3/4 × reference) × (32 + W / 65536)
   = 960 MHz + 30 MHz × W / 65536
```

`W` is a 24-bit sigma-delta word. One increment corresponds to
**457.763671875 Hz** at the nominal reference frequency. The firmware rounds
the requested LO to the nearest word and reports the resulting frequency
in integer Hz. For example, requesting 2440 MHz gives `W = 0x315555` and a
reported LO of 2,439,999,847 Hz. The physical frequency follows the actual
reference clock.

The earlier control sweep measured about **457.69 Hz per word**, using four
passes over 33 consecutive words. That agrees with the nominal step above.

### Programming and capacitor selection

The analog registers are accessed through the ROM's `regi2c` helpers, using
RFPLL block `0x62` and sigma-delta block `0x63`. In the notation below,
`0x62:11[6]` means bit 6 of register 11 in analog block `0x62`.

The current `tune_pll()` sequence is:

1. Set `0x6000e0c4[25]` to take software control of RFPLL tuning and clear
   manual capacitor mode at `0x62:11[6]`.
2. Write `0x07` to `0x63:0`, write the word's high, middle and low bytes to
   `0x63:3`, `:4` and `:5`, then write `0x17` to `0x63:0`.
3. Restart calibration through bits 6 and 5 of `0x62:0`. Poll the completion
   bit, `0x62:7[1]`, up to 100 times with a 20 µs delay between polls.
4. After completion and a further 5 µs delay, enable manual capacitor mode
   and scan all 512 codes. The low eight bits go to `0x62:1`; the ninth goes
   to `0x62:2[4]`.
5. Wait 20 µs at each code and read `0x62:12[3:2]`. A value of zero is the
   firmware's acceptance criterion. Find the longest consecutive run of
   accepted codes and select `first + (length - 1) / 2`.
6. Hold that capacitor code in manual mode for reception.

Choosing the middle gives room on either side of the selected code. A full
scan also handles the shape of the capacitor map without relying on a
frequency-to-capacitor lookup table. The parent project's map swept all 512
codes: the main curve progressed through code 383, while codes 384–511
repeated the final 32-code section.

`iqstream status` reports the result as `pll=cap[first+length]`. For example,
`pll=224[220+10]` would mean code 224 was selected from codes 220 through 229.
Those numbers describe the capacitor scan's status window.

The current LO control accepts **2210–2790 MHz**. Earlier capacitor-status
sweeps found an interval around 2207–2795 MHz on the development board. In
the later screenshot session, the highest accepted LO was 2781 MHz, with
2782–2790 MHz returning an error. These are the observations behind the
roughly 2.2–2.8 GHz tuning range described in the README.

If calibration times out or the scan finds no accepted code, the setting
change fails. The firmware restores the previous requested settings and
retunes the receiver to them; receiver status records whether restoration
succeeded.

## Sample rate, width and filter

These three controls answer different questions:

| Control | What it changes | Choices |
|---|---|---|
| `rate` | How many complex pairs the dump engine writes each second | 80 or 16 Msps |
| `width` | Which analog filter bank is activated | 40 or 20 MHz mode |
| `filter` | The two RC codes in that bank | 0–63 for each register |

At 80 Msps, the sampled frequency span is LO ±40 MHz. At 16 Msps it is
LO ±8 MHz. The dump-control register, `0x60033d5c`, selects those rates with
bit 16: clear for 80 Msps, set for 16 Msps. Earlier measurements checked both
the SRAM fill-time slope and the frequency of a known RF tone. A pair arrived
every three CPU cycles at 80 Msps and every fifteen at 16 Msps, with the CPU
running at 240 MHz.

Reducing the rate reduces the packed payload from 200 MB/s to 40 MB/s.
It also makes each capture bank fill five times more slowly. The capture loop
adjusts its timing guards and bank-switch settling delay accordingly.

### Why width has to be set before enabling the baseband

The width fields select an analog bank on the baseband enable edge. During
development, writing them afterward left the active filter narrow even
though register readback showed the requested wide setting. Programming
width first and then pulsing the baseband enable fixed the first capture.

The firmware writes `0x60006100[21:16]` and `0x6002600c[3:2]`, then disables
the baseband for 1 µs before enabling it. Once latched, the filter selection
survives the later PBUS takeover and baseband disable.

### RC filter codes

The active pair lives in analog block `0x67`:

| Width mode | First filter register | Second filter register |
|---|---|---|
| 20 MHz | 4 | 5 |
| 40 MHz | 6 | 7 |

`filter=12` writes 12 to both registers. `filter=12,20` sets them separately.
Both registers affect both I and Q. Higher codes narrow the response; zero
is the widest exposed setting. The numbers are raw RC codes, so a request
such as `filter=20` means code 20.

The width labels identify the selected bank; the RC overrides then determine
its response. eSpDR boots with `width=40` and `filter=0` to use that bank's
widest exposed setting.

The development experiments swept a fixed RF tone across the receive
passband by moving the ESP's LO. All 64 codes in each of the four registers
were tested, including independent changes to each register. Those tests
established which pair was active and how the controls changed the response.
A separate simultaneous test captured tones at 2413, 2420.25 and 2427 MHz
together, spanning 14 MHz.

The displayed spectrum uses the received samples with FFT windowing, optional
DC removal, and averaging or peak detection. FFT normalization applies the
same scale factor to every bin. The browser then handles smoothing, peak
hold, and the display's level and colour scales. There is no frequency-dependent
passband correction in the host or frontend.

Signals beyond the sampled span's edges can fold back into it: an offset of
±46 MHz was observed aliasing into
the 80 Msps capture. Rate and filtering can be set independently, so choose
the filter with the sampled span and surrounding signals in mind.

## Gain and manual stage control

The normal `gain=0..127` control selects a vendor gain-table entry. The
firmware forces that entry, reads the resulting RF and baseband stage words,
and holds them through PBUS. Gain stays fixed during capture.

The measured table is roughly 1 dB per step over indices 35–76. Elsewhere
it has discontinuities: 29→30 produced a drop of about 4 dB, and indices
83–127 return to lower or repeated settings. The full 128-index sweep found
107 distinct pairs of stage words.

For direct control, `rf=0..511` writes the RF word at PBUS `(1,2)`.
`bb=0..127` supplies the low seven bits of the baseband word at PBUS `(0,1)`;
the firmware preserves its receive-enable bits by writing `0x180 | bb`.
The raw words combine fields with interacting effects, so their numerical
order is best treated as a register setting.

Each explicit override remains active across gain-table changes. Use
`rf=auto bb=auto` to return both stages to the words selected by `gain`.
Here, `auto` means “use the table's value”; the receive AGC remains disabled.

## DC offset and I/Q balance

A DC offset consumes sample headroom and appears at the centre of the
spectrum. Four nine-bit PBUS controls adjust it:

| Setting | PBUS block, index | Main component | Measured change per code |
|---|---|---|---:|
| `dc0` | 3, 1 | I | about 1.20 ADC counts |
| `dc1` | 3, 2 | I | about 0.79 ADC counts |
| `dc2` | 2, 1 | Q | about 1.25 ADC counts |
| `dc3` | 2, 2 | Q | about 0.81 ADC counts |

These slopes came from the development board at gain selector 48. Each
field was swept through all 512 values, with repeated adjacent-code checks
across the 127/128, 255/256 and 383/384 boundaries. `dcN=auto` uses the
hardware-selected value captured during receiver setup.

I/Q imbalance produces a mirror of a signal on the opposite side of the LO.
`iq=A,P` adjusts the receive correction coefficients: amplitude is a signed
five-bit value (−16..15), and phase is a signed six-bit value (−32..31).
The measured effects were consistent with roughly 1/64 amplitude-coefficient
steps and 1/128 I-to-Q correction steps. Near balance, the latter was about
0.46° per code in the tested configuration.

The activation bits matter here too. The coefficients take effect with
`0x6000607c[27]` set and bit 28 clear. The firmware sets that mode and writes
amplitude to bits 20:16 and phase to bits 26:21, preserving the other fields.
`iq=auto` restores the correction register saved during receiver setup.

The browser's FFT DC removal is a display-processing option. The `dc0`–`dc3`
and `iq` controls above change the receiver samples themselves.

## Trying the controls

These commands use the host tool after loading both devices. Stop
`iqstream serve` before using the command line to take ownership of them.

```sh
# See the effective settings and the PLL capacitor window.
iqstream status

# Wide capture with table-selected gain.
iqstream set lo=2440M rate=80 width=40 filter=0 gain=24 rf=auto bb=auto
iqstream capture --seconds 5 --output wide.iqc

# Select the slower sample rate and a narrower RC setting.
iqstream set rate=16 width=20 filter=30
iqstream capture --seconds 5 --output narrow.iqc

# Return DC and I/Q correction to the hardware-selected values.
iqstream set dc0=auto dc1=auto dc2=auto dc3=auto iq=auto
```

Each setting change repeats the receive configuration between captures.
The PLL is retuned when the LO changes or after a PLL failure. Settings last
until the ESP firmware is reloaded. In the browser, changing a receiver
setting ends and verifies the current run before starting the next one.

## Finding the implementation

| File | What to look for |
|---|---|
| [`radio.c`](../esp32s3/src/radio.c) | PHY calibration, `tune_pll`, width/filter setup, PBUS ownership and setting restoration |
| [`board.h`](../esp32s3/src/board.h) | Register addresses, analog block IDs and dump-control constants |
| [`capture.c`](../esp32s3/src/capture.c) | Writer startup, bank switching, sentinel searches and ROM-state preservation |
| [`memory.ld`](../esp32s3/memory.ld) | SRAM placement for code, data and capture banks |
| [`receiver.cpp`](../host/src/receiver.cpp) | Command-line values, signed coefficient encoding and status formatting |
| [`spectrum.cpp`](../host/src/spectrum.cpp) | Spectrum processing and RF-frequency orientation |

The development measurements summarized here were recorded on September 19,
2026. The receive-control campaign included 6,586 validated short captures
and 554 simultaneous USRP source-witness bursts. The figures above describe
those measured configurations; the source links describe the implementation
shipped with eSpDR.
