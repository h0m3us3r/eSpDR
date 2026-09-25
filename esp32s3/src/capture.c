/*
 * Continuous acquisition on both CPU cores.
 *
 * The ADC dump engine writes pairs into whichever of the four capture banks
 * is selected, at a ring index that advances continuously across banks. Core
 * 0 services banks 0 and 2, core 1 banks 1 and 3. For each unit a core:
 *
 *   1. prepares the next bank: sentinel words where that bank's unit is
 *      predicted to start and end, so both edges can be found exactly later;
 *   2. polls until its bank holds LINK_THRESHOLD_PAIRS new pairs;
 *   3. switches the writer to the next bank (handing it to the other core);
 *   4. locates the exact first and last pair written, publishes the end as
 *      the next unit's start, and transmits the unit on its lane.
 *
 * Steady state needs no locking: the stagger alone keeps the cores apart.
 * The ownership, timing and sentinel checks only detect a broken schedule;
 * any failure stops the writer and ends the run on both cores.
 *
 * Each core runs its own copy of the loop from its own instruction SRAM bank
 * and transmits only from its own capture banks, so neither core can stall
 * the other during the cycle-exact transmit kernel.
 */
#include "capture.h"

#include <stdbool.h>

#include "control.h"
#include "hal/dedic_gpio_cpu_ll.h"
#include "link.h"
#include "platform.h"
#include "radio.h"
#include "soc/systimer_reg.h"
#include "soc/usb_serial_jtag_reg.h"

#define RING_MASK (LINK_RING_PAIRS - 1u)
#define SENTINEL 0xA5C33C5Au
#define START_GUARD 512u   /* sentinels ahead of a predicted unit start */
#define END_GUARD 1024u    /* sentinels ahead of a predicted unit end */
#define NOT_FOUND 0xFFFFFFFFu
#define TICKS_PER_SECOND 16000000u
/* A bank must be left before the writer could wrap around onto its own
 * unit: within this many pairs, 5 per 16 MHz timer tick at 80 Msps. */
#define MAX_BANK_AGE_PAIRS (LINK_RING_PAIRS - 128u)
/* Samples still in the ADC pipeline land shortly after a bank switch: within
 * 48 CPU cycles at 80 Msps, and five times that at 16 Msps. */
#define SWITCH_SETTLE_CYCLES_PER_PAIR_TICK 240u

#define ALWAYS_INLINE static inline __attribute__((always_inline))

/* Transmit descriptor; its layout is fixed by transmit.S. */
typedef struct {
    uint32_t reserved[2]; /* zero: XOR masks for checksum seed and end marker */
    uint32_t sequence;
    uint32_t layout;
    struct {
        uint32_t address;
        uint32_t groups; /* 16-pair groups in this fragment */
    } fragment[4];        /* head, contiguous bulk, wrapped bulk, tail */
} tx_descriptor;

_Static_assert(sizeof(tx_descriptor) == 48, "transmit.S descriptor layout");

extern void transmit_lane0(tx_descriptor *tx);
extern void transmit_lane1(tx_descriptor *tx);

typedef struct {
    uint32_t units;
    uint64_t pairs;
    uint32_t service_max;
} lane_stats;

static struct {
    volatile uint32_t core1_ready, started, stopped, core1_done, stop_requested, stopped_by_host;
    /* Per-bank handoff state, published by the core that switched into it. */
    volatile uint32_t busy[CAPTURE_BANKS], valid[CAPTURE_BANKS], start[CAPTURE_BANKS];
    volatile uint32_t start_probe[CAPTURE_BANKS], end_probe[CAPTURE_BANKS];
    volatile uint32_t epoch[CAPTURE_BANKS];
    volatile uint32_t status, fail_lane, fail_detail;
    uint32_t t0, t_stop;
    uint64_t duration_ticks;
    uint32_t dump_control, max_bank_age, switch_settle; /* for the sample rate */
    lane_stats lane[2];
    tx_descriptor tx[2] __attribute__((aligned(16)));
} ring;

static volatile uint32_t core1_request;

static struct {
    uint32_t status, detail, fail_lane, stopped_by_host;
    lane_stats lane[2];
    uint64_t ticks;
} last_run;

ALWAYS_INLINE uint32_t *bank(unsigned b) { return (uint32_t *)(CAPTURE_BANK_BASE + b * CAPTURE_BANK_BYTES); }

/* Low 32 bits of the 16 MHz system timer, without a function call. */
ALWAYS_INLINE uint32_t tick(void)
{
    REG(SYSTIMER_UNIT0_OP_REG) |= SYSTIMER_TIMER_UNIT0_UPDATE;
    while (!(REG(SYSTIMER_UNIT0_OP_REG) & SYSTIMER_TIMER_UNIT0_VALUE_VALID)) {
    }
    return REG(SYSTIMER_UNIT0_VALUE_LO_REG);
}

ALWAYS_INLINE void wait_cycles(uint32_t cycles)
{
    uint32_t start = cpu_cycles();
    while (cpu_cycles() - start < cycles) {
    }
}

/* Fills ring indices [at, at + count) of a bank with sentinels, wrapping. */
ALWAYS_INLINE void fill_sentinels(unsigned b, unsigned at, unsigned count)
{
    while (count) {
        unsigned n = LINK_RING_PAIRS - at;
        if (n > count)
            n = count;
        uint32_t *p = bank(b) + at;
        unsigned head = (-at) & 3;
        if (head > n)
            head = n;
        for (unsigned i = 0; i < head; i++)
            *p++ = SENTINEL;
        unsigned vectors = (n - head) / 4;
        if (vectors) {
            uint32_t value = SENTINEL;
            __asm__ volatile("ee.movi.32.q q6, %[v], 0\n"
                             "ee.movi.32.q q6, %[v], 1\n"
                             "ee.movi.32.q q6, %[v], 2\n"
                             "ee.movi.32.q q6, %[v], 3\n"
                             "loopnez %[n], 1f\n"
                             "ee.vst.128.ip q6, %[p], 16\n"
                             "1:"
                             : [p] "+a"(p)
                             : [v] "a"(value), [n] "a"(vectors)
                             : "memory");
        }
        for (unsigned i = head + vectors * 4; i < n; i++)
            *p++ = SENTINEL;
        count -= n;
        at = 0;
    }
    memory_barrier();
}

ALWAYS_INLINE void stop_writer(void)
{
    REG(DUMP_CTRL_REG) = ring.dump_control;
    memory_barrier();
    REG(DUMP_BANK_SELECT_REG) &= ~15u;
}

ALWAYS_INLINE void fail(unsigned code, unsigned lane, unsigned detail)
{
    if (!ring.status) {
        ring.fail_lane = lane;
        ring.fail_detail = detail;
        ring.status = code;
    }
    stop_writer();
    memory_barrier();
    ring.t_stop = tick();
    ring.stopped = 1;
}

/* Offset of the first written pair after a start probe; START_GUARD if none. */
ALWAYS_INLINE unsigned find_first_written(unsigned b, unsigned probe)
{
    const uint32_t *p = bank(b);
    if (p[(probe + START_GUARD - 1) & RING_MASK] == SENTINEL)
        return START_GUARD;
    unsigned lo = 0, hi = START_GUARD - 1;
    while (lo < hi) {
        unsigned mid = (lo + hi) / 2;
        if (p[(probe + mid) & RING_MASK] == SENTINEL)
            lo = mid + 1;
        else
            hi = mid;
    }
    return lo;
}

/* Ring index one past the last written pair, searching the end probe region
 * from the write index observed before the switch. */
ALWAYS_INLINE unsigned find_end(unsigned b, unsigned write_index, unsigned guard)
{
    const uint32_t *p = bank(b);
    unsigned origin = ring.end_probe[b];
    unsigned offset = (write_index - origin) & RING_MASK;
    if (offset >= guard || p[(origin + guard - 1) & RING_MASK] != SENTINEL)
        return NOT_FOUND;
    unsigned lo = offset, hi = guard - 1;
    while (lo < hi) {
        unsigned mid = (lo + hi) / 2;
        if (p[(origin + mid) & RING_MASK] == SENTINEL)
            hi = mid;
        else
            lo = mid + 1;
    }
    return (origin + lo) & RING_MASK;
}

/*
 * Builds the transmit descriptor for pairs [first, first + count) of a bank.
 * The kernel sends whole 16-pair groups from 16-pair-aligned addresses, so a
 * partial head group is moved down to the aligned slot below it, and both
 * partial groups are zero-padded in place. The bank belongs to this core
 * until transmission ends, and the free arc of at least 96 pairs keeps the
 * head and tail groups apart.
 */
ALWAYS_INLINE void prepare_descriptor(unsigned lane, unsigned b, unsigned first, unsigned count,
                                      unsigned sequence)
{
    tx_descriptor *tx = &ring.tx[lane];
    uint32_t *p = bank(b);
    unsigned head = (-first) & 15;
    if (head > count)
        head = count;
    unsigned tail = (count - head) & 15;
    unsigned bulk = count - head - tail;
    unsigned head_at = first & ~15u;
    unsigned tail_at = (first + count - tail) & RING_MASK;

    if (head) {
        for (unsigned i = 0; i < head; i++)
            p[head_at + i] = p[first + i];
        for (unsigned i = head; i < 16; i++)
            p[head_at + i] = 0;
    }
    if (tail)
        for (unsigned i = tail; i < 16; i++)
            p[tail_at + i] = 0;

    unsigned bulk_at = (first + head) & RING_MASK;
    unsigned before_wrap = LINK_RING_PAIRS - bulk_at;
    if (before_wrap > bulk)
        before_wrap = bulk;

    tx->reserved[0] = 0;
    tx->reserved[1] = 0;
    tx->sequence = sequence;
    tx->layout = first | (count << 14) | (b << 28) | ((uint32_t)LINK_VERSION << 30);
    tx->fragment[0].address = (uint32_t)(p + head_at);
    tx->fragment[0].groups = head ? 1 : 0;
    tx->fragment[1].address = (uint32_t)(p + bulk_at);
    tx->fragment[1].groups = before_wrap / 16;
    tx->fragment[2].address = (uint32_t)p;
    tx->fragment[2].groups = (bulk - before_wrap) / 16;
    tx->fragment[3].address = (uint32_t)(p + tail_at);
    tx->fragment[3].groups = tail ? 1 : 0;
    memory_barrier();
}

ALWAYS_INLINE void lane_loop(const unsigned lane, void (*const transmit)(tx_descriptor *))
{
    lane_stats *stats = &ring.lane[lane];
    uint64_t elapsed = 0;
    uint32_t previous_tick = ring.t0;
    unsigned b = lane;

    for (unsigned unit = 0; !ring.stopped; unit++, b ^= 2) {
        unsigned sequence = unit * 2 + lane;
        unsigned next = (b + 1) & 3;

        /* Wait for the other lane to move the writer into this bank and
         * publish its start. At 80 Msps that has always happened by now; at
         * 16 Msps a bank fills five times more slowly than a unit transmits. */
        while (!ring.valid[b] && !ring.stopped) {
        }
        if (ring.stopped)
            break;
        ring.valid[b] = 0;

        if ((REG(DUMP_BANK_SELECT_REG) & 15u) != (1u << b)) {
            fail(ESP_FAIL_OWNER, lane, REG(DUMP_BANK_SELECT_REG));
            break;
        }

        /* 1. Prepare the next bank before its predicted start. */
        uint32_t prepare_start = cpu_cycles();
        if (ring.busy[next]) {
            fail(ESP_FAIL_REUSE, lane, next);
            break;
        }
        unsigned probe = (ring.start[b] + LINK_THRESHOLD_PAIRS) & RING_MASK;
        unsigned end_probe = (probe + LINK_THRESHOLD_PAIRS) & RING_MASK;
        fill_sentinels(next, probe, START_GUARD);
        fill_sentinels(next, end_probe, END_GUARD);
        ring.start_probe[next] = probe;
        ring.end_probe[next] = end_probe;
        if (lane == 0 &&
            (REG(USB_SERIAL_JTAG_EP1_CONF_REG) & USB_SERIAL_JTAG_SERIAL_OUT_EP_DATA_AVAIL))
            ring.stop_requested = 1;
        uint32_t prepare_cycles = cpu_cycles() - prepare_start;

        unsigned written = (REG(DUMP_WRITE_INDEX_REG) - ring.start[b]) & RING_MASK;
        if (written >= LINK_THRESHOLD_PAIRS) {
            fail(ESP_FAIL_LATE_PREPARE, lane, written);
            break;
        }

        /* 2. Wait for the threshold. */
        unsigned write_index;
        uint32_t age;
        do {
            write_index = REG(DUMP_WRITE_INDEX_REG) & RING_MASK;
            age = tick() - ring.epoch[b];
            if (ring.stopped)
                break;
            if (age > ring.max_bank_age) {
                fail(ESP_FAIL_LATE_POLL, lane, age);
                break;
            }
        } while (((write_index - ring.start[b]) & RING_MASK) < LINK_THRESHOLD_PAIRS);
        if (ring.stopped)
            break;

        /* 3. Switch the writer. Lane 1 ends the run, so both lanes always
         *    carry the same number of units. */
        uint32_t service_start = cpu_cycles();
        uint32_t switched = tick();
        age = switched - ring.epoch[b];
        bool last = false;
        if (lane == 1) {
            elapsed += (uint32_t)(switched - previous_tick);
            previous_tick = switched;
            bool expired = ring.duration_ticks && elapsed >= ring.duration_ticks;
            last = expired || ring.stop_requested;
            if (last && !expired)
                ring.stopped_by_host = 1;
        }
        if (age > ring.max_bank_age) {
            fail(ESP_FAIL_LATE_SWITCH, lane, age);
            break;
        }
        ring.busy[b] = 1;
        if (last) {
            stop_writer();
            ring.t_stop = switched;
            ring.stopped = 1;
        } else {
            REG(DUMP_BANK_SELECT_REG) = (REG(DUMP_BANK_SELECT_REG) & ~15u) | (1u << next);
            memory_barrier();
            ring.epoch[next] = switched;
        }

        /* 4. Locate the unit exactly and publish its end. */
        wait_cycles(ring.switch_settle);
        unsigned end = find_end(b, write_index,
                                sequence ? END_GUARD : LINK_RING_PAIRS - LINK_THRESHOLD_PAIRS);
        if (end == NOT_FOUND) {
            fail(ESP_FAIL_END, lane, write_index);
            break;
        }
        /* The writer starts at ring index 0; bank 0 was filled with sentinels. */
        unsigned probe_at = sequence ? ring.start_probe[b] : 0;
        unsigned skip = find_first_written(b, probe_at);
        unsigned first = (probe_at + skip) & RING_MASK;
        if (skip >= START_GUARD || first != ring.start[b]) {
            fail(ESP_FAIL_START, lane, first | (ring.start[b] << 16));
            break;
        }
        unsigned count = (end - first) & RING_MASK;
        if (count < LINK_MIN_PAIRS || count > LINK_MAX_PAIRS) {
            fail(ESP_FAIL_LENGTH, lane, count);
            break;
        }
        if (!last) {
            ring.start[next] = end;
            memory_barrier();
            ring.valid[next] = 1;
        }

        prepare_descriptor(lane, b, first, count, sequence);
        transmit(&ring.tx[lane]);
        ring.busy[b] = 0;
        memory_barrier();

        stats->units++;
        stats->pairs += count;
        uint32_t service = cpu_cycles() - service_start + prepare_cycles;
        if (service > stats->service_max)
            stats->service_max = service;
    }
    dedic_gpio_cpu_ll_write_all(0);
}

static void __attribute__((noinline, aligned(16))) lane_loop_core0(void)
{
    lane_loop(0, transmit_lane0);
}

static void CORE1_CODE __attribute__((aligned(16))) lane_loop_core1(void)
{
    lane_loop(1, transmit_lane1);
}

/* Core 1 idles here, in its own code bank, and joins each run. */
void CORE1_CODE __attribute__((noreturn)) core1_main(void)
{
    dedic_gpio_cpu_ll_write_all(0);
    memory_barrier();
    core1_alive = 1;
    uint32_t seen = core1_request;
    for (;;) {
        if (core1_request == seen)
            continue;
        seen = core1_request;
        ring.core1_ready = 1;
        memory_barrier();
        while (!ring.started) {
        }
        lane_loop_core1();
        memory_barrier();
        ring.core1_done = 1;
    }
}

/* The ROM's working memory, kept here while a run overwrites it. */
static uint32_t rom_data[ROM_DATA_BYTES / 4];

static void copy_words(volatile uint32_t *to, const volatile uint32_t *from, unsigned words)
{
    while (words--)
        *to++ = *from++;
}

unsigned capture_run(unsigned seconds)
{
    uint32_t *state = (uint32_t *)(void *)&ring;
    for (unsigned i = 0; i < sizeof(ring) / 4; i++)
        state[i] = 0;
    ring.duration_ticks = (uint64_t)seconds * TICKS_PER_SECOND;
    unsigned pairs_per_tick = radio_pairs_per_tick();
    ring.dump_control = radio_dump_control();
    ring.max_bank_age = MAX_BANK_AGE_PAIRS / pairs_per_tick;
    ring.switch_settle = SWITCH_SETTLE_CYCLES_PER_PAIR_TICK / pairs_per_tick;

    copy_words(rom_data, (volatile uint32_t *)ROM_DATA_BASE, ROM_DATA_BYTES / 4);

    for (unsigned b = 0; b < CAPTURE_BANKS; b++)
        fill_sentinels(b, 0, LINK_RING_PAIRS);
    ring.start[0] = 0;
    ring.end_probe[0] = LINK_THRESHOLD_PAIRS;
    ring.valid[0] = 1;

    core1_request++;
    while (!ring.core1_ready) {
    }
    memory_barrier();

    REG(DUMP_CTRL_REG) = ring.dump_control;
    REG(DUMP_BANK_SELECT_REG) = (REG(DUMP_BANK_SELECT_REG) & ~15u) | 1u;
    memory_barrier();
    uint64_t started = timer_ticks();
    ring.t0 = (uint32_t)started;
    ring.epoch[0] = ring.t0;
    REG(DUMP_CTRL_REG) = ring.dump_control | DUMP_CTRL_RUN;
    memory_barrier();
    ring.started = 1;

    lane_loop_core0();
    while (!ring.core1_done) {
    }

    /* Extend the 32-bit stop tick with a fresh 64-bit reading. */
    uint64_t finished = timer_ticks();
    uint64_t stop = finished - (uint32_t)((uint32_t)finished - ring.t_stop);
    stop_writer();
    memory_barrier();
    copy_words((volatile uint32_t *)ROM_DATA_BASE, rom_data, ROM_DATA_BYTES / 4);
    memory_barrier();

    last_run.status = ring.status;
    last_run.detail = ring.fail_detail;
    last_run.fail_lane = ring.fail_lane;
    last_run.stopped_by_host = ring.stopped_by_host;
    last_run.lane[0] = ring.lane[0];
    last_run.lane[1] = ring.lane[1];
    last_run.ticks = stop - started;
    return ring.status;
}

uint32_t capture_stat(unsigned index)
{
    switch (index) {
    case ESP_STAT_STATUS: return last_run.status;
    case ESP_STAT_DETAIL: return last_run.detail;
    case ESP_STAT_FAIL_LANE: return last_run.fail_lane;
    case ESP_STAT_UNITS0: return last_run.lane[0].units;
    case ESP_STAT_UNITS1: return last_run.lane[1].units;
    case ESP_STAT_PAIRS0_LO: return (uint32_t)last_run.lane[0].pairs;
    case ESP_STAT_PAIRS0_HI: return (uint32_t)(last_run.lane[0].pairs >> 32);
    case ESP_STAT_PAIRS1_LO: return (uint32_t)last_run.lane[1].pairs;
    case ESP_STAT_PAIRS1_HI: return (uint32_t)(last_run.lane[1].pairs >> 32);
    case ESP_STAT_TICKS_LO: return (uint32_t)last_run.ticks;
    case ESP_STAT_TICKS_HI: return (uint32_t)(last_run.ticks >> 32);
    case ESP_STAT_SERVICE_MAX0: return last_run.lane[0].service_max;
    case ESP_STAT_SERVICE_MAX1: return last_run.lane[1].service_max;
    case ESP_STAT_STOPPED_BY_HOST: return last_run.stopped_by_host;
    default: return 0;
    }
}
