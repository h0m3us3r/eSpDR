#!/usr/bin/env bash
# Offline tests: reference codec, spectrum engine and web server, link receiver
# and the RTL stream path. Requires a C++17 compiler, FFTW3, zlib and Verilator 5
# (with --timing).
#   tests/run.sh [--long]      --long adds a full-size (1 MiB transfer) stream run
set -euo pipefail
cd "$(dirname "$0")/.."
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

step() { printf '\n== %s\n' "$*"; }

step "reference codec"
c++ -std=c++17 -O2 -Wall -Wextra -Werror -Iprotocol protocol/iq_record.cpp tests/codec_test.cpp -o "$work/codec_test"
c++ -std=c++17 -O1 -g -fsanitize=address,undefined -Iprotocol protocol/iq_record.cpp tests/codec_test.cpp -o "$work/codec_test_asan"
"$work/codec_test"
"$work/codec_test_asan" > /dev/null

step "spectrum engine and web server"
c++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread -Iprotocol -Ihost/src tests/host_test.cpp host/src/spectrum.cpp \
    host/src/web.cpp -lfftw3f -lz -o "$work/host_test"
"$work/host_test"

sh fpga/protocol_vh.sh "$work/protocol.vh" protocol/*.h
rtl=fpga/rtl
sim() {  # sim NAME [PARAMETER=VALUE]... -- SOURCES... -- PLUSARGS...
    local name=$1; shift
    local params=() sources=() plusargs=()
    while [[ $1 != -- ]]; do params+=("-G$1"); shift; done; shift
    while [[ $# -gt 0 && $1 != -- ]]; do sources+=("$1"); shift; done
    [[ $# -gt 0 ]] && shift
    plusargs=("$@")
    verilator --binary --timing -O3 -j "$(nproc)" -Wno-fatal -Wno-lint -Wno-style -Wno-TIMESCALEMOD \
        -DSIMULATION -I"$work" --top-module tb -Mdir "$work/$name" "${params[@]}" "${sources[@]}" > "$work/$name.log"
    "$work/$name/Vtb" "${plusargs[@]}" | grep -v -e '\$finish' -e '^- '
}

step "link receiver"
c++ -std=c++17 -O2 -Wall -Wextra -Werror -Iprotocol tests/link_wave.cpp -o "$work/link_wave"
link_test() {  # link_test NAME GENERATOR-ARGUMENTS...
    local name=$1; shift
    read -r _ accepted _ rejected < <("$work/link_wave" "$work/$name.bin" "$work/$name.u32" "$@")
    sim "$name" -- $rtl/link_lane.v $rtl/link_unpack.v tests/rtl/tb_link.v -- \
        +wave="$work/$name.bin" +pairs="$work/$name.u32" +units="$accepted" +framing="$rejected"
}
link_test link 12 7
link_test link_faults 40 3 --faults

step "control port"
sim control -- $rtl/uart.v $rtl/control_port.v tests/rtl/tb_control.v

step "status snapshots"
sim snapshot -- $rtl/snapshot.v tests/rtl/tb_snapshot.v
sim snapshot_slow_source SOURCE_PS=12000 -- $rtl/snapshot.v tests/rtl/tb_snapshot.v
sim snapshot_fast_destination SOURCE_PS=10000 DESTINATION_PS=4167 -- $rtl/snapshot.v tests/rtl/tb_snapshot.v

step "encoder"
"$work/codec_test" pattern "$work/pattern.u32"
sim encoder -- $rtl/encoder.v tests/rtl/tb_encoder.v -- +input="$work/pattern.u32" +output="$work/encoder.bin"
"$work/codec_test" check "$work/pattern.u32" "$work/encoder.bin"

step "reorder"
sim reorder -- $rtl/async_fifo.v $rtl/reorder.v tests/rtl/tb_reorder.v
sim reorder_max FIXED_LENGTH=16288 -- $rtl/async_fifo.v $rtl/reorder.v tests/rtl/tb_reorder.v
sim reorder_overrun OVERRUN=1 -- $rtl/async_fifo.v $rtl/reorder.v tests/rtl/tb_reorder.v
sim reorder_lost LOSE_EVERY=7 -- $rtl/async_fifo.v $rtl/reorder.v tests/rtl/tb_reorder.v
sim reorder_silent_lane LOSE_RUN=1 -- $rtl/async_fifo.v $rtl/reorder.v tests/rtl/tb_reorder.v
sim reorder_burst LOSE_BURST=1 -- $rtl/async_fifo.v $rtl/reorder.v tests/rtl/tb_reorder.v
sim reorder_misread GARBLE=1 -- $rtl/async_fifo.v $rtl/reorder.v tests/rtl/tb_reorder.v
sim reorder_misread_alone GARBLE=2 -- $rtl/async_fifo.v $rtl/reorder.v tests/rtl/tb_reorder.v

step "DDR ring"
sim ring -- $rtl/ddr_ring.v tests/rtl/tb_ring.v
sim ring_clear CLEAR_AT=5000 -- $rtl/ddr_ring.v tests/rtl/tb_ring.v

step "FT600 transmitter"
sim usb -- $rtl/usb_tx.v tests/rtl/tb_usb.v

stream_sources=($rtl/async_fifo.v $rtl/snapshot.v $rtl/reorder.v $rtl/encoder.v $rtl/packer.v $rtl/ddr_ring.v $rtl/usb_tx.v $rtl/stream.v tests/rtl/tb_stream.v)
step "stream"
sim stream -- "${stream_sources[@]}" -- +pairs="$work/stream.u32" +usb="$work/stream.bin"
"$work/codec_test" check "$work/stream.u32" "$work/stream.bin" --end
sim stream_max FIXED_LENGTH=16288 -- "${stream_sources[@]}" -- +pairs="$work/stream_max.u32" +usb="$work/stream_max.bin"
"$work/codec_test" check "$work/stream_max.u32" "$work/stream_max.bin" --end
sim stream_lost UNITS=24 LOSE_EVERY=5 -- "${stream_sources[@]}" -- +pairs="$work/stream_lost.u32" +usb="$work/stream_lost.bin"
"$work/codec_test" check "$work/stream_lost.u32" "$work/stream_lost.bin" --end

if [[ ${1:-} == --long ]]; then
    step "stream, full-size transfers"
    sim stream_full UNITS=128 RING_BITS=18 USB_WORDS=524288 DRAIN_CYCLES=1500000 \
        -- "${stream_sources[@]}" -- +pairs="$work/stream_full.u32" +usb="$work/stream_full.bin"
    "$work/codec_test" check "$work/stream_full.u32" "$work/stream_full.bin" --end
fi

step "all tests passed"
