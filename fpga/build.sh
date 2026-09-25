#!/bin/sh
# Builds fpga/build/iqstream.bit (Vivado 2025.2 on PATH, or set VIVADO).
set -eu
here=$(cd "$(dirname "$0")" && pwd)
out=${1:-$here/build}
mkdir -p "$out"
exec "${VIVADO:-vivado}" -mode batch -nojournal -log "$out/vivado.log" \
    -source "$here/build.tcl" -tclargs "$out"
