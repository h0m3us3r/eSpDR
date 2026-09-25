#!/bin/sh
# Translates the numeric #defines of the shared protocol headers into
# Verilog `defines: protocol_vh.sh OUTPUT.vh HEADER.h...
set -eu
out=$1
shift
awk '
    /^#define [A-Z0-9_]+ (0x[0-9A-Fa-f]+|[0-9]+)([ \t]|$)/ {
        value = $3
        if (value ~ /^0x/) value = "32'"'"'h" substr(value, 3)
        printf "`define %s %s\n", $2, value
    }
' "$@" > "$out"
