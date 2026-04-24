#!/usr/bin/env bash
# Usage:
#   ./build.sh sinetest                  -> builds one target, outputs bitstreams/sinetest.fs
#   ./build.sh sinetest blink button_test -> builds multiple sequentially
#   ./build.sh all                       -> builds every synthesize_*.tcl in this directory
#
# Each build's .fs is copied to bitstreams/<target>.fs so they don't overwrite.

set -e

GW_SH="/c/Gowin/Gowin_V1.9.11.03_Education_x64/IDE/bin/gw_sh.exe"

if [ ! -f "$GW_SH" ]; then
    echo "Error: gw_sh not found at $GW_SH"
    exit 1
fi

mkdir -p bitstreams

# Expand "all" to every target
if [ "$1" = "all" ]; then
    TARGETS=()
    for tcl in synthesize_*.tcl; do
        t="${tcl#synthesize_}"
        t="${t%.tcl}"
        TARGETS+=("$t")
    done
else
    TARGETS=("$@")
fi

if [ ${#TARGETS[@]} -eq 0 ]; then
    echo "Usage: $0 <target> [target2 ...] | all"
    echo ""
    echo "Available targets:"
    ls synthesize_*.tcl | sed 's/synthesize_//; s/\.tcl//' | sed 's/^/  /'
    exit 1
fi

declare -a SUCCESS FAILED

for TARGET in "${TARGETS[@]}"; do
    TCL="synthesize_${TARGET}.tcl"

    if [ ! -f "$TCL" ]; then
        echo "!!! Skipping $TARGET — $TCL not found"
        FAILED+=("$TARGET")
        continue
    fi

    echo ""
    echo "================================================================"
    echo "  Building: $TARGET"
    echo "================================================================"

    if "$GW_SH" "$TCL" > "bitstreams/${TARGET}.log" 2>&1; then
        if [ -f "impl/pnr/project.fs" ]; then
            cp "impl/pnr/project.fs"  "bitstreams/${TARGET}.fs"
            cp "impl/pnr/project.bin" "bitstreams/${TARGET}.bin" 2>/dev/null || true
            echo "  OK  -> bitstreams/${TARGET}.fs"
            SUCCESS+=("$TARGET")
        else
            echo "  FAIL — no project.fs produced (see bitstreams/${TARGET}.log)"
            FAILED+=("$TARGET")
        fi
    else
        echo "  FAIL — synthesis error (see bitstreams/${TARGET}.log)"
        FAILED+=("$TARGET")
    fi
done

echo ""
echo "================================================================"
echo "  Summary"
echo "================================================================"
echo "Built ${#SUCCESS[@]}/${#TARGETS[@]} targets into bitstreams/"
if [ ${#SUCCESS[@]} -gt 0 ]; then
    echo "  OK: ${SUCCESS[*]}"
fi
if [ ${#FAILED[@]} -gt 0 ]; then
    echo "  FAILED: ${FAILED[*]}"
fi
