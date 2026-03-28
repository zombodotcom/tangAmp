#!/usr/bin/env python3
"""
Cross-validation script: compare WDF vs Newton-Raphson simulation outputs.

File format: sample_number input_q16 output_q16
Q16.16 fixed-point: divide by 65536 to get volts.
"""

import sys
import os
import numpy as np

SCALE = 65536.0
SETTLE_SAMPLES = 2000
VERILOG_SETTLE_SAMPLES = 10000

WDF_NR_FILE      = "wdf_sim_output.txt"       # Newton-Raphson reference
WDF_WDF_FILE     = "wdf_sim_wdf_output.txt"   # WDF reference
WDF_VERILOG_FILE = "wdf_tb_output.txt"        # Verilog WDF (may not exist)

NR_VS_WDF_THRESHOLD     = 0.05   # 5%  relative error tolerance
VERILOG_VS_NR_THRESHOLD = 0.10   # 10% relative error tolerance (fixed-point)


def load_file(path):
    """Load a sim output file. Returns (inputs, outputs) as float arrays in volts."""
    samples, inputs, outputs = [], [], []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) != 3:
                continue
            samples.append(int(parts[0]))
            inputs.append(int(parts[1]) / SCALE)
            outputs.append(int(parts[2]) / SCALE)
    return np.array(inputs), np.array(outputs)


def compare(name_a, outputs_a, name_b, outputs_b, threshold, rms_only=False):
    """
    Compare two output arrays (already settled).
    If rms_only=True, compare RMS levels instead of sample-by-sample
    (useful when sample counts or phase differ, e.g. Verilog vs Python).
    Returns (passed, max_diff, rms_diff, rel_error).
    """
    ref_rms = float(np.sqrt(np.mean(outputs_a ** 2)))
    test_rms = float(np.sqrt(np.mean(outputs_b ** 2)))

    if rms_only:
        # Compare RMS levels (tolerant of phase/length differences)
        rms_diff = abs(ref_rms - test_rms)
        max_diff = rms_diff  # not meaningful for RMS comparison
        if ref_rms < 1e-9:
            rel_error = 0.0 if rms_diff < 1e-9 else float("inf")
        else:
            rel_error = rms_diff / ref_rms
        n = f"{len(outputs_a)} vs {len(outputs_b)}"
    else:
        # Sample-by-sample comparison
        n_samp = min(len(outputs_a), len(outputs_b))
        a = outputs_a[:n_samp]
        b = outputs_b[:n_samp]
        diff = np.abs(a - b)
        max_diff = float(np.max(diff))
        rms_diff = float(np.sqrt(np.mean(diff ** 2)))
        if ref_rms < 1e-9:
            rel_error = 0.0 if rms_diff < 1e-9 else float("inf")
        else:
            rel_error = rms_diff / ref_rms
        n = str(n_samp)

    passed = rel_error <= threshold

    status = "PASS" if passed else "FAIL"
    mode = "RMS-level" if rms_only else "sample-by-sample"
    print(f"--- {name_a} vs {name_b} ({mode}) ---")
    print(f"  Samples          : {n} (after skipping {SETTLE_SAMPLES} settling)")
    print(f"  Ref RMS          : {ref_rms:.4f} V")
    print(f"  Test RMS         : {test_rms:.4f} V")
    print(f"  Max difference   : {max_diff:.6f} V")
    print(f"  RMS difference   : {rms_diff:.6f} V")
    print(f"  Relative error   : {rel_error*100:.3f}%  (threshold {threshold*100:.1f}%)")
    print(f"  Result           : {status}")
    print()

    return passed, max_diff, rms_diff, rel_error


def main():
    all_passed = True
    any_comparison_ran = False

    # ------------------------------------------------------------------
    # Load Newton-Raphson reference (required)
    # ------------------------------------------------------------------
    if not os.path.exists(WDF_NR_FILE):
        print(f"ERROR: Required reference file '{WDF_NR_FILE}' not found.")
        sys.exit(1)

    print(f"Loading Newton-Raphson reference: {WDF_NR_FILE}")
    nr_inputs, nr_outputs = load_file(WDF_NR_FILE)
    nr_settled = nr_outputs[SETTLE_SAMPLES:]
    print(f"  Total samples: {len(nr_outputs)}, settled: {len(nr_settled)}")
    print()

    # ------------------------------------------------------------------
    # Load WDF Python reference (required)
    # ------------------------------------------------------------------
    if not os.path.exists(WDF_WDF_FILE):
        print(f"ERROR: Required WDF reference file '{WDF_WDF_FILE}' not found.")
        sys.exit(1)

    print(f"Loading WDF Python reference: {WDF_WDF_FILE}")
    wdf_inputs, wdf_outputs = load_file(WDF_WDF_FILE)
    wdf_settled = wdf_outputs[SETTLE_SAMPLES:]
    print(f"  Total samples: {len(wdf_outputs)}, settled: {len(wdf_settled)}")
    print()

    # ------------------------------------------------------------------
    # Comparison 1: Newton-Raphson vs WDF Python (5% threshold)
    # ------------------------------------------------------------------
    passed, _, _, _ = compare(
        "Newton-Raphson", nr_settled,
        "WDF-Python",     wdf_settled,
        NR_VS_WDF_THRESHOLD,
    )
    any_comparison_ran = True
    if not passed:
        all_passed = False

    # ------------------------------------------------------------------
    # Comparison 2: Newton-Raphson vs Verilog WDF (10% threshold, optional)
    # ------------------------------------------------------------------
    if os.path.exists(WDF_VERILOG_FILE):
        print(f"Loading Verilog WDF output: {WDF_VERILOG_FILE}")
        vlog_inputs, vlog_outputs = load_file(WDF_VERILOG_FILE)
        vlog_settled = vlog_outputs[VERILOG_SETTLE_SAMPLES:]
        print(f"  Total samples: {len(vlog_outputs)}, settled: {len(vlog_settled)}")
        print()

        passed, _, _, _ = compare(
            "Newton-Raphson", nr_settled,
            "Verilog-WDF",    vlog_settled,
            VERILOG_VS_NR_THRESHOLD,
            rms_only=True,  # Verilog may have different sample count/phase
        )
        any_comparison_ran = True
        if not passed:
            all_passed = False
    else:
        print(f"Skipping Verilog comparison: '{WDF_VERILOG_FILE}' not found.")
        print()

    # ------------------------------------------------------------------
    # Final summary
    # ------------------------------------------------------------------
    if not any_comparison_ran:
        print("No comparisons were run.")
        sys.exit(1)

    if all_passed:
        print("Overall result: PASS")
        sys.exit(0)
    else:
        print("Overall result: FAIL")
        sys.exit(1)


if __name__ == "__main__":
    main()
