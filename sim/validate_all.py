#!/usr/bin/env python3
"""
validate_all.py -- Run all validation scripts and report summary.

Runs each validation in order, captures exit codes, and prints a summary table.
Exit code 0 = all passed, 1 = at least one failed.
"""

import subprocess
import sys
import time

tests = [
    ("Quick Smoke Test",    "python quick_test.py"),
    ("WDF Sim (NR)",        "python wdf_triode_sim.py"),
    ("WDF Sim (WDF)",       "python wdf_triode_sim_wdf.py"),
    ("Cross-Validation",    "python validate_wdf.py"),
    ("Physics Validation",  "python validate_physics.py"),
    ("SPICE Validation",    "python validate_spice.py"),
]


def main():
    print("=" * 64)
    print("  tangAmp -- Full Validation Suite")
    print("=" * 64)
    print()

    results = []
    t_start = time.perf_counter()

    for name, cmd in tests:
        print(f"--- Running: {name} ---")
        print(f"    Command: {cmd}")
        t0 = time.perf_counter()

        try:
            proc = subprocess.run(
                cmd,
                shell=True,
                capture_output=True,
                text=True,
                timeout=600,
            )
            elapsed = time.perf_counter() - t0
            exit_code = proc.returncode
            passed = (exit_code == 0)

            # Print last few lines of output for context
            output_lines = (proc.stdout or "").strip().split("\n")
            for line in output_lines[-5:]:
                print(f"    {line}")
            if proc.stderr:
                err_lines = proc.stderr.strip().split("\n")
                # Only show non-warning stderr lines
                for line in err_lines[-3:]:
                    if line.strip():
                        print(f"    [stderr] {line}")

        except subprocess.TimeoutExpired:
            elapsed = time.perf_counter() - t0
            exit_code = -1
            passed = False
            print(f"    TIMEOUT after {elapsed:.0f}s")
        except Exception as e:
            elapsed = time.perf_counter() - t0
            exit_code = -2
            passed = False
            print(f"    ERROR: {e}")

        status = "PASS" if passed else "FAIL"
        print(f"    Result: {status} ({elapsed:.1f}s)")
        print()
        results.append((name, passed, elapsed, exit_code))

    total_time = time.perf_counter() - t_start

    # Summary table
    print("=" * 64)
    print("  SUMMARY")
    print("=" * 64)
    print()
    print(f"  {'Test':<25s} {'Status':<8s} {'Time':>8s}")
    print(f"  {'-'*25} {'-'*8} {'-'*8}")

    all_passed = True
    for name, passed, elapsed, exit_code in results:
        status = "PASS" if passed else "FAIL"
        if not passed:
            all_passed = False
        print(f"  {name:<25s} {status:<8s} {elapsed:>7.1f}s")

    print(f"  {'-'*25} {'-'*8} {'-'*8}")
    print(f"  {'Total':<25s} {'':8s} {total_time:>7.1f}s")
    print()

    if all_passed:
        print("  >>> ALL VALIDATIONS PASSED <<<")
        return 0
    else:
        n_fail = sum(1 for _, p, _, _ in results if not p)
        print(f"  >>> {n_fail} VALIDATION(S) FAILED <<<")
        return 1


if __name__ == "__main__":
    sys.exit(main())
