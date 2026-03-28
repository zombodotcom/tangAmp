"""
run_tests.py -- Parallel test runner for tangAmp WDF pipeline.

Usage:
  python run_tests.py          # Full pipeline with parallelism
  python run_tests.py --quick  # Fast smoke test (~1s, Python-only)
"""

import subprocess
import sys
import time
import os
import re
import concurrent.futures

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))

# Build env with iverilog on PATH
# On Windows, shell=True uses cmd.exe which needs Windows-style paths
env = os.environ.copy()
if sys.platform == "win32":
    iverilog_path = r"C:\iverilog\bin"
else:
    iverilog_path = "/c/iverilog/bin"
env["PATH"] = iverilog_path + os.pathsep + env.get("PATH", "")


def run_cmd(label, cmd, shell=False):
    """Run a command, return (label, success, elapsed, stdout, stderr)."""
    t0 = time.perf_counter()
    try:
        result = subprocess.run(
            cmd, cwd=PROJECT_DIR, env=env, capture_output=True, text=True,
            timeout=120, shell=shell
        )
        elapsed = time.perf_counter() - t0
        success = result.returncode == 0
        return label, success, elapsed, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        elapsed = time.perf_counter() - t0
        return label, False, elapsed, "", "TIMEOUT after 120s"
    except Exception as e:
        elapsed = time.perf_counter() - t0
        return label, False, elapsed, "", str(e)


def extract_info(stdout, stderr):
    """Extract key metrics from stdout for display."""
    text = stdout + stderr
    # Look for gain
    m = re.search(r'Gain[:\s]+([0-9.]+)\s*dB', text)
    gain = f" ({m.group(1)}dB)" if m else ""
    # Look for samples
    m = re.search(r'(\d+)\s*samples', text)
    samples = f" ({m.group(1)} samples)" if m else ""
    # Look for error percentage
    m = re.search(r'Relative error\s*:\s*([0-9.]+)%', text)
    error = f" ({m.group(1)}% error)" if m else ""
    # Look for PASS/FAIL from validate
    m = re.search(r'Overall result:\s*(PASS|FAIL)', text)
    result = f" {m.group(1)}" if m else ""
    return gain or samples or error or result or ""


def run_quick():
    """Run quick smoke test via quick_test.py."""
    total_t0 = time.perf_counter()
    print("=== tangAmp Quick Smoke Test ===")

    label, success, elapsed, stdout, stderr = run_cmd(
        "Quick smoke test",
        [sys.executable, "quick_test.py"]
    )

    # Print quick_test.py output directly
    output = stdout.strip()
    if output:
        print(output)
    if not success and stderr.strip():
        for line in stderr.strip().splitlines()[-10:]:
            print(f"  ERROR: {line}")

    return 0 if success else 1


def run_full():
    """Run full pipeline with parallelism."""
    total_t0 = time.perf_counter()
    all_ok = True

    print("=== tangAmp Full Test Pipeline ===")

    # =========================================================================
    # Stage 1 (parallel): LUT gen + Python NR sim + Python WDF sim
    # =========================================================================
    print("[PARALLEL] LUT gen + Python NR sim + Python WDF sim...")

    tasks_s1 = [
        ("LUT generation", [sys.executable, "tube_lut_gen.py"]),
        ("Python NR sim", [sys.executable, "wdf_triode_sim.py"]),
        ("Python WDF sim", [sys.executable, "wdf_triode_sim_wdf.py"]),
    ]

    results_s1 = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as pool:
        futures = {
            pool.submit(run_cmd, label, cmd): label
            for label, cmd in tasks_s1
        }
        for future in concurrent.futures.as_completed(futures):
            label, success, elapsed, stdout, stderr = future.result()
            results_s1[label] = (success, elapsed, stdout, stderr)
            info = extract_info(stdout, stderr)
            status = "OK" if success else "FAIL"
            print(f"  [{elapsed:.1f}s] {label:<24s}{status}{info}")
            if not success:
                all_ok = False
                for line in (stderr or stdout or "").strip().splitlines()[-5:]:
                    print(f"         {line}")

    # =========================================================================
    # Stage 2 (sequential): Verilog compile + sim (needs LUTs)
    # =========================================================================
    print("[SEQUENTIAL] Verilog sim (needs LUTs)...")

    lut_ok = results_s1.get("LUT generation", (False,))[0]
    if not lut_ok:
        print("  [--.-s] Verilog compile+sim   SKIP (LUT generation failed)")
        all_ok = False
    else:
        # Use shell=True for chained command
        label, success, elapsed, stdout, stderr = run_cmd(
            "Verilog compile+sim",
            "iverilog -g2012 -o wdf_sim_v ../fpga/wdf_triode_wdf_tb.v ../rtl/wdf_triode_wdf.v && vvp wdf_sim_v",
            shell=True
        )
        # Count samples in output file
        samples_info = ""
        wdf_tb_path = os.path.join(PROJECT_DIR, "wdf_tb_output.txt")
        if os.path.exists(wdf_tb_path):
            try:
                with open(wdf_tb_path) as f:
                    count = sum(1 for line in f if line.strip())
                samples_info = f" ({count} samples)"
            except Exception:
                pass
        status = "OK" if success else "FAIL"
        print(f"  [{elapsed:.1f}s] {label:<24s}{status}{samples_info}")
        if not success:
            all_ok = False
            for line in (stderr or stdout or "").strip().splitlines()[-5:]:
                print(f"         {line}")

    # =========================================================================
    # Stage 3 (parallel): Validation + Analysis
    # =========================================================================
    print("[PARALLEL] Validation + Analysis...")

    tasks_s3 = []
    wdf_tb_path = os.path.join(PROJECT_DIR, "wdf_tb_output.txt")
    if os.path.exists(wdf_tb_path):
        tasks_s3.append(("Waveform analysis", [sys.executable, "analyze_tb.py", "wdf_tb_output.txt"]))
    if os.path.exists(os.path.join(PROJECT_DIR, "validate_wdf.py")):
        tasks_s3.append(("Cross-validation", [sys.executable, "validate_wdf.py"]))

    if tasks_s3:
        with concurrent.futures.ThreadPoolExecutor(max_workers=len(tasks_s3)) as pool:
            futures = {
                pool.submit(run_cmd, label, cmd): label
                for label, cmd in tasks_s3
            }
            for future in concurrent.futures.as_completed(futures):
                label, success, elapsed, stdout, stderr = future.result()
                info = extract_info(stdout, stderr)
                status = "PASS" if success else "FAIL"
                print(f"  [{elapsed:.1f}s] {label:<24s}{status}{info}")
                if not success:
                    all_ok = False
                    for line in (stderr or stdout or "").strip().splitlines()[-5:]:
                        print(f"         {line}")
    else:
        print("  SKIP  No validation tasks to run")

    # =========================================================================
    # Summary
    # =========================================================================
    total_time = time.perf_counter() - total_t0
    if all_ok:
        print(f"PASS ({total_time:.1f}s total)")
    else:
        print(f"FAIL ({total_time:.1f}s total)")

    return 0 if all_ok else 1


def main():
    if "--quick" in sys.argv:
        return run_quick()
    else:
        return run_full()


if __name__ == "__main__":
    sys.exit(main())
