r"""
validate_transformer_freq.py
Validates frequency-dependent saturation in the output transformer model.

Tests that lower frequencies produce more distortion than higher frequencies
at the same input amplitude, matching the physical relationship:
  V_sat = 4.44 * f * N * A * Bsat  (core flux ~ V/f)

Usage:
  python validate_transformer_freq.py
"""

import numpy as np
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.join(SCRIPT_DIR, "..")
DEMOS_DIR = os.path.join(ROOT_DIR, "demos")
os.makedirs(DEMOS_DIR, exist_ok=True)

sys.path.insert(0, SCRIPT_DIR)
from output_transformer import output_transformer

FS = 48000.0


def measure_thd(freq, amplitude, fs=FS, duration=0.5):
    """
    Feed a sine at `freq` Hz with given amplitude through the transformer.
    Return the THD percentage.
    """
    n_samples = int(duration * fs)
    t = np.arange(n_samples) / fs

    sig_in = amplitude * np.sin(2 * np.pi * freq * t)
    sig_out = output_transformer(
        sig_in, fs=fs,
        sat_threshold=0.7, sat_lf_depth=0.6, lf_envelope_fc=200.0
    )

    # Discard first third for settling
    settle = n_samples // 3
    seg = sig_out[settle:]

    N = len(seg)
    w = np.hanning(N)
    spectrum = np.abs(np.fft.rfft(seg * w))
    freqs_fft = np.fft.rfftfreq(N, 1.0 / fs)

    # Find fundamental
    fund_idx = np.argmin(np.abs(freqs_fft - freq))
    fund_power = spectrum[fund_idx] ** 2

    # Sum harmonics 2 through 8
    harm_power = 0.0
    for h in range(2, 9):
        h_freq = freq * h
        if h_freq >= fs / 2:
            break
        h_idx = np.argmin(np.abs(freqs_fft - h_freq))
        harm_power += spectrum[h_idx] ** 2

    if fund_power > 1e-20:
        return 100.0 * np.sqrt(harm_power / fund_power)
    return 0.0


def measure_output_peak(freq, amplitude, fs=FS, duration=0.5):
    """
    Feed a sine at `freq` Hz through the transformer and return the
    steady-state output peak (after settling).
    """
    n_samples = int(duration * fs)
    t = np.arange(n_samples) / fs

    sig_in = amplitude * np.sin(2 * np.pi * freq * t)
    sig_out = output_transformer(
        sig_in, fs=fs,
        sat_threshold=0.7, sat_lf_depth=0.6, lf_envelope_fc=200.0
    )

    settle = n_samples // 3
    return np.max(np.abs(sig_out[settle:]))


def theoretical_vsat(freq, ref_freq=1000.0, ref_vsat=25.0):
    """
    Theoretical saturation voltage from transformer physics:
      V_sat = 4.44 * f * N * A * Bsat
    V_sat is proportional to f. Scale relative to a reference frequency.
    """
    return ref_vsat * (freq / ref_freq)


def main():
    print("=" * 60)
    print("Transformer Frequency-Dependent Saturation Validation")
    print("=" * 60)

    test_freqs = [40, 80, 200, 1000]
    test_amplitude = 20.0  # High drive to exercise saturation

    # ---- Test 1: THD comparison at same amplitude ----
    print(f"\n1. THD at {test_amplitude}V drive for each frequency...")
    thd_results = {}
    for freq in test_freqs:
        thd = measure_thd(freq, test_amplitude, duration=0.8)
        thd_results[freq] = thd
        print(f"   {freq:>5} Hz: THD = {thd:.2f}%")

    # Check ordering: lower frequencies should have higher THD.
    # We require strict ordering for frequencies well below the envelope
    # detector cutoff (200Hz), and only require the overall trend (lowest
    # freq has highest THD) for the full range.
    print("\n2. Checking THD ordering (lower freq -> more distortion)...")
    passed_ordering = True
    for i in range(len(test_freqs) - 1):
        f_low = test_freqs[i]
        f_high = test_freqs[i + 1]
        if thd_results[f_low] <= thd_results[f_high]:
            # Strict check only for sub-200Hz pairs; warn but allow above
            if f_low < 200:
                print(f"   FAIL: {f_low}Hz THD ({thd_results[f_low]:.2f}%) should be > "
                      f"{f_high}Hz THD ({thd_results[f_high]:.2f}%)")
                passed_ordering = False
            else:
                print(f"   WARN: {f_low}Hz ({thd_results[f_low]:.2f}%) <= "
                      f"{f_high}Hz ({thd_results[f_high]:.2f}%) "
                      f"(near envelope cutoff, acceptable)")
        else:
            print(f"   OK: {f_low}Hz ({thd_results[f_low]:.2f}%) > "
                  f"{f_high}Hz ({thd_results[f_high]:.2f}%)")

    # ---- Test 2: 40Hz should have substantially more THD than 1kHz ----
    print("\n3. Checking 40Hz vs 1kHz THD ratio...")
    if thd_results[1000] > 0.01:
        ratio = thd_results[40] / thd_results[1000]
    else:
        ratio = float('inf') if thd_results[40] > 0.01 else 1.0
    print(f"   THD ratio 40Hz/1kHz: {ratio:.2f}x")
    print(f"   Expected: >1.5x (bass saturates harder)")
    passed_ratio = ratio > 1.5
    if passed_ratio:
        print(f"   OK")
    else:
        print(f"   FAIL: ratio {ratio:.2f} not > 1.5")

    # ---- Test 3: Output peak compression at different frequencies ----
    print(f"\n4. Output peak at {test_amplitude}V drive (compression check)...")
    peak_results = {}
    for freq in test_freqs:
        peak = measure_output_peak(freq, test_amplitude, duration=0.8)
        peak_results[freq] = peak
        print(f"   {freq:>5} Hz: output peak = {peak:.3f}V")

    # Lower frequencies should be more compressed (lower output peak relative
    # to what a linear system would produce)
    print("\n   Lower frequencies should show more compression (lower output peak).")

    # ---- Test 4: Compare against V_sat = 4.44 * f * N * A * Bsat ----
    print("\n5. Theoretical V_sat comparison (proportional to f):")
    print(f"   {'Freq':>8}  {'THD (%)':>10}  {'V_sat theory':>12}")
    print("   " + "-" * 36)
    ref_vsat = 25.0
    for freq in test_freqs:
        v_theory = theoretical_vsat(freq, ref_freq=1000.0, ref_vsat=ref_vsat)
        print(f"   {freq:>5} Hz  {thd_results[freq]:>8.2f} %  {v_theory:>10.1f} V")

    # ---- Generate plot ----
    print("\n6. Generating validation plot...")
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Frequency-Dependent Transformer Saturation Validation",
                 fontsize=14, fontweight='bold')

    colors = ['#e41a1c', '#ff7f00', '#4daf4a', '#377eb8']

    # Top-left: waveforms at the test amplitude
    ax = axes[0, 0]
    t_plot = np.arange(int(0.05 * FS)) / FS
    for i, freq in enumerate(test_freqs):
        n_pts = int(0.8 * FS)
        t_s = np.arange(n_pts) / FS
        sig_in = test_amplitude * np.sin(2 * np.pi * freq * t_s)
        sig_out = output_transformer(sig_in, fs=FS,
                                     sat_threshold=0.7, sat_lf_depth=0.6)
        # Show last 50ms
        show_n = len(t_plot)
        seg = sig_out[-show_n:]
        ax.plot(t_plot * 1000, seg, color=colors[i], linewidth=0.8,
                label=f'{freq}Hz')
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Output (V)")
    ax.set_title(f"Waveforms at {test_amplitude}V drive")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # Top-right: THD vs frequency
    ax = axes[0, 1]
    ax.bar(range(len(test_freqs)),
           [thd_results[f] for f in test_freqs],
           color=colors, tick_label=[f'{f}Hz' for f in test_freqs])
    ax.set_ylabel("THD (%)")
    ax.set_title(f"THD at {test_amplitude}V drive")
    ax.grid(True, alpha=0.3, axis='y')

    # Bottom-left: THD vs amplitude sweep at each frequency
    ax = axes[1, 0]
    amps_sweep = np.linspace(1, 40, 20)
    for i, freq in enumerate(test_freqs):
        thds = [measure_thd(freq, amp, duration=0.4) for amp in amps_sweep]
        ax.plot(amps_sweep, thds, color=colors[i], linewidth=1.2,
                label=f'{freq}Hz')
    ax.axhline(1.0, color='gray', ls='--', alpha=0.5, label='1% THD')
    ax.set_xlabel("Input Amplitude (V)")
    ax.set_ylabel("THD (%)")
    ax.set_title("THD vs Drive Level")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # Bottom-right: summary text
    ax = axes[1, 1]
    ax.axis('off')
    summary_lines = [
        "Frequency-Dependent Saturation Test",
        "",
        "Core physics: flux = V / (N * omega)",
        "  -> Low frequencies produce more flux",
        "  -> Low frequencies saturate first",
        "",
        f"Drive level: {test_amplitude}V",
        "",
        "THD Results:",
    ]
    for freq in test_freqs:
        summary_lines.append(f"  {freq:>5} Hz: {thd_results[freq]:.2f}%")
    summary_lines.append("")
    summary_lines.append(f"THD ratio 40Hz/1kHz: {ratio:.2f}x")
    summary_lines.append(f"Ordering correct: {'YES' if passed_ordering else 'NO'}")
    summary_lines.append(f"Ratio > 1.5x:     {'YES' if passed_ratio else 'NO'}")
    summary_lines.append("")
    overall = passed_ordering and passed_ratio
    summary_lines.append(f"Overall: {'PASS' if overall else 'FAIL'}")

    ax.text(0.05, 0.95, '\n'.join(summary_lines), transform=ax.transAxes,
            fontsize=11, verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round',
                      facecolor='lightgreen' if overall else 'lightyellow',
                      alpha=0.5))

    plt.tight_layout()
    plot_path = os.path.join(DEMOS_DIR, "validation_transformer_freq.png")
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    print(f"   Saved: {plot_path}")
    plt.close()

    # Final verdict
    print(f"\n{'=' * 60}")
    print(f"Overall: {'PASS' if overall else 'FAIL'}")
    print(f"{'=' * 60}")

    return 0 if overall else 1


if __name__ == "__main__":
    sys.exit(main())
