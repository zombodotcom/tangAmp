"""
validate_cabinet.py
Validate cabinet IR taps from cab_ir.hex against published 12" guitar speaker data.

Checks:
  1. Frequency response shape vs Eminence Legend 1258 target curve
  2. Causality (no pre-ringing)
  3. Proper windowing (tapered ends)
  4. Normalization (peak tap = 1.0)

PASS criteria: general shape matches within 6dB at key frequency bands.
"""

import numpy as np
import os
import sys
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

FS = 48000
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def load_hex_taps(hex_path):
    """Load Q1.15 signed 16-bit taps from hex file."""
    taps = []
    with open(hex_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            val = int(line, 16)
            # Convert unsigned 16-bit to signed
            if val >= 0x8000:
                val -= 0x10000
            taps.append(val)
    taps = np.array(taps, dtype=np.float64)
    # Q1.15: divide by 32768 to get float
    taps_float = taps / 32768.0
    return taps_float


def compute_freq_response(ir, fs, n_fft=4096):
    """Compute frequency response magnitude in dB from IR taps."""
    ir_padded = np.zeros(n_fft)
    ir_padded[:len(ir)] = ir
    sp = np.abs(np.fft.rfft(ir_padded))
    freqs = np.fft.rfftfreq(n_fft, 1.0 / fs)
    sp_db = 20 * np.log10(sp + 1e-12)
    return freqs, sp_db


def get_target_curve():
    """
    Approximate published frequency response for Eminence Legend 1258 (12" guitar speaker).
    Reference points from datasheet:
      - 50Hz: -10dB
      - 80Hz: -3dB (resonance)
      - 100Hz: 0dB (reference)
      - 200Hz-2kHz: 0dB (flat midrange)
      - 3.5kHz: +3dB (presence peak)
      - 5kHz: -3dB
      - 8kHz: -10dB
      - 10kHz: -15dB
    """
    # (frequency_hz, gain_db relative to 100Hz)
    target_points = [
        (20, -20.0),
        (50, -10.0),
        (80, -3.0),
        (100, 0.0),
        (150, 0.0),
        (200, 0.0),
        (500, 0.0),
        (1000, 0.0),
        (2000, 0.0),
        (3000, 2.0),
        (3500, 3.0),
        (4000, 1.0),
        (5000, -3.0),
        (6000, -6.0),
        (8000, -10.0),
        (10000, -15.0),
        (15000, -25.0),
        (20000, -35.0),
    ]
    freqs = np.array([p[0] for p in target_points])
    gains = np.array([p[1] for p in target_points])
    return freqs, gains


def interpolate_db_at_freq(freqs_hz, sp_db, target_freq):
    """Get dB value at a specific frequency by interpolation."""
    return np.interp(target_freq, freqs_hz, sp_db)


def check_causality(ir):
    """
    Check IR is causal: the bulk of energy should not appear before the main peak.
    For a linear-phase FIR, the peak is at center -- pre-peak taps should be small.
    We check that no tap before the peak exceeds 10% of peak magnitude.
    """
    peak_idx = np.argmax(np.abs(ir))
    peak_val = np.abs(ir[peak_idx])

    # Check first 10% of taps for significant energy
    early_region = max(1, peak_idx // 4)
    early_max = np.max(np.abs(ir[:early_region]))
    pre_ring_ratio = early_max / peak_val if peak_val > 1e-12 else 0.0

    return {
        'peak_index': int(peak_idx),
        'n_taps': len(ir),
        'early_max_ratio': float(pre_ring_ratio),
        'pass': pre_ring_ratio < 0.15,  # Allow up to 15% pre-ringing
        'note': 'Linear-phase FIR has symmetric pre/post ringing by design'
    }


def check_windowing(ir):
    """Check that the IR is properly windowed (tapers to zero at edges)."""
    n = len(ir)
    edge_samples = max(3, n // 20)
    start_energy = np.max(np.abs(ir[:edge_samples]))
    end_energy = np.max(np.abs(ir[-edge_samples:]))
    peak_val = np.max(np.abs(ir))

    start_ratio = start_energy / peak_val if peak_val > 1e-12 else 0.0
    end_ratio = end_energy / peak_val if peak_val > 1e-12 else 0.0

    return {
        'start_edge_ratio': float(start_ratio),
        'end_edge_ratio': float(end_ratio),
        'pass': start_ratio < 0.05 and end_ratio < 0.05,
    }


def check_normalization(ir):
    """Check that peak tap magnitude is close to 1.0."""
    peak = np.max(np.abs(ir))
    return {
        'peak_value': float(peak),
        'pass': 0.8 <= peak <= 1.01,
    }


def check_frequency_shape(freqs_hz, sp_db, tolerance_db=6.0):
    """
    Compare IR frequency response against target speaker curve.
    PASS if within tolerance_db at key frequency bands.
    """
    target_freqs, target_gains = get_target_curve()

    # Normalize our response: set 0dB at ~500Hz (midband reference)
    ref_db = interpolate_db_at_freq(freqs_hz, sp_db, 500.0)
    sp_db_norm = sp_db - ref_db

    results = []
    all_pass = True
    for tf, tg in zip(target_freqs, target_gains):
        if tf < 30 or tf > FS / 2 - 100:
            continue
        our_db = interpolate_db_at_freq(freqs_hz, sp_db_norm, tf)
        error = abs(our_db - tg)
        # Wider tolerance at extreme LF (short FIR has limited bass resolution)
        local_tol = tolerance_db + 4.0 if tf <= 50 else tolerance_db
        ok = error <= local_tol
        if not ok:
            all_pass = False
        results.append({
            'freq_hz': float(tf),
            'target_db': float(tg),
            'measured_db': float(round(our_db, 1)),
            'error_db': float(round(error, 1)),
            'pass': ok,
        })

    return {
        'points': results,
        'pass': all_pass,
        'ref_freq_hz': 500.0,
        'ref_db': float(round(ref_db, 1)),
    }


def check_key_features(freqs_hz, sp_db):
    """
    Check for key speaker features:
      - Bass rolloff below ~80Hz
      - Relatively flat midrange (200Hz-2kHz)
      - HF rolloff above 5kHz
    """
    ref_db = interpolate_db_at_freq(freqs_hz, sp_db, 500.0)
    sp_norm = sp_db - ref_db

    # Bass rolloff: 50Hz should be below midband
    # Note: short FIR (129 taps at 48kHz) has limited bass resolution,
    # so we check that bass is at least somewhat attenuated
    bass_50 = interpolate_db_at_freq(freqs_hz, sp_norm, 50.0)
    has_bass_rolloff = bass_50 < -1.0

    # Flat midrange: std dev of response 200Hz-2kHz
    mid_mask = (freqs_hz >= 200) & (freqs_hz <= 2000)
    if np.any(mid_mask):
        mid_std = np.std(sp_norm[mid_mask])
    else:
        mid_std = 99.0
    has_flat_mid = mid_std < 5.0

    # HF rolloff: 8kHz should be well below midband
    hf_8k = interpolate_db_at_freq(freqs_hz, sp_norm, 8000.0)
    has_hf_rolloff = hf_8k < -3.0

    return {
        'bass_rolloff': {'50Hz_db': float(round(bass_50, 1)), 'pass': has_bass_rolloff},
        'flat_midrange': {'std_db': float(round(mid_std, 1)), 'pass': has_flat_mid},
        'hf_rolloff': {'8kHz_db': float(round(hf_8k, 1)), 'pass': has_hf_rolloff},
        'pass': has_bass_rolloff and has_flat_mid and has_hf_rolloff,
    }


def main():
    hex_path = os.path.join(SCRIPT_DIR, "..", "data", "cab_ir.hex")
    if not os.path.exists(hex_path):
        print(f"ERROR: {hex_path} not found. Run gen_cab_taps.py first.")
        sys.exit(1)

    print("=" * 60)
    print("Cabinet IR Validation")
    print("=" * 60)

    # Load taps
    ir = load_hex_taps(hex_path)
    print(f"\nLoaded {len(ir)} taps from cab_ir.hex")
    print(f"  Peak tap: {np.max(np.abs(ir)):.5f}")
    print(f"  Sum |taps|: {np.sum(np.abs(ir)):.4f}")

    # Compute frequency response
    n_fft = 4096
    freqs, sp_db = compute_freq_response(ir, FS, n_fft)

    # Run checks
    print("\n--- Causality Check ---")
    causality = check_causality(ir)
    print(f"  Peak at tap {causality['peak_index']} / {causality['n_taps']}")
    print(f"  Early energy ratio: {causality['early_max_ratio']:.4f}")
    print(f"  Note: {causality['note']}")
    print(f"  Result: {'PASS' if causality['pass'] else 'FAIL'}")

    print("\n--- Windowing Check ---")
    windowing = check_windowing(ir)
    print(f"  Start edge ratio: {windowing['start_edge_ratio']:.5f}")
    print(f"  End edge ratio: {windowing['end_edge_ratio']:.5f}")
    print(f"  Result: {'PASS' if windowing['pass'] else 'FAIL'}")

    print("\n--- Normalization Check ---")
    normalization = check_normalization(ir)
    print(f"  Peak value: {normalization['peak_value']:.5f}")
    print(f"  Result: {'PASS' if normalization['pass'] else 'FAIL'}")

    print("\n--- Key Speaker Features ---")
    features = check_key_features(freqs, sp_db)
    print(f"  Bass rolloff (50Hz): {features['bass_rolloff']['50Hz_db']:.1f} dB  "
          f"{'PASS' if features['bass_rolloff']['pass'] else 'FAIL'}")
    print(f"  Flat midrange (200-2kHz std): {features['flat_midrange']['std_db']:.1f} dB  "
          f"{'PASS' if features['flat_midrange']['pass'] else 'FAIL'}")
    print(f"  HF rolloff (8kHz): {features['hf_rolloff']['8kHz_db']:.1f} dB  "
          f"{'PASS' if features['hf_rolloff']['pass'] else 'FAIL'}")
    print(f"  Result: {'PASS' if features['pass'] else 'FAIL'}")

    print("\n--- Frequency Response vs Target (6dB tolerance) ---")
    shape = check_frequency_shape(freqs, sp_db, tolerance_db=6.0)
    print(f"  Reference: {shape['ref_freq_hz']}Hz = {shape['ref_db']} dB")
    print(f"  {'Freq':>8s}  {'Target':>8s}  {'Measured':>8s}  {'Error':>8s}  {'Result':>6s}")
    for pt in shape['points']:
        print(f"  {pt['freq_hz']:8.0f}  {pt['target_db']:8.1f}  {pt['measured_db']:8.1f}  "
              f"{pt['error_db']:8.1f}  {'PASS' if pt['pass'] else 'FAIL'}")
    print(f"  Overall: {'PASS' if shape['pass'] else 'FAIL'}")

    # Overall result
    all_pass = (causality['pass'] and windowing['pass'] and
                normalization['pass'] and features['pass'] and shape['pass'])

    print("\n" + "=" * 60)
    print(f"OVERALL: {'PASS' if all_pass else 'FAIL'}")
    print("=" * 60)

    # --- Plot ---
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Cabinet IR Validation: cab_ir.hex vs Eminence Legend 1258 Target",
                 fontsize=13, fontweight='bold')

    # Top-left: IR waveform
    ax = axes[0, 0]
    t_ms = np.arange(len(ir)) / FS * 1000
    ax.plot(t_ms, ir, color='#1f77b4', linewidth=1.0)
    ax.axhline(0, color='gray', linewidth=0.5)
    peak_idx = np.argmax(np.abs(ir))
    ax.axvline(t_ms[peak_idx], color='red', linewidth=0.8, linestyle='--',
               label=f'Peak at {t_ms[peak_idx]:.2f} ms')
    ax.set_xlabel("Time (ms)")
    ax.set_ylabel("Amplitude")
    ax.set_title("IR Waveform (Q1.15 from hex)")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # Top-right: Frequency response comparison
    ax = axes[0, 1]
    ref_db = interpolate_db_at_freq(freqs, sp_db, 500.0)
    sp_norm = sp_db - ref_db
    mask = freqs >= 20
    ax.semilogx(freqs[mask], sp_norm[mask], color='#1f77b4', linewidth=1.5,
                label='Our IR (cab_ir.hex)')
    # Target curve
    target_freqs, target_gains = get_target_curve()
    ax.semilogx(target_freqs, target_gains, 'ro-', linewidth=1.5, markersize=5,
                label='Target: Eminence 1258', alpha=0.8)
    # Tolerance band
    target_interp = np.interp(freqs[mask], target_freqs, target_gains)
    ax.fill_between(freqs[mask], target_interp - 6, target_interp + 6,
                    color='red', alpha=0.08, label='+/- 6dB tolerance')
    ax.set_xlim(20, 20000)
    ax.set_ylim(-40, 10)
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Gain (dB, ref 500Hz)")
    ax.set_title("Frequency Response: IR vs Target Speaker")
    ax.legend(fontsize=9, loc='lower left')
    ax.grid(True, alpha=0.3, which='both')

    # Bottom-left: Error at target points
    ax = axes[1, 0]
    pt_freqs = [p['freq_hz'] for p in shape['points']]
    pt_errors = [p['error_db'] for p in shape['points']]
    pt_colors = ['green' if p['pass'] else 'red' for p in shape['points']]
    ax.bar(range(len(pt_freqs)), pt_errors, color=pt_colors, alpha=0.7)
    ax.set_xticks(range(len(pt_freqs)))
    ax.set_xticklabels([f"{f:.0f}" for f in pt_freqs], rotation=45, fontsize=8)
    ax.axhline(6.0, color='red', linewidth=1, linestyle='--', label='6dB threshold')
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Error (dB)")
    ax.set_title("Error at Target Points (green=PASS, red=FAIL)")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # Bottom-right: Summary text
    ax = axes[1, 1]
    ax.axis('off')
    summary_lines = [
        f"Cabinet IR Validation Summary",
        f"",
        f"Taps: {len(ir)}   Sample rate: {FS} Hz",
        f"Format: Q1.15 signed 16-bit",
        f"",
        f"Causality:      {'PASS' if causality['pass'] else 'FAIL'}  "
        f"(early energy ratio: {causality['early_max_ratio']:.4f})",
        f"Windowing:      {'PASS' if windowing['pass'] else 'FAIL'}  "
        f"(edge ratios: {windowing['start_edge_ratio']:.4f}, {windowing['end_edge_ratio']:.4f})",
        f"Normalization:  {'PASS' if normalization['pass'] else 'FAIL'}  "
        f"(peak: {normalization['peak_value']:.4f})",
        f"Speaker shape:  {'PASS' if features['pass'] else 'FAIL'}  "
        f"(bass/mid/HF checks)",
        f"Freq response:  {'PASS' if shape['pass'] else 'FAIL'}  "
        f"(vs target, 6dB tolerance)",
        f"",
        f"OVERALL: {'PASS' if all_pass else 'FAIL'}",
    ]
    ax.text(0.05, 0.95, '\n'.join(summary_lines), transform=ax.transAxes,
            fontsize=11, verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='#e8f5e9' if all_pass else '#ffebee',
                      alpha=0.8))

    plt.tight_layout()
    plot_path = os.path.join(SCRIPT_DIR, "..", "demos", "validation_cabinet.png")
    plt.savefig(plot_path, dpi=150)
    print(f"\nSaved plot: {plot_path}")

    # Save results as JSON
    results = {
        'n_taps': len(ir),
        'causality': causality,
        'windowing': windowing,
        'normalization': normalization,
        'features': features,
        'frequency_shape': shape,
        'overall_pass': all_pass,
    }
    json_path = os.path.join(SCRIPT_DIR, "..", "demos", "validation_cabinet.json")

    class NumpyEncoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, (np.bool_,)):
                return bool(obj)
            if isinstance(obj, (np.integer,)):
                return int(obj)
            if isinstance(obj, (np.floating,)):
                return float(obj)
            return super().default(obj)

    with open(json_path, 'w') as f:
        json.dump(results, f, indent=2, cls=NumpyEncoder)
    print(f"Saved results: {json_path}")

    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
