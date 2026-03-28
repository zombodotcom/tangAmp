r"""
generate_all_demos.py
Comprehensive demo audio generation for tangAmp simulator.

Generates WAV files for all 5 amp presets across multiple signal types,
comparison demos, A/B dry/wet pairs, and a gain sweep for Marshall.
"""

import numpy as np
import os
import json
import sys
import time

# Ensure we can import from the project directory
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from amp_sim import AmpSim, save_wav, FS

# =============================================================================
# Config
# =============================================================================

PRESETS = ['fender_deluxe', 'marshall_jcm800', 'vox_ac30', 'mesa_dual_rec', 'fender_twin']
N_SETTLE = 2000
DEMOS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demos")


# =============================================================================
# Signal generators (all return n_settle + n_audio samples)
# =============================================================================

def make_chord(duration=3.0):
    """E power chord: 82 + 123 + 165 Hz at 0.15V each."""
    n_audio = int(duration * FS)
    n_total = N_SETTLE + n_audio
    t = np.arange(n_total) / FS
    sig = np.zeros(n_total)
    sig[N_SETTLE:] = (0.15 * np.sin(2 * np.pi * 82.0 * t[N_SETTLE:]) +
                      0.15 * np.sin(2 * np.pi * 123.0 * t[N_SETTLE:]) +
                      0.15 * np.sin(2 * np.pi * 165.0 * t[N_SETTLE:]))
    return sig, n_audio


def make_clean_note(duration=2.0):
    """Clean single note 440 Hz at 0.3V."""
    n_audio = int(duration * FS)
    n_total = N_SETTLE + n_audio
    t = np.arange(n_total) / FS
    sig = np.zeros(n_total)
    sig[N_SETTLE:] = 0.3 * np.sin(2 * np.pi * 440.0 * t[N_SETTLE:])
    return sig, n_audio


def make_dirty_note(duration=2.0):
    """Dirty single note 440 Hz at 2.0V."""
    n_audio = int(duration * FS)
    n_total = N_SETTLE + n_audio
    t = np.arange(n_total) / FS
    sig = np.zeros(n_total)
    sig[N_SETTLE:] = 2.0 * np.sin(2 * np.pi * 440.0 * t[N_SETTLE:])
    return sig, n_audio


def make_volume_swell(duration=3.0):
    """Volume swell 330 Hz from 0.05V to 3V."""
    n_audio = int(duration * FS)
    n_total = N_SETTLE + n_audio
    t = np.arange(n_total) / FS
    sig = np.zeros(n_total)
    t_audio = t[N_SETTLE:] - t[N_SETTLE]
    envelope = np.linspace(0.05, 3.0, n_audio)
    sig[N_SETTLE:] = envelope * np.sin(2 * np.pi * 330.0 * t[N_SETTLE:])
    return sig, n_audio


# Signal registry: (name, generator, description)
SIGNALS = [
    ('chord',        make_chord,        'E power chord (82+123+165 Hz, 0.15V each, 3s)'),
    ('clean',        make_clean_note,   'Clean single note (440 Hz, 0.3V, 2s)'),
    ('dirty',        make_dirty_note,   'Dirty single note (440 Hz, 2.0V, 2s)'),
    ('volume_swell', make_volume_swell, 'Volume swell (330 Hz, 0.05V->3V, 3s)'),
]


# =============================================================================
# Helpers
# =============================================================================

def save_dry(filename, signal_with_settle, n_audio):
    """Save the dry (input) signal as WAV, trimming settle samples."""
    dry = signal_with_settle[N_SETTLE:N_SETTLE + n_audio]
    save_wav(filename, dry, int(FS))


def process_and_save(amp, signal_with_settle, wav_path, skip_existing=True):
    """Process through amp and save. Returns the output signal or None if skipped."""
    if skip_existing and os.path.exists(wav_path) and os.path.getsize(wav_path) > 100:
        print(f"[exists, skip]", end=" ")
        return None
    t0 = time.time()
    out = amp.process(signal_with_settle, settle=N_SETTLE)
    elapsed = time.time() - t0
    save_wav(wav_path, out, int(FS))
    print(f"({elapsed:.0f}s)", end=" ")
    return out


# =============================================================================
# Main
# =============================================================================

def main():
    os.makedirs(DEMOS_DIR, exist_ok=True)

    manifest = []
    file_count = 0

    # Pre-generate all input signals
    signal_cache = {}
    for sig_name, sig_fn, sig_desc in SIGNALS:
        signal_cache[sig_name] = sig_fn()

    # -----------------------------------------------------------------
    # 1. Per-preset demos (chord, clean, dirty, volume_swell)
    # -----------------------------------------------------------------
    print("=" * 70)
    print("PART 1: Per-preset signal demos")
    print("=" * 70)

    for preset in PRESETS:
        amp = AmpSim(preset)
        print(f"\n--- {amp.name} ({preset}) ---")

        for sig_name, _, sig_desc in SIGNALS:
            sig, n_audio = signal_cache[sig_name]
            fname = f"{preset}_{sig_name}.wav"
            wav_path = os.path.join(DEMOS_DIR, fname)

            print(f"  {sig_name}...", end=" ", flush=True)
            out = process_and_save(amp, sig, wav_path)
            if out is not None:
                peak = np.max(np.abs(out))
                rms = np.sqrt(np.mean(out ** 2))
                print(f"peak={peak:.1f}V rms={rms:.1f}V")
            else:
                print()

            manifest.append({
                "file": fname,
                "preset": preset,
                "amp_name": amp.name,
                "signal": sig_name,
                "signal_desc": sig_desc,
                "gain": amp.input_gain,
                "master": amp.master_vol,
                "category": "preset_demo",
            })
            file_count += 1

    # -----------------------------------------------------------------
    # 2a. Comparison: same chord through all 5 amps (already generated
    #     above as {preset}_chord.wav, but we list them under comparison)
    # -----------------------------------------------------------------
    # The chord files from Part 1 serve double duty. No extra files needed.

    # -----------------------------------------------------------------
    # 2b. Marshall gain sweep: gain 1, 3, 5, 7, 10
    # -----------------------------------------------------------------
    print("\n" + "=" * 70)
    print("PART 2: Marshall JCM800 gain sweep")
    print("=" * 70)

    gain_levels = [1, 3, 5, 7, 10]
    chord_sig, chord_n = signal_cache['chord']

    for gain in gain_levels:
        amp = AmpSim('marshall_jcm800')
        amp.set_gain(gain)
        amp.set_master(5)

        fname = f"marshall_jcm800_gain{gain}.wav"
        wav_path = os.path.join(DEMOS_DIR, fname)

        print(f"  gain={gain}...", end=" ", flush=True)
        out = process_and_save(amp, chord_sig, wav_path)
        if out is not None:
            peak = np.max(np.abs(out))
            rms = np.sqrt(np.mean(out ** 2))
            print(f"peak={peak:.1f}V rms={rms:.1f}V")
        else:
            print()

        manifest.append({
            "file": fname,
            "preset": "marshall_jcm800",
            "amp_name": "Marshall JCM800",
            "signal": "chord",
            "signal_desc": "E power chord (82+123+165 Hz, gain sweep)",
            "gain": gain,
            "master": 5,
            "category": "gain_sweep",
        })
        file_count += 1

    # -----------------------------------------------------------------
    # 3. A/B demos: dry + wet for each preset (using chord signal)
    # -----------------------------------------------------------------
    print("\n" + "=" * 70)
    print("PART 3: A/B dry/wet demos")
    print("=" * 70)

    for preset in PRESETS:
        amp = AmpSim(preset)

        # Dry
        dry_fname = f"{preset}_dry.wav"
        dry_path = os.path.join(DEMOS_DIR, dry_fname)
        save_dry(dry_path, chord_sig, chord_n)
        manifest.append({
            "file": dry_fname,
            "preset": preset,
            "amp_name": amp.name,
            "signal": "chord",
            "signal_desc": "E power chord - DRY input",
            "gain": 0,
            "master": 0,
            "category": "ab_dry",
        })
        file_count += 1

        # Wet (re-use the already-generated chord file path, but save as _wet)
        wet_fname = f"{preset}_wet.wav"
        wet_path = os.path.join(DEMOS_DIR, wet_fname)
        print(f"  {preset} wet...", end=" ", flush=True)
        out = process_and_save(amp, chord_sig, wet_path)
        if out is not None:
            peak = np.max(np.abs(out))
            print(f"peak={peak:.1f}V")
        else:
            print()
        manifest.append({
            "file": wet_fname,
            "preset": preset,
            "amp_name": amp.name,
            "signal": "chord",
            "signal_desc": "E power chord - WET output",
            "gain": amp.input_gain,
            "master": amp.master_vol,
            "category": "ab_wet",
        })
        file_count += 1

    # -----------------------------------------------------------------
    # 5. Write manifest JSON
    # -----------------------------------------------------------------
    manifest_path = os.path.join(DEMOS_DIR, "demo_manifest.json")
    with open(manifest_path, "w") as f:
        json.dump({
            "generated_by": "generate_all_demos.py",
            "sample_rate": int(FS),
            "bit_depth": 16,
            "channels": 1,
            "total_files": file_count,
            "files": manifest,
        }, f, indent=2)
    print(f"\nManifest saved: {manifest_path}")

    # -----------------------------------------------------------------
    # Summary
    # -----------------------------------------------------------------
    print("\n" + "=" * 70)
    print(f"TOTAL FILES GENERATED: {file_count}")
    print("=" * 70)


if __name__ == "__main__":
    main()
