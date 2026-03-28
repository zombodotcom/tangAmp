"""
test_triode.py
Comprehensive test suite for the 12AX7 triode simulation.

Tests:
  1. WAV output — hear the tube distortion on test tones and guitar-like signals
  2. Gain sweep — how gain changes with input level (compression curve)
  3. THD analysis — harmonic distortion at different drive levels
  4. Tube comparison — 12AX7 vs 12AU7 vs 6SL7 vs EL34 vs 6L6
"""

import numpy as np
import wave
import struct
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

FS = 48000.0
VB = 200.0
RP = 100000.0
RG = 1000000.0
RK = 1500.0
CIN = 22e-9

# ═══════════════════════════════════════════════════════════════════════════════
# Tube Definitions
# ═══════════════════════════════════════════════════════════════════════════════

TUBES = {
    "12AX7": dict(mu=100.0, ex=1.4,  kg1=1060.0, kp=600.0, kvb=300.0, desc="High gain preamp (Marshall/Fender)"),
    "12AU7": dict(mu=21.5,  ex=1.35, kg1=1180.0, kp=84.0,  kvb=300.0, desc="Low gain preamp (clean/hifi)"),
    "6SL7":  dict(mu=70.0,  ex=1.4,  kg1=1060.0, kp=600.0, kvb=300.0, desc="Medium gain (vintage)"),
    "EL34":  dict(mu=11.0,  ex=1.35, kg1=650.0,  kp=60.0,  kvb=24.0,  desc="Power tube (Marshall crunch)"),
    "6L6":   dict(mu=8.7,   ex=1.35, kg1=1460.0, kp=48.0,  kvb=12.0,  desc="Power tube (Fender clean)"),
}

def koren_ip(vpk, vgk, p):
    vpk = float(np.clip(vpk, 0.0, 500.0))
    vgk = float(np.clip(vgk, -10.0, 1.0))
    if vpk <= 0: return 0.0
    inner = p['kp'] * (1.0/p['mu'] + vgk / np.sqrt(p['kvb'] + vpk**2))
    inner = np.clip(inner, -500, 500)
    Ed = (vpk / p['kp']) * np.log1p(np.exp(float(inner)))
    if Ed <= 0: return 0.0
    return (Ed ** p['ex']) / p['kg1']

# ═══════════════════════════════════════════════════════════════════════════════
# Core Simulation
# ═══════════════════════════════════════════════════════════════════════════════

def find_dc_op(p):
    lo, hi = 0.0, VB / (RP + RK)
    for _ in range(100):
        mid = (lo + hi) / 2
        r = mid - koren_ip(VB - (RP+RK)*mid, -RK*mid, p)
        if r < 0: lo = mid
        else: hi = mid
    return mid  # Ip

def simulate(audio_in, p, settle=500):
    tau = RG * CIN
    k = 2.0 * FS * tau
    c_hp = (k - 1.0) / (k + 1.0)
    hp_g = (1.0 + c_hp) / 2.0

    ip_dc = find_dc_op(p)
    vp_dc = VB - RP * ip_dc

    n = len(audio_in) + settle
    full_in = np.concatenate([np.zeros(settle), audio_in])

    hp_y, hp_x_prev, ip_prev = 0.0, 0.0, ip_dc
    out = np.zeros(n)

    for i in range(n):
        vin = full_in[i]
        hp_y = c_hp * hp_y + hp_g * (vin - hp_x_prev)
        hp_x_prev = vin
        v_grid = hp_y

        ip = ip_prev
        for _ in range(5):
            vpk = VB - (RP+RK)*ip
            vgk = v_grid - RK*ip
            ip_m = koren_ip(vpk, vgk, p)
            f = ip - ip_m
            if abs(f) < 1e-9: break
            h = max(abs(ip)*1e-6, 1e-9)
            ip_mp = koren_ip(VB-(RP+RK)*(ip+h), v_grid-RK*(ip+h), p)
            fp = 1.0 - (ip_mp - ip_m)/h
            if abs(fp) < 1e-15: break
            ip = max(ip - f/fp, 0.0)

        vp = VB - RP * ip
        ip_prev = ip
        out[i] = vp - vp_dc

    return out[settle:]  # trim settling

def to_wav(filename, samples, sample_rate=48000):
    """Write float samples to 16-bit WAV."""
    peak = np.max(np.abs(samples))
    if peak > 0:
        samples = samples / peak * 0.9  # normalize to 90%
    int_samples = np.clip(samples * 32767, -32768, 32767).astype(np.int16)
    with wave.open(filename, 'w') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(int_samples.tobytes())

# ═══════════════════════════════════════════════════════════════════════════════
# TEST 1: WAV Output — Hear the tube
# ═══════════════════════════════════════════════════════════════════════════════

print("=" * 60)
print("TEST 1: WAV Output — Audible tube distortion")
print("=" * 60)

p = TUBES["12AX7"]
duration = 3.0  # seconds
t = np.arange(int(FS * duration)) / FS

# Clean sine
sine_440 = 0.5 * np.sin(2*np.pi*440*t)

# Chord (E power chord: E2=82Hz + B2=123Hz + E3=165Hz)
chord = 0.15 * (np.sin(2*np.pi*82.4*t) + np.sin(2*np.pi*123.5*t) + np.sin(2*np.pi*164.8*t))

# Dirty sine (overdriven)
dirty = 2.0 * np.sin(2*np.pi*440*t)

# Volume swell (crescendo from clean to crunch)
swell_env = np.linspace(0.05, 3.0, len(t))
swell = swell_env * np.sin(2*np.pi*330*t)

signals = {
    "clean_sine": (sine_440, "440Hz sine at 0.5V (clean)"),
    "power_chord": (chord, "E power chord (3 notes, moderate drive)"),
    "dirty_sine": (dirty, "440Hz sine at 2V (overdriven)"),
    "volume_swell": (swell, "330Hz with volume swell (clean to crunch)"),
}

for name, (sig, desc) in signals.items():
    print(f"  Processing {name}: {desc}")
    processed = simulate(sig, p)

    # Save dry (input) and wet (processed)
    to_wav(f"wav_dry_{name}.wav", sig)
    to_wav(f"wav_wet_{name}.wav", processed)

print(f"  Saved {len(signals)*2} WAV files (wav_dry_*.wav / wav_wet_*.wav)")
print(f"  Compare dry vs wet to hear the tube character!\n")

# ═══════════════════════════════════════════════════════════════════════════════
# TEST 2: Gain Sweep — Compression Curve
# ═══════════════════════════════════════════════════════════════════════════════

print("=" * 60)
print("TEST 2: Gain Sweep — Tube compression curve")
print("=" * 60)

input_levels = np.logspace(-2, 0.7, 30)  # 0.01V to 5V
gains_db = []
thds = []

for level in input_levels:
    sig = level * np.sin(2*np.pi*440*np.arange(int(FS*0.1))/FS)
    out = simulate(sig, p, settle=200)

    in_rms = np.sqrt(np.mean(sig**2))
    out_rms = np.sqrt(np.mean(out**2))
    gain = 20*np.log10(out_rms / in_rms + 1e-12) if in_rms > 0 else 0
    gains_db.append(gain)

    # THD: ratio of harmonics to fundamental
    N = len(out)
    spectrum = np.abs(np.fft.rfft(out * np.hanning(N)))
    freqs = np.fft.rfftfreq(N, 1/FS)
    fund_idx = np.argmin(np.abs(freqs - 440))
    fund_power = spectrum[fund_idx]**2

    harm_power = 0
    for h in range(2, 8):
        h_idx = np.argmin(np.abs(freqs - 440*h))
        harm_power += spectrum[h_idx]**2

    thd = np.sqrt(harm_power / (fund_power + 1e-12)) * 100
    thds.append(thd)

    print(f"  Vin={level:.3f}V  Gain={gain:.1f}dB  THD={thd:.1f}%")

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
fig.suptitle("12AX7 Tube Compression & Distortion", fontsize=13)

ax1.semilogx(input_levels, gains_db, 'r-o', markersize=4)
ax1.set_xlabel("Input Level (V peak)")
ax1.set_ylabel("Gain (dB)")
ax1.set_title("Gain vs Input — Compression visible at high drive")
ax1.grid(True, alpha=0.3)
ax1.axhline(gains_db[0], color='gray', ls='--', alpha=0.5, label=f'Small-signal: {gains_db[0]:.1f}dB')
ax1.legend()

ax2.semilogx(input_levels, thds, 'b-o', markersize=4)
ax2.set_xlabel("Input Level (V peak)")
ax2.set_ylabel("THD (%)")
ax2.set_title("THD vs Input — Distortion rises with drive")
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("test_gain_sweep.png", dpi=150)
print(f"\n  Saved: test_gain_sweep.png\n")

# ═══════════════════════════════════════════════════════════════════════════════
# TEST 3: Tube Comparison
# ═══════════════════════════════════════════════════════════════════════════════

print("=" * 60)
print("TEST 3: Tube Type Comparison")
print("=" * 60)

test_sig = 0.5 * np.sin(2*np.pi*440*np.arange(int(FS*0.5))/FS)

fig, axes = plt.subplots(len(TUBES), 2, figsize=(14, 3*len(TUBES)))
fig.suptitle("Tube Comparison — Same 440Hz Input at 0.5V", fontsize=14, y=1.01)

tube_results = {}
for i, (name, params) in enumerate(TUBES.items()):
    print(f"  {name}: {params['desc']}")
    ip_dc = find_dc_op(params)
    vp_dc = VB - RP * ip_dc

    out = simulate(test_sig, params)
    tube_results[name] = out

    out_rms = np.sqrt(np.mean(out**2))
    in_rms = np.sqrt(np.mean(test_sig**2))
    gain = 20*np.log10(out_rms/in_rms + 1e-12)

    # Time domain
    t_plot = np.arange(len(out))[:480] / FS * 1000  # ~10ms
    axes[i, 0].plot(t_plot, out[:480], 'r', linewidth=1.2)
    axes[i, 0].set_ylabel(f"{name}\n(V)")
    axes[i, 0].set_title(f"{name} — {params['desc']} (gain={gain:.1f}dB, Ip_dc={ip_dc*1000:.2f}mA)")
    axes[i, 0].grid(True, alpha=0.3)

    # FFT
    N = len(out)
    sp = np.abs(np.fft.rfft(out * np.hanning(N)))
    sp_db = 20*np.log10(sp/(max(sp)+1e-12)+1e-12)
    f = np.fft.rfftfreq(N, 1/FS)
    axes[i, 1].plot(f, sp_db, 'r', linewidth=0.8)
    axes[i, 1].set_xlim(0, 5000); axes[i, 1].set_ylim(-80, 5)
    axes[i, 1].set_title("Harmonics")
    axes[i, 1].grid(True, alpha=0.3)
    for h in range(1, 8):
        axes[i, 1].axvline(440*h, color='gray', alpha=0.2, ls='--', lw=0.5)

    # WAV for each tube
    to_wav(f"wav_tube_{name}.wav", out)
    print(f"    Gain={gain:.1f}dB  Ip_dc={ip_dc*1000:.2f}mA  Vp_dc={vp_dc:.0f}V")

axes[-1, 0].set_xlabel("Time (ms)")
axes[-1, 1].set_xlabel("Frequency (Hz)")

plt.tight_layout()
plt.savefig("test_tube_comparison.png", dpi=150, bbox_inches='tight')
print(f"\n  Saved: test_tube_comparison.png")
print(f"  Saved: wav_tube_*.wav for each tube type\n")

# ═══════════════════════════════════════════════════════════════════════════════
# Summary
# ═══════════════════════════════════════════════════════════════════════════════

print("=" * 60)
print("ALL TESTS COMPLETE")
print("=" * 60)
print("\nWAV files to listen to:")
print("  wav_dry_*.wav  — dry input signals")
print("  wav_wet_*.wav  — through 12AX7 tube stage")
print("  wav_tube_*.wav — same signal through different tubes")
print("\nPlots:")
print("  test_gain_sweep.png     — compression & THD curves")
print("  test_tube_comparison.png — waveform & harmonics per tube")
print("\nPlay the WAV files to hear the difference!")
