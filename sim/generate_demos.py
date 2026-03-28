"""
generate_demos.py — Generate interesting demo audio for the dashboard.
Creates chord progressions, sweeps, riffs, and extreme settings.
"""

import numpy as np
import wave
import os
import json

FS = 48000.0
VB, RP, RG, RK, CIN = 200.0, 100000.0, 1000000.0, 1500.0, 22e-9

TUBES = {
    "12AX7": dict(mu=100.0, ex=1.4, kg1=1060.0, kp=600.0, kvb=300.0),
    "12AU7": dict(mu=21.5, ex=1.35, kg1=1180.0, kp=84.0, kvb=300.0),
    "6SL7": dict(mu=70.0, ex=1.4, kg1=1060.0, kp=600.0, kvb=300.0),
    "EL34": dict(mu=11.0, ex=1.35, kg1=650.0, kp=60.0, kvb=24.0),
    "6L6": dict(mu=8.7, ex=1.35, kg1=1460.0, kp=48.0, kvb=12.0),
}


def koren_ip(vpk, vgk, p):
    vpk = float(np.clip(vpk, 0.0, 500.0))
    vgk = float(np.clip(vgk, -10.0, 1.0))
    if vpk <= 0:
        return 0.0
    inner = p["kp"] * (1.0 / p["mu"] + vgk / np.sqrt(p["kvb"] + vpk * vpk))
    inner = np.clip(inner, -500, 500)
    Ed = (vpk / p["kp"]) * np.log1p(np.exp(float(inner)))
    if Ed <= 0:
        return 0.0
    return (Ed ** p["ex"]) / p["kg1"]


def find_dc_op(p):
    lo, hi = 0.0, VB / (RP + RK)
    for _ in range(100):
        mid = (lo + hi) / 2
        r = mid - koren_ip(VB - (RP + RK) * mid, -RK * mid, p)
        if r < 0:
            lo = mid
        else:
            hi = mid
    return mid


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
        ip = ip_prev
        for _ in range(5):
            vpk = VB - (RP + RK) * ip
            vgk = hp_y - RK * ip
            ip_m = koren_ip(vpk, vgk, p)
            f = ip - ip_m
            if abs(f) < 1e-9:
                break
            h = max(abs(ip) * 1e-6, 1e-9)
            ip_mp = koren_ip(VB - (RP + RK) * (ip + h), hp_y - RK * (ip + h), p)
            fp = 1.0 - (ip_mp - ip_m) / h
            if abs(fp) < 1e-15:
                break
            ip = max(ip - f / fp, 0.0)
        vp = VB - RP * ip
        ip_prev = ip
        out[i] = vp - vp_dc
    return out[settle:]


def to_wav(filename, samples):
    peak = np.max(np.abs(samples))
    if peak > 0:
        samples = samples / peak * 0.9
    int_samples = np.clip(samples * 32767, -32768, 32767).astype(np.int16)
    with wave.open(filename, "w") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(int(FS))
        wf.writeframes(int_samples.tobytes())


def note_freq(name):
    """Convert note name to frequency. E.g. 'E2' -> 82.41"""
    notes = {'C':0,'D':2,'E':4,'F':5,'G':7,'A':9,'B':11}
    n = notes[name[0].upper()]
    i = 1
    if len(name) > 1 and name[1] == '#':
        n += 1
        i = 2
    elif len(name) > 1 and name[1] == 'b':
        n -= 1
        i = 2
    octave = int(name[i:])
    midi = n + 12 * (octave + 1)
    return 440.0 * 2 ** ((midi - 69) / 12.0)


def make_chord(freqs, duration, amplitude=0.15):
    """Make a chord from a list of frequencies."""
    t = np.arange(int(FS * duration)) / FS
    sig = np.zeros_like(t)
    for f in freqs:
        sig += amplitude * np.sin(2 * np.pi * f * t)
    return sig


def make_note(freq, duration, amplitude=0.3, attack=0.01, release=0.05):
    """Single note with attack/release envelope."""
    t = np.arange(int(FS * duration)) / FS
    env = np.ones_like(t)
    att_samples = int(attack * FS)
    rel_samples = int(release * FS)
    if att_samples > 0:
        env[:att_samples] = np.linspace(0, 1, att_samples)
    if rel_samples > 0:
        env[-rel_samples:] = np.linspace(1, 0, rel_samples)
    return amplitude * np.sin(2 * np.pi * freq * t) * env


os.makedirs("demos", exist_ok=True)

print("=== Generating Demo Audio ===\n")

# ─── 1. Blues Riff in E ─────────────────────────────────────────────────────
print("1. Blues riff in E...")
E2, A2, B2 = note_freq('E2'), note_freq('A2'), note_freq('B2')
E3, G3, A3 = note_freq('E3'), note_freq('G3'), note_freq('A3')

beat = 0.4  # seconds per beat
riff = np.concatenate([
    make_note(E2, beat*2, 0.4),           # E low, long
    make_note(G3, beat, 0.3),             # G
    make_note(A3, beat, 0.3),             # A
    make_note(E3, beat*2, 0.35),          # E octave up
    make_note(G3, beat, 0.3),             # G
    make_note(E2, beat, 0.4),             # back to E low
    # repeat
    make_note(A2, beat*2, 0.4),           # A
    make_note(G3, beat, 0.3),
    make_note(A3, beat, 0.3),
    make_note(E3, beat*2, 0.35),
    make_note(G3, beat, 0.3),
    make_note(E2, beat, 0.4),
])

to_wav("demos/dry_blues_riff.wav", riff)
wet = simulate(riff, TUBES["12AX7"])
to_wav("demos/wet_blues_riff_12AX7.wav", wet)
wet2 = simulate(riff, TUBES["EL34"])
to_wav("demos/wet_blues_riff_EL34.wav", wet2)
print(f"   12AX7: {20*np.log10(np.sqrt(np.mean(wet**2))/(np.sqrt(np.mean(riff**2))+1e-12)):.1f}dB")

# ─── 2. Chord Progression: E - A - B - E ────────────────────────────────────
print("2. Chord progression E-A-B-E...")
chord_dur = 1.5
progression = np.concatenate([
    make_chord([note_freq('E2'), note_freq('B2'), note_freq('E3'), note_freq('G#3'), note_freq('B3')], chord_dur, 0.12),
    make_chord([note_freq('A2'), note_freq('E3'), note_freq('A3'), note_freq('C#4')], chord_dur, 0.12),
    make_chord([note_freq('B2'), note_freq('F#3'), note_freq('B3'), note_freq('D#4')], chord_dur, 0.12),
    make_chord([note_freq('E2'), note_freq('B2'), note_freq('E3'), note_freq('G#3'), note_freq('B3')], chord_dur, 0.12),
])
to_wav("demos/dry_chord_progression.wav", progression)
wet = simulate(progression, TUBES["12AX7"])
to_wav("demos/wet_chord_progression_12AX7.wav", wet)
wet2 = simulate(progression * 3, TUBES["12AX7"])  # cranked
to_wav("demos/wet_chord_progression_cranked.wav", wet2)
print("   Clean and cranked versions done")

# ─── 3. Clean to Full Crunch Sweep ──────────────────────────────────────────
print("3. Drive sweep: clean -> full crunch...")
dur = 5.0
t = np.arange(int(FS * dur)) / FS
drive_env = np.linspace(0.05, 5.0, len(t))
sweep = drive_env * np.sin(2 * np.pi * 330 * t)
to_wav("demos/dry_drive_sweep.wav", sweep)
wet = simulate(sweep, TUBES["12AX7"])
to_wav("demos/wet_drive_sweep.wav", wet)
print("   0.05V -> 5.0V at 330Hz through 12AX7")

# ─── 4. Frequency Sweep (low to high) ───────────────────────────────────────
print("4. Frequency sweep 80Hz -> 2kHz...")
dur = 4.0
t = np.arange(int(FS * dur)) / FS
freq_sweep = 80 * (2000/80) ** (t / dur)  # exponential sweep
phase = 2 * np.pi * 80 * dur / np.log(2000/80) * (np.exp(t / dur * np.log(2000/80)) - 1)
sig = 0.5 * np.sin(phase)
to_wav("demos/dry_freq_sweep.wav", sig)
wet = simulate(sig, TUBES["12AX7"])
to_wav("demos/wet_freq_sweep.wav", wet)
print("   Exponential chirp through 12AX7")

# ─── 5. Same Riff Through All 5 Tubes ───────────────────────────────────────
print("5. Same riff through all tubes...")
test_riff = np.concatenate([
    make_note(note_freq('E2'), 0.5, 0.5),
    make_note(note_freq('G3'), 0.3, 0.4),
    make_note(note_freq('A3'), 0.3, 0.4),
    make_note(note_freq('E3'), 0.5, 0.45),
    make_note(note_freq('B2'), 0.4, 0.45),
])
to_wav("demos/dry_riff_comparison.wav", test_riff)
for name, params in TUBES.items():
    wet = simulate(test_riff, params)
    to_wav(f"demos/wet_riff_{name}.wav", wet)
    g = 20*np.log10(np.sqrt(np.mean(wet**2))/(np.sqrt(np.mean(test_riff**2))+1e-12))
    print(f"   {name}: {g:.1f}dB gain")

# ─── 6. Staccato Hits (transient response) ──────────────────────────────────
print("6. Staccato hits (transient response)...")
hit = make_note(note_freq('E2'), 0.08, 0.8, attack=0.002, release=0.03)
gap = np.zeros(int(FS * 0.15))
hits = np.concatenate([np.concatenate([hit, gap]) for _ in range(16)])
to_wav("demos/dry_staccato.wav", hits)
wet = simulate(hits, TUBES["12AX7"])
to_wav("demos/wet_staccato.wav", wet)
print("   16 palm-muted style hits through 12AX7")

# ─── 7. Double Stop Bends ───────────────────────────────────────────────────
print("7. Double stop with bend...")
dur = 3.0
t = np.arange(int(FS * dur)) / FS
base = note_freq('B3')
bend_target = note_freq('C4')
bend = np.where(t < 1.0, base, base + (bend_target - base) * np.minimum((t - 1.0) / 0.5, 1.0))
sig = 0.4 * np.sin(2 * np.pi * np.cumsum(bend) / FS) + 0.35 * np.sin(2 * np.pi * note_freq('E3') * t)
to_wav("demos/dry_doublestop.wav", sig)
wet = simulate(sig, TUBES["12AX7"])
to_wav("demos/wet_doublestop.wav", wet)
print("   B+E with bend to C through 12AX7")

print(f"\n=== Done! Generated {len(os.listdir('demos'))} files in demos/ ===")
print("\nBest ones to listen to:")
print("  demos/wet_blues_riff_12AX7.wav  — blues riff, classic tube overdrive")
print("  demos/wet_drive_sweep.wav       — hear clean morph into crunch")
print("  demos/wet_chord_progression_cranked.wav — power chords cranked")
print("  demos/wet_staccato.wav          — palm mute transient response")
