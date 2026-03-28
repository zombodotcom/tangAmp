"""
synth_api.py — Generate dry/wet WAV pair for dashboard live synth.
Called by dashboard.py: python synth_api.py <tube> <freq> <drive> <dur>
Outputs JSON to stdout: {"dry": "file.wav", "wet": "file.wav", "gain": "29.1dB", "thd": "1.2%", "tube": "12AX7"}
"""

import sys
import json
import numpy as np
import wave
import os

# Tube definitions
TUBES = {
    "12AX7": dict(mu=100.0, ex=1.4, kg1=1060.0, kp=600.0, kvb=300.0),
    "12AU7": dict(mu=21.5, ex=1.35, kg1=1180.0, kp=84.0, kvb=300.0),
    "6SL7": dict(mu=70.0, ex=1.4, kg1=1060.0, kp=600.0, kvb=300.0),
    "EL34": dict(mu=11.0, ex=1.35, kg1=650.0, kp=60.0, kvb=24.0),
    "6L6": dict(mu=8.7, ex=1.35, kg1=1460.0, kp=48.0, kvb=12.0),
}

VB, RP, RG, RK, CIN, FS = 200.0, 100000.0, 1000000.0, 1500.0, 22e-9, 48000.0


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


def main():
    tube_name = sys.argv[1] if len(sys.argv) > 1 else "12AX7"
    freq = float(sys.argv[2]) if len(sys.argv) > 2 else 440.0
    drive = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5
    dur = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0

    if tube_name not in TUBES:
        tube_name = "12AX7"

    p = TUBES[tube_name]
    n_samples = int(FS * dur)
    t = np.arange(n_samples) / FS
    sig = drive * np.sin(2 * np.pi * freq * t)

    processed = simulate(sig, p, settle=200)

    # Compute gain and THD
    in_rms = np.sqrt(np.mean(sig ** 2))
    out_rms = np.sqrt(np.mean(processed ** 2))
    gain_db = 20 * np.log10(out_rms / (in_rms + 1e-12)) if in_rms > 0 else 0

    N = len(processed)
    spectrum = np.abs(np.fft.rfft(processed * np.hanning(N)))
    freqs = np.fft.rfftfreq(N, 1 / FS)
    fund_idx = np.argmin(np.abs(freqs - freq))
    fund_power = spectrum[fund_idx] ** 2
    harm_power = sum(spectrum[np.argmin(np.abs(freqs - freq * h))] ** 2 for h in range(2, 8))
    thd = np.sqrt(harm_power / (fund_power + 1e-12)) * 100

    # Save files
    os.makedirs("synth_cache", exist_ok=True)
    dry_file = f"synth_cache/dry_{tube_name}_{int(freq)}hz_{drive}v.wav"
    wet_file = f"synth_cache/wet_{tube_name}_{int(freq)}hz_{drive}v.wav"

    to_wav(dry_file, sig)
    to_wav(wet_file, processed)

    result = {
        "dry": dry_file,
        "wet": wet_file,
        "gain": f"{gain_db:.1f}dB",
        "thd": f"{thd:.1f}%",
        "tube": tube_name,
    }
    print(json.dumps(result))


if __name__ == "__main__":
    main()
