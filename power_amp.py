r"""
power_amp.py
Push-pull power amp simulation using Koren tube model.

Models:
  - Single triode stage with power tube constants (6L6, EL34, 300B)
  - Higher plate voltage (400V) and lower plate resistance (transformer primary)
  - Transformer saturation via tanh soft-clipping
  - Power supply sag: dynamic compression from supply droop under load
"""

import numpy as np
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

try:
    from scipy.io import wavfile
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

# =============================================================================
# Tube Constants
# =============================================================================

TUBES = {
    "12AX7": dict(mu=100.0, ex=1.4, kg1=1060.0, kp=600.0, kvb=300.0),
    "12AU7": dict(mu=27.48, ex=1.03, kg1=466.13, kp=135.10, kvb=24224.55),
    "6SL7":  dict(mu=90.41, ex=1.25, kg1=597.32, kp=511.97, kvb=6747.79),
    "EL34":  dict(mu=10.98, ex=1.42, kg1=249.65, kp=43.2,   kvb=333.0),
    "6L6":   dict(mu=10.11, ex=1.37, kg1=406.6,  kp=31.2,   kvb=640.7),
    "300B":  dict(mu=3.95,  ex=1.4,  kg1=1550.0, kp=65.0,   kvb=300.0),
}

FS = 48000.0


# =============================================================================
# Koren Model (parameterized)
# =============================================================================

def koren_ip(vpk, vgk, mu, ex, kg1, kp, kvb):
    """Plate current Ip in amps."""
    vpk = float(np.clip(vpk, 0.0, 600.0))
    vgk = float(np.clip(vgk, -50.0, 5.0))
    if vpk <= 0:
        return 0.0
    inner = kp * (1.0 / mu + vgk / np.sqrt(kvb + vpk * vpk))
    inner = np.clip(inner, -500, 500)
    Ed = (vpk / kp) * np.log1p(np.exp(float(inner)))
    if Ed <= 0:
        return 0.0
    return (Ed ** ex) / kg1


def koren_dip_dvpk(vpk, vgk, tube, h=0.1):
    return (koren_ip(vpk + h, vgk, **tube) - koren_ip(vpk - h, vgk, **tube)) / (2 * h)


def koren_dip_dvgk(vpk, vgk, tube, h=0.1):
    return (koren_ip(vpk, vgk + h, **tube) - koren_ip(vpk, vgk - h, **tube)) / (2 * h)


# =============================================================================
# Power Amp Stage
# =============================================================================

def simulate_power_amp(audio_in, tube_name='6L6', vb=400.0, rp=2000.0, rk=250.0,
                       rg=1e6, cin=22e-9, sag_amount=0.3, clip_level=200.0,
                       fs=48000.0, settle=2000):
    """
    Power amp simulation using WDF triode stage with power tube constants.

    Parameters
    ----------
    audio_in : ndarray
        Input voltage signal (including settle samples).
    tube_name : str
        Tube type key from TUBES dict.
    vb : float
        B+ supply voltage (400V typical for power amp).
    rp : float
        Plate load resistance (transformer primary, ~2k ohm).
    rk : float
        Cathode resistance (250 ohm typical).
    rg : float
        Grid leak resistance.
    cin : float
        Input coupling capacitor.
    sag_amount : float
        Sag depth (0=none, 1=full). Controls dynamic compression.
    clip_level : float
        Transformer saturation threshold in volts.
    fs : float
        Sample rate.
    settle : int
        Number of settle samples at start.

    Returns
    -------
    out_vplate : ndarray
        Plate voltage output.
    vp_dc : float
        DC quiescent plate voltage.
    """
    tube = TUBES[tube_name]
    n_total = len(audio_in)
    out_vplate = np.zeros(n_total)

    # High-pass coupling filter
    tau_hp = rg * cin
    k_hp = 2.0 * fs * tau_hp
    c_hp = (k_hp - 1.0) / (k_hp + 1.0)
    hp_gain = (1.0 + c_hp) / 2.0

    # Sag filter state (low-pass on rectified output level)
    sag_tau = 0.05  # 50ms time constant
    sag_alpha = 1.0 / (1.0 + 2.0 * fs * sag_tau)
    sag_level = 0.0

    # Newton-Raphson state
    prev_ip = 1.0e-3  # power tubes draw more current
    hp_y = 0.0
    hp_x_prev = 0.0

    R_p = rp
    R_k = rk

    for n in range(n_total):
        vin = audio_in[n]

        # High-pass filter (input coupling)
        hp_y = c_hp * hp_y + hp_gain * (vin - hp_x_prev)
        hp_x_prev = vin
        v_grid = hp_y

        # Sag: reduce supply voltage based on average output level
        vb_eff = vb - sag_amount * sag_level * 100.0

        # Upward pass (simple: no cathode bypass cap for power amp)
        a_p = vb_eff
        a_g = v_grid
        a_k = 0.0

        # Newton-Raphson at root
        Ip = prev_ip
        for iteration in range(15):
            Vpk = (a_p - a_k) - (R_p + R_k) * Ip
            Vgk = a_g - a_k - R_k * Ip

            ip_model = koren_ip(Vpk, Vgk, **tube)
            f_val = Ip - ip_model

            if abs(f_val) < 1e-9:
                break

            dip_dvpk = koren_dip_dvpk(Vpk, Vgk, tube)
            dip_dvgk = koren_dip_dvgk(Vpk, Vgk, tube)
            df_dIp = 1.0 + dip_dvpk * (R_p + R_k) + dip_dvgk * R_k

            if abs(df_dIp) < 1e-15:
                break

            Ip -= f_val / df_dIp
            Ip = max(Ip, 0.0)

        prev_ip = Ip
        v_plate = vb_eff - R_p * Ip
        out_vplate[n] = v_plate

        # Update sag filter (track rectified output swing)
        if n >= settle:
            sag_level = sag_level * (1.0 - sag_alpha) + abs(Ip) * sag_alpha

    # DC quiescent point
    sl = slice(max(settle - 100, 0), settle)
    vp_dc = out_vplate[sl].mean()

    # AC-couple
    ac_out = out_vplate - vp_dc

    # Transformer saturation: soft clip with tanh
    ac_out = np.tanh(ac_out / clip_level) * clip_level

    return ac_out, vp_dc


# =============================================================================
# WAV output
# =============================================================================

def save_wav(filename, signal, fs=48000):
    if not HAS_SCIPY:
        print(f"  [scipy not available, skipping {filename}]")
        return
    peak = np.max(np.abs(signal))
    if peak > 1e-12:
        signal = signal / peak * 0.9
    sig16 = np.clip(signal * 32767, -32768, 32767).astype(np.int16)
    wavfile.write(filename, int(fs), sig16)
    print(f"  Saved: {filename}")


# =============================================================================
# Main: Demo power amp with different tubes and drive levels
# =============================================================================

if __name__ == "__main__":
    demos_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demos")
    os.makedirs(demos_dir, exist_ok=True)

    # Input: E power chord, 1 second
    duration = 1.0
    n_settle = 2000
    n_audio = int(duration * FS)
    n_total = n_settle + n_audio
    t = np.arange(n_total) / FS

    audio_in = np.zeros(n_total)
    audio_in[n_settle:] = (0.15 * np.sin(2 * np.pi * 82.0 * t[n_settle:]) +
                           0.15 * np.sin(2 * np.pi * 123.0 * t[n_settle:]) +
                           0.15 * np.sin(2 * np.pi * 165.0 * t[n_settle:]))

    print("=" * 60)
    print("Power Amp Simulation")
    print("=" * 60)

    # Test each power tube
    tube_configs = [
        ("6L6",  400.0, 2000.0, 250.0, 0.3, 200.0, "Fender-style clean power"),
        ("EL34", 450.0, 1700.0, 200.0, 0.2, 180.0, "Marshall-style aggressive"),
        ("300B", 350.0, 2500.0, 300.0, 0.1, 250.0, "Audiophile warm"),
    ]

    results = []

    for tube_name, vb, rp, rk, sag, clip, desc in tube_configs:
        print(f"\n--- {tube_name}: {desc} ---")
        print(f"  VB={vb}V, RP={rp}, RK={rk}, sag={sag}, clip={clip}V")

        ac_out, vp_dc = simulate_power_amp(
            audio_in, tube_name=tube_name, vb=vb, rp=rp, rk=rk,
            sag_amount=sag, clip_level=clip, settle=n_settle)

        audio_part = ac_out[n_settle:]
        peak = np.max(np.abs(audio_part))
        rms = np.sqrt(np.mean(audio_part ** 2))
        print(f"  Vp_dc = {vp_dc:.1f}V")
        print(f"  Output peak = {peak:.1f}V, RMS = {rms:.1f}V")

        wav_path = os.path.join(demos_dir, f"poweramp_{tube_name.lower()}.wav")
        save_wav(wav_path, audio_part, int(FS))

        results.append((tube_name, desc, audio_part, vp_dc, peak))

    # Drive level sweep with 6L6
    print(f"\n{'='*60}")
    print("Drive Level Sweep (6L6)")
    print("=" * 60)

    drive_levels = [0.05, 0.15, 0.5, 1.5, 5.0]
    drive_results = []

    for drive in drive_levels:
        audio_driven = np.zeros(n_total)
        audio_driven[n_settle:] = drive * (
            np.sin(2 * np.pi * 82.0 * t[n_settle:]) +
            np.sin(2 * np.pi * 123.0 * t[n_settle:]) +
            np.sin(2 * np.pi * 165.0 * t[n_settle:]))

        ac_out, vp_dc = simulate_power_amp(
            audio_driven, tube_name='6L6', vb=400.0, rp=2000.0, rk=250.0,
            sag_amount=0.3, clip_level=200.0, settle=n_settle)

        audio_part = ac_out[n_settle:]
        peak = np.max(np.abs(audio_part))
        print(f"  Drive={drive:.2f}V -> peak={peak:.1f}V")
        drive_results.append((drive, audio_part))

    # Plot: tube comparison + drive sweep
    fig, axes = plt.subplots(3, 2, figsize=(16, 14))
    fig.suptitle("Power Amp Simulation", fontsize=14, fontweight='bold')
    t_audio = np.arange(n_audio) / FS
    mask = t_audio < 0.03  # 30ms

    colors = ['#1f77b4', '#d62728', '#2ca02c']
    for i, (tube_name, desc, audio_part, vp_dc, peak) in enumerate(results):
        # Waveform
        ax = axes[i, 0]
        ax.plot(t_audio[mask] * 1000, audio_part[mask], color=colors[i], linewidth=0.8)
        ax.set_ylabel("V")
        ax.set_title(f"{tube_name} -- {desc} (peak={peak:.0f}V)")
        ax.grid(True, alpha=0.3)
        if i == 2:
            ax.set_xlabel("Time (ms)")

        # Spectrum
        ax = axes[i, 1]
        N = len(audio_part)
        w = np.hanning(N)
        sp = np.abs(np.fft.rfft(audio_part * w))
        sp_db = 20 * np.log10(sp / (np.max(sp) + 1e-12) + 1e-12)
        f = np.fft.rfftfreq(N, 1.0 / FS)
        ax.plot(f, sp_db, color=colors[i], linewidth=0.8)
        ax.set_xlim(0, 5000)
        ax.set_ylim(-80, 5)
        ax.set_ylabel("dB")
        ax.set_title(f"{tube_name} -- Spectrum")
        ax.grid(True, alpha=0.3)
        for freq in [82, 123, 165]:
            ax.axvline(freq, color='gray', alpha=0.3, ls='--', lw=0.5)
        if i == 2:
            ax.set_xlabel("Frequency (Hz)")

    plt.tight_layout()
    plot_path = os.path.join(demos_dir, "power_amp.png")
    plt.savefig(plot_path, dpi=150)
    print(f"\nSaved: {plot_path}")
    plt.close()

    print("\nDone.")
