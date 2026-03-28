r"""
tone_stack.py
3-band parametric EQ tone stack using cascaded biquad filters.
Self-contained: duplicates the WDF triode sim core from wdf_triode_sim_v2.py.
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
# Circuit Parameters (duplicated from wdf_triode_sim_v2.py)
# =============================================================================

VB  = 200.0
RP  = 100000.0
RG  = 1000000.0
RK  = 1500.0
CIN = 22e-9
CK  = 22e-6
FS  = 48000.0

MU, EX, KG1, KP, KVB = 100.0, 1.4, 1060.0, 600.0, 300.0

tau_hp = RG * CIN
k_hp = 2.0 * FS * tau_hp
c_hp = (k_hp - 1.0) / (k_hp + 1.0)
hp_gain = (1.0 + c_hp) / 2.0

R_CK = 1.0 / (2.0 * FS * CK)
G_rk = 1.0 / RK
G_ck = 1.0 / R_CK
G_total = G_rk + G_ck
R_cathode_bypass = 1.0 / G_total
gamma_k = G_rk / G_total


# =============================================================================
# Koren Model
# =============================================================================

def koren_ip(vpk, vgk):
    vpk = float(np.clip(vpk, 0.0, 500.0))
    vgk = float(np.clip(vgk, -10.0, 1.0))
    if vpk <= 0:
        return 0.0
    inner = KP * (1.0 / MU + vgk / np.sqrt(KVB + vpk * vpk))
    inner = np.clip(inner, -500, 500)
    Ed = (vpk / KP) * np.log1p(np.exp(float(inner)))
    if Ed <= 0:
        return 0.0
    return (Ed ** EX) / KG1

def koren_dip_dvpk(vpk, vgk, h=0.01):
    return (koren_ip(vpk + h, vgk) - koren_ip(vpk - h, vgk)) / (2 * h)

def koren_dip_dvgk(vpk, vgk, h=0.01):
    return (koren_ip(vpk, vgk + h) - koren_ip(vpk, vgk - h)) / (2 * h)


# =============================================================================
# Single-Stage Simulation
# =============================================================================

def simulate_single(audio_in, use_bypass=False, settle=2000):
    n_total = len(audio_in)
    out_vplate = np.zeros(n_total)

    R_p = RP
    R_g = RG
    R_k = R_cathode_bypass if use_bypass else RK

    prev_ip = 0.5e-3
    hp_y = 0.0
    hp_x_prev = 0.0
    z_ck = 0.0

    for n in range(n_total):
        vin = audio_in[n]
        hp_y = c_hp * hp_y + hp_gain * (vin - hp_x_prev)
        hp_x_prev = vin
        v_grid_filtered = hp_y

        b_plate = VB
        b_grid = v_grid_filtered

        if use_bypass:
            b_rk = 0.0
            b_ck = z_ck
            bDiff = b_ck - b_rk
            b_cathode = b_ck - gamma_k * bDiff
        else:
            b_cathode = 0.0

        a_p = b_plate
        a_g = b_grid
        a_k = b_cathode

        Ip = prev_ip
        for iteration in range(20):
            Vpk = (a_p - a_k) - (R_p + R_k) * Ip
            Vgk = a_g - a_k - R_k * Ip
            ip_model = koren_ip(Vpk, Vgk)
            f_val = Ip - ip_model
            if abs(f_val) < 1e-10:
                break
            dip_dvpk = koren_dip_dvpk(Vpk, Vgk)
            dip_dvgk = koren_dip_dvgk(Vpk, Vgk)
            df_dIp = 1.0 + dip_dvpk * (R_p + R_k) + dip_dvgk * R_k
            if abs(df_dIp) < 1e-15:
                break
            Ip -= f_val / df_dIp
            Ip = max(Ip, 0.0)

        prev_ip = Ip
        b_p = a_p - 2.0 * R_p * Ip
        b_k = a_k + 2.0 * R_k * Ip
        v_plate = (a_p + b_p) / 2.0
        out_vplate[n] = v_plate

        if use_bypass:
            a_ck = b_k + b_cathode - b_ck
            a_rk = a_ck + bDiff
            z_ck = a_ck

    sl = slice(max(settle - 100, 0), settle)
    vp_dc = out_vplate[sl].mean()
    return out_vplate, vp_dc


# =============================================================================
# Biquad Filter Tone Stack
# =============================================================================

def biquad_coeffs(ftype, fc, fs, gain_db=0.0, Q=0.707):
    """
    Compute biquad filter coefficients.

    Parameters
    ----------
    ftype : str
        'lowshelf', 'highshelf', or 'peaking'
    fc : float
        Center/corner frequency (Hz)
    fs : float
        Sample rate (Hz)
    gain_db : float
        Gain in dB (for shelf/peaking)
    Q : float
        Quality factor

    Returns
    -------
    b : ndarray (3,)
    a : ndarray (3,)
    """
    A = 10.0 ** (gain_db / 40.0)  # sqrt of linear gain
    w0 = 2.0 * np.pi * fc / fs
    cos_w0 = np.cos(w0)
    sin_w0 = np.sin(w0)
    alpha = sin_w0 / (2.0 * Q)

    if ftype == 'lowshelf':
        sqA = np.sqrt(A)
        b0 = A * ((A + 1) - (A - 1) * cos_w0 + 2 * sqA * alpha)
        b1 = 2 * A * ((A - 1) - (A + 1) * cos_w0)
        b2 = A * ((A + 1) - (A - 1) * cos_w0 - 2 * sqA * alpha)
        a0 = (A + 1) + (A - 1) * cos_w0 + 2 * sqA * alpha
        a1 = -2 * ((A - 1) + (A + 1) * cos_w0)
        a2 = (A + 1) + (A - 1) * cos_w0 - 2 * sqA * alpha
    elif ftype == 'highshelf':
        sqA = np.sqrt(A)
        b0 = A * ((A + 1) + (A - 1) * cos_w0 + 2 * sqA * alpha)
        b1 = -2 * A * ((A - 1) + (A + 1) * cos_w0)
        b2 = A * ((A + 1) + (A - 1) * cos_w0 - 2 * sqA * alpha)
        a0 = (A + 1) - (A - 1) * cos_w0 + 2 * sqA * alpha
        a1 = 2 * ((A - 1) - (A + 1) * cos_w0)
        a2 = (A + 1) - (A - 1) * cos_w0 - 2 * sqA * alpha
    elif ftype == 'peaking':
        b0 = 1 + alpha * A
        b1 = -2 * cos_w0
        b2 = 1 - alpha * A
        a0 = 1 + alpha / A
        a1 = -2 * cos_w0
        a2 = 1 - alpha / A
    else:
        raise ValueError(f"Unknown filter type: {ftype}")

    b = np.array([b0, b1, b2]) / a0
    a = np.array([1.0, a1 / a0, a2 / a0])
    return b, a


def apply_biquad(samples, b, a):
    """
    Apply biquad filter using direct-form II transposed.

    Parameters
    ----------
    samples : ndarray
        Input signal
    b, a : ndarray (3,)
        Filter coefficients (a[0] assumed 1.0)

    Returns
    -------
    y : ndarray
        Filtered signal
    """
    n = len(samples)
    y = np.zeros(n)
    d1 = 0.0
    d2 = 0.0
    for i in range(n):
        x = samples[i]
        out = b[0] * x + d1
        d1 = b[1] * x - a[1] * out + d2
        d2 = b[2] * x - a[2] * out
        y[i] = out
    return y


def tone_stack(samples, bass, mid, treble, fs=48000):
    """
    3-band parametric EQ tone stack.

    Parameters
    ----------
    samples : ndarray
        Input signal
    bass : float
        0-10 (5=flat, 0=-15dB, 10=+15dB)
    mid : float
        0-10 (5=flat, 0=-12dB, 10=+12dB)
    treble : float
        0-10 (5=flat, 0=-15dB, 10=+15dB)
    fs : float
        Sample rate

    Returns
    -------
    y : ndarray
        Processed signal
    """
    bass_db = (bass - 5.0) * 3.0       # range: -15 to +15
    mid_db = (mid - 5.0) * 2.4         # range: -12 to +12
    treble_db = (treble - 5.0) * 3.0   # range: -15 to +15

    # Bass: lowshelf at 200Hz
    b_bass, a_bass = biquad_coeffs('lowshelf', 200.0, fs, bass_db, Q=0.707)
    y = apply_biquad(samples, b_bass, a_bass)

    # Mid: peaking at 800Hz, Q=0.7
    b_mid, a_mid = biquad_coeffs('peaking', 800.0, fs, mid_db, Q=0.7)
    y = apply_biquad(y, b_mid, a_mid)

    # Treble: highshelf at 3kHz
    b_tre, a_tre = biquad_coeffs('highshelf', 3000.0, fs, treble_db, Q=0.707)
    y = apply_biquad(y, b_tre, a_tre)

    return y


def tone_stack_freq_response(bass, mid, treble, fs=48000, n_points=4096):
    """Compute frequency response of tone stack by filtering an impulse."""
    impulse = np.zeros(n_points)
    impulse[0] = 1.0
    y = tone_stack(impulse, bass, mid, treble, fs)
    sp = np.fft.rfft(y)
    freqs = np.fft.rfftfreq(n_points, 1.0 / fs)
    mag_db = 20.0 * np.log10(np.abs(sp) + 1e-12)
    return freqs, mag_db


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
# Main
# =============================================================================

if __name__ == "__main__":
    demos_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "demos")
    os.makedirs(demos_dir, exist_ok=True)

    # Generate 3 seconds of 440Hz sine through 1-stage triode with bypass
    duration = 3.0
    n_settle = 2000
    n_audio = int(duration * FS)
    n_total = n_settle + n_audio
    t = np.arange(n_total) / FS
    audio_in = np.zeros(n_total)
    audio_in[n_settle:] = 0.5 * np.sin(2 * np.pi * 440 * t[n_settle:])

    print("Simulating 1-stage triode with bypass (3 seconds)...")
    vplate, vp_dc = simulate_single(audio_in, use_bypass=True, settle=n_settle)
    ac_signal = vplate[n_settle:] - vp_dc
    print(f"  Triode sim done. Peak AC = {np.max(np.abs(ac_signal)):.2f}V")

    # Presets
    presets = {
        "Fender Clean":    (6, 4, 7),
        "Marshall Crunch":  (5, 8, 6),
        "Scooped Metal":   (10, 0, 10),
        "Mid Boost":       (3, 10, 3),
    }

    # Process and save WAVs
    for name, (bass, mid, treble) in presets.items():
        print(f"\nProcessing: {name} (bass={bass}, mid={mid}, treble={treble})")
        processed = tone_stack(ac_signal, bass, mid, treble, fs=FS)
        fname = name.lower().replace(" ", "_")
        save_wav(os.path.join(demos_dir, f"tone_{fname}.wav"), processed, int(FS))

    # Plot frequency responses
    print("\nPlotting frequency responses...")
    fig, ax = plt.subplots(1, 1, figsize=(10, 6))
    colors = ['#1f77b4', '#d62728', '#2ca02c', '#ff7f0e']

    for i, (name, (bass, mid, treble)) in enumerate(presets.items()):
        freqs, mag_db = tone_stack_freq_response(bass, mid, treble, fs=FS)
        ax.semilogx(freqs[1:], mag_db[1:], color=colors[i], linewidth=2.0,
                     label=f"{name} (B={bass} M={mid} T={treble})")

    ax.set_xlim(20, 20000)
    ax.set_ylim(-20, 20)
    ax.set_xlabel("Frequency (Hz)", fontsize=12)
    ax.set_ylabel("Gain (dB)", fontsize=12)
    ax.set_title("Tone Stack Frequency Response -- 3-Band Parametric EQ", fontsize=14)
    ax.legend(loc='upper right', fontsize=10)
    ax.grid(True, alpha=0.3, which='both')
    ax.axhline(0, color='gray', linewidth=0.5, linestyle='--')

    plt.tight_layout()
    plot_path = os.path.join(demos_dir, "tonestack_freq.png")
    plt.savefig(plot_path, dpi=150)
    print(f"\nSaved: {plot_path}")
    print("Done.")
