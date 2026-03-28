r"""
full_chain_demo.py
Complete amp simulation chain: triode preamp -> tone stack -> cabinet IR.
Self-contained: all functions duplicated for standalone use.
"""

import numpy as np
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

try:
    from scipy.io import wavfile
    from scipy.signal import firwin2
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

# =============================================================================
# Circuit Parameters
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
hp_gain_coeff = (1.0 + c_hp) / 2.0

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
    R_k = R_cathode_bypass if use_bypass else RK

    prev_ip = 0.5e-3
    hp_y = 0.0
    hp_x_prev = 0.0
    z_ck = 0.0

    for n in range(n_total):
        vin = audio_in[n]
        hp_y = c_hp * hp_y + hp_gain_coeff * (vin - hp_x_prev)
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
            z_ck = a_ck

    sl = slice(max(settle - 100, 0), settle)
    vp_dc = out_vplate[sl].mean()
    return out_vplate, vp_dc


# =============================================================================
# Cascaded Stages
# =============================================================================

def simulate_cascade(audio_in, n_stages=2, interstage_atten_db=20.0,
                     use_bypass=True, settle=2000):
    """
    Simulate N cascaded 12AX7 stages.

    Parameters
    ----------
    interstage_atten_db : float
        Interstage attenuation in dB (converted to linear divider).
    """
    signal = audio_in.copy()
    atten_lin = 10.0 ** (interstage_atten_db / 20.0)

    for stage in range(n_stages):
        vplate, vp_dc = simulate_single(signal, use_bypass=use_bypass, settle=settle)
        ac_out = vplate - vp_dc
        if stage < n_stages - 1:
            signal = ac_out / atten_lin

    return vplate, vp_dc


# =============================================================================
# Biquad Tone Stack
# =============================================================================

def biquad_coeffs(ftype, fc, fs, gain_db=0.0, Q=0.707):
    A = 10.0 ** (gain_db / 40.0)
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
    bass_db = (bass - 5.0) * 3.0
    mid_db = (mid - 5.0) * 2.4
    treble_db = (treble - 5.0) * 3.0

    b_bass, a_bass = biquad_coeffs('lowshelf', 200.0, fs, bass_db, Q=0.707)
    y = apply_biquad(samples, b_bass, a_bass)

    b_mid, a_mid = biquad_coeffs('peaking', 800.0, fs, mid_db, Q=0.7)
    y = apply_biquad(y, b_mid, a_mid)

    b_tre, a_tre = biquad_coeffs('highshelf', 3000.0, fs, treble_db, Q=0.707)
    y = apply_biquad(y, b_tre, a_tre)
    return y


# =============================================================================
# Cabinet IR
# =============================================================================

def _apply_peak(mag, freqs, fc, gain_db, Q):
    gain_lin = 10.0 ** (gain_db / 20.0)
    bw = fc / Q
    shape = 1.0 / (1.0 + ((freqs - fc) / (bw / 2.0)) ** 2)
    mag *= (1.0 + (gain_lin - 1.0) * shape)


def _apply_lpf(mag, freqs, fc):
    ratio = freqs / fc
    atten = 1.0 / np.sqrt(1.0 + ratio ** 4)
    mag *= atten


def make_cabinet_ir(cab_type, n_taps=257, fs=48000):
    nyq = fs / 2.0
    n_freqs = 512
    freqs_hz = np.linspace(0, nyq, n_freqs)
    mag = np.ones(n_freqs)

    if cab_type == '1x12_open':
        _apply_peak(mag, freqs_hz, 100.0, 6.0, 2.0)
        _apply_peak(mag, freqs_hz, 2500.0, 3.0, 1.5)
        _apply_lpf(mag, freqs_hz, 5000.0)
    elif cab_type == '4x12_closed':
        _apply_peak(mag, freqs_hz, 80.0, 8.0, 3.0)
        _apply_peak(mag, freqs_hz, 3500.0, 2.0, 1.0)
        _apply_lpf(mag, freqs_hz, 4500.0)
    else:
        raise ValueError(f"Unknown cabinet type: {cab_type}")

    freqs_norm = freqs_hz / nyq

    if HAS_SCIPY:
        ir = firwin2(n_taps, freqs_norm, mag)
    else:
        full_mag = np.zeros(n_taps)
        n_half = n_taps // 2 + 1
        freq_interp = np.linspace(0, 1, n_half)
        mag_interp = np.interp(freq_interp, freqs_norm, mag)
        full_mag[:n_half] = mag_interp
        full_mag[n_half:] = mag_interp[-2:0:-1]
        ir = np.real(np.fft.ifft(full_mag))
        ir = np.roll(ir, n_taps // 2)

    ir *= np.hanning(n_taps)
    peak = np.max(np.abs(ir))
    if peak > 1e-12:
        ir /= peak
    return ir


def apply_cabinet(samples, ir_taps):
    return np.convolve(samples, ir_taps, mode='full')[:len(samples)]


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

    # Input: E power chord (82Hz + 123Hz + 165Hz) at 0.15V each, 3 seconds
    duration = 3.0
    n_settle = 2000
    n_audio = int(duration * FS)
    n_total = n_settle + n_audio
    t = np.arange(n_total) / FS

    audio_in = np.zeros(n_total)
    audio_in[n_settle:] = (0.15 * np.sin(2 * np.pi * 82.0 * t[n_settle:]) +
                           0.15 * np.sin(2 * np.pi * 123.0 * t[n_settle:]) +
                           0.15 * np.sin(2 * np.pi * 165.0 * t[n_settle:]))

    # Pre-generate cabinet IRs
    ir_1x12 = make_cabinet_ir('1x12_open', n_taps=257, fs=FS)
    ir_4x12 = make_cabinet_ir('4x12_closed', n_taps=257, fs=FS)

    # Amp presets
    presets = [
        {
            'name': 'Fender Deluxe Clean',
            'stages': 1,
            'bypass': True,
            'interstage_db': 20.0,
            'bass': 6, 'mid': 5, 'treble': 7,
            'cab': '1x12_open',
        },
        {
            'name': 'Marshall Plexi',
            'stages': 2,
            'bypass': True,
            'interstage_db': 20.0,
            'bass': 5, 'mid': 8, 'treble': 6,
            'cab': '4x12_closed',
        },
        {
            'name': 'Scooped Metal',
            'stages': 3,
            'bypass': True,
            'interstage_db': 24.0,
            'bass': 10, 'mid': 0, 'treble': 10,
            'cab': '4x12_closed',
        },
        {
            'name': 'Jazz Clean',
            'stages': 1,
            'bypass': False,
            'interstage_db': 20.0,
            'bass': 4, 'mid': 6, 'treble': 3,
            'cab': '1x12_open',
        },
    ]

    results = []

    for p in presets:
        name = p['name']
        print(f"\n{'='*60}")
        print(f"Processing: {name}")
        print(f"  {p['stages']} stage(s), bypass={p['bypass']}, "
              f"bass={p['bass']}/mid={p['mid']}/treble={p['treble']}, cab={p['cab']}")
        print(f"{'='*60}")

        # Triode stages
        if p['stages'] == 1:
            vplate, vp_dc = simulate_single(audio_in, use_bypass=p['bypass'],
                                            settle=n_settle)
        else:
            vplate, vp_dc = simulate_cascade(audio_in, n_stages=p['stages'],
                                             interstage_atten_db=p['interstage_db'],
                                             use_bypass=p['bypass'],
                                             settle=n_settle)

        ac_signal = vplate[n_settle:] - vp_dc
        print(f"  Triode peak AC: {np.max(np.abs(ac_signal)):.2f}V")

        # Tone stack
        toned = tone_stack(ac_signal, p['bass'], p['mid'], p['treble'], fs=FS)

        # Cabinet
        ir = ir_1x12 if p['cab'] == '1x12_open' else ir_4x12
        final = apply_cabinet(toned, ir)

        # Save WAV
        fname = name.lower().replace(" ", "_")
        wav_path = os.path.join(demos_dir, f"amp_{fname}.wav")
        save_wav(wav_path, final, int(FS))

        results.append((name, final))

    # Comparison plot: 4 rows, waveform + FFT for each
    print("\nGenerating comparison plot...")
    fig, axes = plt.subplots(4, 2, figsize=(16, 16))
    fig.suptitle("Full Amp Chain Comparison -- E Power Chord (82+123+165 Hz)",
                 fontsize=14, fontweight='bold')

    colors = ['#1f77b4', '#d62728', '#2ca02c', '#ff7f0e']
    t_audio = np.arange(n_audio) / FS

    for i, (name, signal) in enumerate(results):
        # Waveform (show first 50ms)
        ax = axes[i, 0]
        mask = t_audio < 0.05
        ax.plot(t_audio[mask] * 1000, signal[mask], color=colors[i], linewidth=0.8)
        ax.set_ylabel("Amplitude")
        ax.set_title(f"{name} -- Waveform")
        ax.grid(True, alpha=0.3)
        if i == 3:
            ax.set_xlabel("Time (ms)")

        # FFT
        ax = axes[i, 1]
        N = len(signal)
        w = np.hanning(N)
        sp = np.abs(np.fft.rfft(signal * w))
        sp_db = 20 * np.log10(sp / (np.max(sp) + 1e-12) + 1e-12)
        f = np.fft.rfftfreq(N, 1.0 / FS)
        ax.plot(f, sp_db, color=colors[i], linewidth=0.8)
        ax.set_xlim(0, 8000)
        ax.set_ylim(-80, 5)
        ax.set_ylabel("dB")
        ax.set_title(f"{name} -- Spectrum")
        ax.grid(True, alpha=0.3)
        # Mark fundamental frequencies
        for freq in [82, 123, 165]:
            ax.axvline(freq, color='gray', alpha=0.3, ls='--', lw=0.5)
        if i == 3:
            ax.set_xlabel("Frequency (Hz)")

    plt.tight_layout()
    plot_path = os.path.join(demos_dir, "amp_comparison.png")
    plt.savefig(plot_path, dpi=150)
    print(f"\nSaved: {plot_path}")
    print("Done.")
