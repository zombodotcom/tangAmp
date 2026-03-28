r"""
output_transformer.py
Output transformer model for tube power amplifier.

Models the key sonic characteristics of a real output transformer:
  1. Bandpass filtering: HPF ~60Hz (primary inductance) + LPF ~8kHz (leakage inductance)
  2. Frequency-dependent saturation: core flux = integral(V)/N, so low frequencies
     produce more flux and saturate first. This creates the "sag" and "bloom" that
     tube amp players love.

The saturation model:
  - Core flux is proportional to V/(f*N): lower frequencies require more flux
    for the same voltage amplitude, so they saturate first
  - When flux exceeds core capacity, primary inductance drops, causing waveform
    distortion (mostly even harmonics due to asymmetric B-H curve)
  - We model this as tanh soft-clipping with a threshold that decreases when
    low-frequency content is high (envelope-following)

Usage:
  python output_transformer.py
  -> generates demos/transformer_comparison.png,
     demos/power_no_transformer.wav, demos/power_with_transformer.wav
"""

import numpy as np
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

try:
    from scipy.io import wavfile
    from scipy.signal import butter, sosfilt, sosfiltfilt
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

FS = 48000.0


# =============================================================================
# Biquad coefficients (for non-scipy fallback and Verilog reference)
# =============================================================================

def biquad_highpass(fc, fs, Q=0.7071):
    """2nd-order highpass biquad coefficients (Direct Form I)."""
    w0 = 2.0 * np.pi * fc / fs
    alpha = np.sin(w0) / (2.0 * Q)
    cos_w0 = np.cos(w0)

    b0 = (1.0 + cos_w0) / 2.0
    b1 = -(1.0 + cos_w0)
    b2 = (1.0 + cos_w0) / 2.0
    a0 = 1.0 + alpha
    a1 = -2.0 * cos_w0
    a2 = 1.0 - alpha

    return np.array([b0/a0, b1/a0, b2/a0, 1.0, a1/a0, a2/a0])


def biquad_lowpass(fc, fs, Q=0.7071):
    """2nd-order lowpass biquad coefficients (Direct Form I)."""
    w0 = 2.0 * np.pi * fc / fs
    alpha = np.sin(w0) / (2.0 * Q)
    cos_w0 = np.cos(w0)

    b0 = (1.0 - cos_w0) / 2.0
    b1 = 1.0 - cos_w0
    b2 = (1.0 - cos_w0) / 2.0
    a0 = 1.0 + alpha
    a1 = -2.0 * cos_w0
    a2 = 1.0 - alpha

    return np.array([b0/a0, b1/a0, b2/a0, 1.0, a1/a0, a2/a0])


def biquad_process(coeffs, x):
    """Apply a biquad filter to signal x (Direct Form II Transposed)."""
    b0, b1, b2, _, a1, a2 = coeffs
    y = np.zeros_like(x)
    z1 = 0.0
    z2 = 0.0
    for n in range(len(x)):
        inp = x[n]
        out = b0 * inp + z1
        z1 = b1 * inp - a1 * out + z2
        z2 = b2 * inp - a2 * out
        y[n] = out
    return y


# =============================================================================
# Output Transformer Model
# =============================================================================

def output_transformer(samples, fs=48000.0, hpf_fc=60.0, lpf_fc=8000.0,
                       sat_threshold=1.0, sat_lf_depth=0.6, lf_envelope_fc=200.0,
                       impedance_ratio=50.0):
    """
    Output transformer model.

    Parameters
    ----------
    samples : ndarray
        Input signal (AC-coupled plate voltage from power amp).
    fs : float
        Sample rate.
    hpf_fc : float
        High-pass cutoff (Hz). Limited by primary inductance -- real transformers
        roll off around 40-80Hz depending on quality.
    lpf_fc : float
        Low-pass cutoff (Hz). Limited by leakage inductance and winding
        capacitance -- typically 6-12kHz for guitar amp transformers.
    sat_threshold : float
        Normalized saturation threshold (1.0 = signal peak after bandpass).
        Core saturates when flux exceeds this. Lower = more saturation.
    sat_lf_depth : float
        How much low-frequency content reduces the saturation threshold.
        0.0 = no frequency dependence, 1.0 = full dependence.
        This models the fact that core flux ~ V/f, so bass saturates first.
    lf_envelope_fc : float
        Cutoff for the low-frequency envelope detector (Hz).
    impedance_ratio : float
        Turns ratio squared (Zprimary/Zsecondary). Typical: 5000/8 ~ 625,
        but we use a moderate value since we scale output separately.

    Returns
    -------
    out : ndarray
        Transformer output signal.
    """
    x = np.array(samples, dtype=np.float64)

    # ---- Step 1: Bandpass filtering ----
    # HPF: primary inductance limits low-frequency coupling
    hpf_coeffs = biquad_highpass(hpf_fc, fs, Q=0.7071)
    x = biquad_process(hpf_coeffs, x)

    # LPF: leakage inductance + winding capacitance limits highs
    lpf_coeffs = biquad_lowpass(lpf_fc, fs, Q=0.7071)
    x = biquad_process(lpf_coeffs, x)

    # ---- Step 2: Frequency-dependent saturation ----
    # Extract low-frequency envelope: LPF of |x| at ~200Hz
    # This tracks how much low-frequency content is present
    env_coeffs = biquad_lowpass(lf_envelope_fc, fs, Q=0.5)
    lf_envelope = biquad_process(env_coeffs, np.abs(x))

    # Normalize envelope relative to signal peak
    peak = np.max(np.abs(x))
    if peak < 1e-12:
        return x

    env_norm = lf_envelope / peak

    # Dynamic saturation threshold: lower when bass content is high
    # Core flux ~ integral(V) ~ V/f, so at 80Hz the flux is 10x that at 800Hz
    # for the same voltage. When flux is high, core saturates.
    dyn_threshold = sat_threshold * (1.0 - sat_lf_depth * np.clip(env_norm, 0, 1))

    # Apply soft saturation (tanh)
    # Scale so that signal at peak just starts to enter saturation region
    x_norm = x / peak
    out = np.zeros_like(x)
    for n in range(len(x)):
        thresh = max(dyn_threshold[n], 0.05)  # floor to avoid division by zero
        out[n] = thresh * np.tanh(x_norm[n] / thresh)

    # Rescale back
    out *= peak

    # Impedance transformation (voltage step-down)
    turns_ratio = np.sqrt(impedance_ratio)
    out /= turns_ratio

    return out


def transformer_frequency_response(fs=48000.0, hpf_fc=60.0, lpf_fc=8000.0, n_points=4096):
    """Compute the frequency response of the transformer's linear bandpass."""
    # Impulse response
    impulse = np.zeros(n_points)
    impulse[0] = 1.0

    hpf_coeffs = biquad_highpass(hpf_fc, fs, Q=0.7071)
    lpf_coeffs = biquad_lowpass(lpf_fc, fs, Q=0.7071)

    y = biquad_process(hpf_coeffs, impulse)
    y = biquad_process(lpf_coeffs, y)

    H = np.fft.rfft(y)
    freqs = np.fft.rfftfreq(n_points, 1.0 / fs)
    mag_db = 20.0 * np.log10(np.abs(H) + 1e-12)

    return freqs, mag_db


# =============================================================================
# Power Amp (duplicated from power_amp.py for standalone use)
# =============================================================================

TUBES = {
    "6L6":  dict(mu=10.11, ex=1.37, kg1=406.6, kp=31.2, kvb=640.7),
    "EL34": dict(mu=10.98, ex=1.42, kg1=249.65, kp=43.2, kvb=333.0),
}


def koren_ip(vpk, vgk, mu, ex, kg1, kp, kvb):
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


def simulate_power_amp(audio_in, tube_name='6L6', vb=400.0, rp=2000.0, rk=250.0,
                       rg=1e6, cin=22e-9, sag_amount=0.3, fs=48000.0, settle=2000):
    """Power amp sim WITHOUT transformer (raw plate voltage, AC-coupled)."""
    tube = TUBES[tube_name]
    n_total = len(audio_in)
    out_vplate = np.zeros(n_total)

    tau_hp = rg * cin
    k_hp = 2.0 * fs * tau_hp
    c_hp = (k_hp - 1.0) / (k_hp + 1.0)
    hp_gain = (1.0 + c_hp) / 2.0

    sag_tau = 0.05
    sag_alpha = 1.0 / (1.0 + 2.0 * fs * sag_tau)
    sag_level = 0.0

    prev_ip = 1.0e-3
    hp_y = 0.0
    hp_x_prev = 0.0

    R_p = rp
    R_k = rk

    for n in range(n_total):
        vin = audio_in[n]
        hp_y = c_hp * hp_y + hp_gain * (vin - hp_x_prev)
        hp_x_prev = vin
        v_grid = hp_y

        vb_eff = vb - sag_amount * sag_level * 100.0

        a_p = vb_eff
        a_g = v_grid
        a_k = 0.0

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

        if n >= settle:
            sag_level = sag_level * (1.0 - sag_alpha) + abs(Ip) * sag_alpha

    sl = slice(max(settle - 100, 0), settle)
    vp_dc = out_vplate[sl].mean()
    ac_out = out_vplate - vp_dc

    return ac_out, vp_dc


# =============================================================================
# WAV helpers
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
# Main: Generate comparison demo
# =============================================================================

if __name__ == "__main__":
    demos_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demos")
    os.makedirs(demos_dir, exist_ok=True)

    print("=" * 60)
    print("Output Transformer Model")
    print("=" * 60)

    # ----- Generate test signals -----
    duration = 1.5
    n_settle = 2000
    n_audio = int(duration * FS)
    n_total = n_settle + n_audio
    t = np.arange(n_total) / FS

    # Low E power chord (82Hz fundamental) -- will saturate transformer core
    audio_low = np.zeros(n_total)
    audio_low[n_settle:] = (0.3 * np.sin(2 * np.pi * 82.0 * t[n_settle:]) +
                            0.2 * np.sin(2 * np.pi * 123.0 * t[n_settle:]) +
                            0.15 * np.sin(2 * np.pi * 165.0 * t[n_settle:]))

    # High notes (330Hz + 660Hz) -- less core saturation
    audio_high = np.zeros(n_total)
    audio_high[n_settle:] = (0.3 * np.sin(2 * np.pi * 330.0 * t[n_settle:]) +
                             0.2 * np.sin(2 * np.pi * 660.0 * t[n_settle:]) +
                             0.15 * np.sin(2 * np.pi * 990.0 * t[n_settle:]))

    # Mixed: starts with high note, transitions to low power chord
    audio_mixed = np.zeros(n_total)
    t_audio = t[n_settle:]
    fade_point = 0.75
    fade_in = np.clip((t_audio - fade_point) / 0.1, 0, 1)
    fade_out = 1.0 - fade_in
    audio_mixed[n_settle:] = (
        fade_out * (0.3 * np.sin(2 * np.pi * 330.0 * t_audio) +
                    0.2 * np.sin(2 * np.pi * 660.0 * t_audio)) +
        fade_in * (0.4 * np.sin(2 * np.pi * 82.0 * t_audio) +
                   0.3 * np.sin(2 * np.pi * 123.0 * t_audio))
    )

    # ----- Run power amp -----
    print("\nRunning power amp (6L6)...")
    raw_low, _ = simulate_power_amp(audio_low, tube_name='6L6', settle=n_settle)
    raw_high, _ = simulate_power_amp(audio_high, tube_name='6L6', settle=n_settle)
    raw_mixed, _ = simulate_power_amp(audio_mixed, tube_name='6L6', settle=n_settle)

    # Trim settle
    raw_low = raw_low[n_settle:]
    raw_high = raw_high[n_settle:]
    raw_mixed = raw_mixed[n_settle:]

    # ----- Apply transformer -----
    print("Applying output transformer...")
    xf_low = output_transformer(raw_low, fs=FS, sat_threshold=0.7, sat_lf_depth=0.6)
    xf_high = output_transformer(raw_high, fs=FS, sat_threshold=0.7, sat_lf_depth=0.6)
    xf_mixed = output_transformer(raw_mixed, fs=FS, sat_threshold=0.7, sat_lf_depth=0.6)

    # Also make a "full signal" version for the WAVs
    raw_full, _ = simulate_power_amp(audio_low, tube_name='6L6', settle=n_settle)
    raw_full = raw_full[n_settle:]
    xf_full = output_transformer(raw_full, fs=FS, sat_threshold=0.7, sat_lf_depth=0.6)

    # ----- Save WAVs -----
    print("\nSaving WAVs...")
    save_wav(os.path.join(demos_dir, "power_no_transformer.wav"), raw_full, int(FS))
    save_wav(os.path.join(demos_dir, "power_with_transformer.wav"), xf_full, int(FS))

    # ----- Frequency response -----
    print("Computing frequency response...")
    freqs, mag_db = transformer_frequency_response(fs=FS)

    # ----- Stats -----
    for label, raw, xf in [("Low E chord", raw_low, xf_low),
                            ("High notes", raw_high, xf_high)]:
        raw_peak = np.max(np.abs(raw))
        xf_peak = np.max(np.abs(xf))
        print(f"\n  {label}:")
        print(f"    Raw peak:         {raw_peak:.1f}V")
        print(f"    Transformer peak: {xf_peak:.2f}V")

    # ----- Plot -----
    print("\nGenerating comparison plot...")
    fig = plt.figure(figsize=(18, 16))
    gs = fig.add_gridspec(4, 2, hspace=0.35, wspace=0.3)
    fig.suptitle("Output Transformer Model", fontsize=14, fontweight='bold')

    t_plot = np.arange(n_audio) / FS

    # Row 1: Frequency response of transformer bandpass
    ax_freq = fig.add_subplot(gs[0, :])
    ax_freq.plot(freqs, mag_db, 'k-', linewidth=1.5)
    ax_freq.set_xlim(20, 20000)
    ax_freq.set_xscale('log')
    ax_freq.set_ylim(-40, 5)
    ax_freq.set_xlabel("Frequency (Hz)")
    ax_freq.set_ylabel("Magnitude (dB)")
    ax_freq.set_title("Transformer Frequency Response (HPF 60Hz + LPF 8kHz, 2nd order)")
    ax_freq.grid(True, alpha=0.3, which='both')
    ax_freq.axvline(60, color='r', ls='--', alpha=0.5, label='HPF 60Hz')
    ax_freq.axvline(8000, color='b', ls='--', alpha=0.5, label='LPF 8kHz')
    ax_freq.legend()

    # Row 2: Low notes -- raw vs transformer (shows saturation)
    mask_30ms = t_plot < 0.03

    ax_low_raw = fig.add_subplot(gs[1, 0])
    ax_low_raw.plot(t_plot[mask_30ms] * 1000, raw_low[mask_30ms], 'b-', linewidth=0.8)
    ax_low_raw.set_title("Low E Chord -- Raw Power Amp")
    ax_low_raw.set_ylabel("Voltage (V)")
    ax_low_raw.grid(True, alpha=0.3)

    ax_low_xf = fig.add_subplot(gs[1, 1])
    ax_low_xf.plot(t_plot[mask_30ms] * 1000, xf_low[mask_30ms], 'r-', linewidth=0.8)
    ax_low_xf.set_title("Low E Chord -- After Transformer (saturated)")
    ax_low_xf.set_ylabel("Voltage (V)")
    ax_low_xf.grid(True, alpha=0.3)

    # Row 3: High notes -- raw vs transformer (cleaner)
    ax_hi_raw = fig.add_subplot(gs[2, 0])
    ax_hi_raw.plot(t_plot[mask_30ms] * 1000, raw_high[mask_30ms], 'b-', linewidth=0.8)
    ax_hi_raw.set_title("High Notes (330Hz) -- Raw Power Amp")
    ax_hi_raw.set_ylabel("Voltage (V)")
    ax_hi_raw.grid(True, alpha=0.3)

    ax_hi_xf = fig.add_subplot(gs[2, 1])
    ax_hi_xf.plot(t_plot[mask_30ms] * 1000, xf_high[mask_30ms], 'r-', linewidth=0.8)
    ax_hi_xf.set_title("High Notes (330Hz) -- After Transformer (less saturated)")
    ax_hi_xf.set_ylabel("Voltage (V)")
    ax_hi_xf.grid(True, alpha=0.3)

    # Row 4: Spectra comparison (low chord, raw vs transformer)
    N = len(raw_low)
    w = np.hanning(N)
    f_axis = np.fft.rfftfreq(N, 1.0 / FS)

    sp_raw = np.abs(np.fft.rfft(raw_low * w))
    sp_raw_db = 20.0 * np.log10(sp_raw / (np.max(sp_raw) + 1e-12) + 1e-12)

    sp_xf = np.abs(np.fft.rfft(xf_low * w))
    sp_xf_db = 20.0 * np.log10(sp_xf / (np.max(sp_xf) + 1e-12) + 1e-12)

    ax_spec = fig.add_subplot(gs[3, 0])
    ax_spec.plot(f_axis, sp_raw_db, 'b-', linewidth=0.8, label='Raw')
    ax_spec.plot(f_axis, sp_xf_db, 'r-', linewidth=0.8, alpha=0.7, label='Transformer')
    ax_spec.set_xlim(20, 15000)
    ax_spec.set_ylim(-80, 5)
    ax_spec.set_xlabel("Frequency (Hz)")
    ax_spec.set_ylabel("dB")
    ax_spec.set_title("Spectrum: Low E Chord (raw vs transformer)")
    ax_spec.legend()
    ax_spec.grid(True, alpha=0.3)
    for freq in [82, 123, 165, 246, 330]:
        ax_spec.axvline(freq, color='gray', alpha=0.2, ls='--', lw=0.5)

    # Spectrum for high notes
    sp_raw_h = np.abs(np.fft.rfft(raw_high * w))
    sp_raw_h_db = 20.0 * np.log10(sp_raw_h / (np.max(sp_raw_h) + 1e-12) + 1e-12)
    sp_xf_h = np.abs(np.fft.rfft(xf_high * w))
    sp_xf_h_db = 20.0 * np.log10(sp_xf_h / (np.max(sp_xf_h) + 1e-12) + 1e-12)

    ax_spec_h = fig.add_subplot(gs[3, 1])
    ax_spec_h.plot(f_axis, sp_raw_h_db, 'b-', linewidth=0.8, label='Raw')
    ax_spec_h.plot(f_axis, sp_xf_h_db, 'r-', linewidth=0.8, alpha=0.7, label='Transformer')
    ax_spec_h.set_xlim(20, 15000)
    ax_spec_h.set_ylim(-80, 5)
    ax_spec_h.set_xlabel("Frequency (Hz)")
    ax_spec_h.set_ylabel("dB")
    ax_spec_h.set_title("Spectrum: High Notes (raw vs transformer)")
    ax_spec_h.legend()
    ax_spec_h.grid(True, alpha=0.3)
    for freq in [330, 660, 990]:
        ax_spec_h.axvline(freq, color='gray', alpha=0.2, ls='--', lw=0.5)

    plot_path = os.path.join(demos_dir, "transformer_comparison.png")
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    print(f"\nSaved: {plot_path}")
    plt.close()

    print("\nDone.")
