r"""
bias_tremolo.py
Bias tremolo effect -- modulates power tube bias voltage with an LFO.

Classic Fender "vibrato" channel effect: instead of simply multiplying the
output by a sine wave (volume tremolo), this shifts the tube's operating
point. As the bias changes, the gain AND harmonic content change together,
producing a more organic, asymmetric pulsing characteristic of real tube amps.

Generates demo WAVs and a comparison plot showing bias vs volume tremolo.
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

# ============================================================================
# Tube Constants (from amp_sim.py)
# ============================================================================

TUBES = {
    "12AX7": dict(mu=100.0, ex=1.4, kg1=1060.0, kp=600.0, kvb=300.0),
    "12AU7": dict(mu=27.48, ex=1.03, kg1=466.13, kp=135.10, kvb=24224.55),
    "6SL7":  dict(mu=90.41, ex=1.25, kg1=597.32, kp=511.97, kvb=6747.79),
    "EL34":  dict(mu=10.98, ex=1.42, kg1=249.65, kp=43.2,   kvb=333.0),
    "6L6":   dict(mu=10.11, ex=1.37, kg1=406.6,  kp=31.2,   kvb=640.7),
    "300B":  dict(mu=3.95,  ex=1.4,  kg1=1550.0, kp=65.0,   kvb=300.0),
}

FS = 48000.0

# Preamp circuit constants
PREAMP_VB  = 200.0
PREAMP_RP  = 100000.0
PREAMP_RG  = 1000000.0
PREAMP_RK  = 1500.0
PREAMP_CIN = 22e-9
PREAMP_CK  = 22e-6

_tau_hp = PREAMP_RG * PREAMP_CIN
_k_hp = 2.0 * FS * _tau_hp
_c_hp = (_k_hp - 1.0) / (_k_hp + 1.0)
_hp_gain = (1.0 + _c_hp) / 2.0

_R_CK = 1.0 / (2.0 * FS * PREAMP_CK)
_G_rk = 1.0 / PREAMP_RK
_G_ck = 1.0 / _R_CK
_G_total = _G_rk + _G_ck
_R_cathode_bypass = 1.0 / _G_total
_gamma_k = _G_rk / _G_total


# ============================================================================
# Koren Model
# ============================================================================

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


# ============================================================================
# LFO Generator
# ============================================================================

def generate_lfo(n_samples, rate_hz, wave='sine', fs=48000.0):
    """Generate LFO signal in [-1, +1] range."""
    t = np.arange(n_samples) / fs
    phase = 2.0 * np.pi * rate_hz * t
    if wave == 'sine':
        return np.sin(phase)
    elif wave == 'triangle':
        return 2.0 * np.abs(2.0 * (t * rate_hz - np.floor(t * rate_hz + 0.5))) - 1.0
    else:
        raise ValueError(f"Unknown wave type: {wave}")


# ============================================================================
# Preamp Stage with Bias Tremolo
# ============================================================================

def simulate_preamp_bias_tremolo(audio_in, tube_name='12AX7', rate_hz=4.0,
                                  depth=0.3, wave='sine', settle=2000):
    """
    Simulate preamp triode with bias tremolo.

    Bias tremolo modulates the cathode bias resistance (Rk), which shifts
    the tube's operating point. At higher effective Rk the tube runs cooler
    (less gain, cleaner). At lower effective Rk it runs hotter (more gain,
    dirtier). This is how Fender "vibrato" works -- an LFO modulates the
    bias of the output tubes.

    Parameters
    ----------
    audio_in : ndarray
        Input signal (including settle samples).
    tube_name : str
        Tube type key.
    rate_hz : float
        Tremolo rate in Hz (typical 2-8 Hz).
    depth : float
        Modulation depth 0.0 (none) to 1.0 (full).
    wave : str
        LFO waveform: 'sine' or 'triangle'.
    settle : int
        Number of settle samples at start.

    Returns
    -------
    out_vplate : ndarray
        Plate voltage output.
    vp_dc : float
        DC operating point (from settle region).
    lfo : ndarray
        The LFO signal used (for plotting).
    """
    tube = TUBES[tube_name]
    n_total = len(audio_in)
    out_vplate = np.zeros(n_total)

    # Generate LFO -- zero during settle, active after
    lfo = np.zeros(n_total)
    lfo[settle:] = generate_lfo(n_total - settle, rate_hz, wave, FS)

    # Nominal cathode resistance (with bypass cap)
    R_k_nominal = _R_cathode_bypass

    prev_ip = 0.5e-3
    hp_y = 0.0
    hp_x_prev = 0.0
    z_ck = 0.0

    R_p = PREAMP_RP

    for n in range(n_total):
        vin = audio_in[n]

        # High-pass coupling cap filter
        hp_y = _c_hp * hp_y + _hp_gain * (vin - hp_x_prev)
        hp_x_prev = vin
        v_grid = hp_y

        # Bias tremolo: modulate cathode resistance
        # depth=0 -> no modulation, depth=1 -> Rk swings from 0 to 2x nominal
        R_k = R_k_nominal * (1.0 + depth * lfo[n])
        R_k = max(R_k, 1.0)  # Prevent zero/negative resistance

        # Cathode bypass: recompute gamma for modulated Rk
        G_rk_mod = 1.0 / (PREAMP_RK * (1.0 + depth * lfo[n]))
        G_rk_mod = max(G_rk_mod, 1e-12)
        G_total_mod = G_rk_mod + _G_ck
        gamma_k_mod = G_rk_mod / G_total_mod
        R_k = 1.0 / G_total_mod

        b_plate = PREAMP_VB
        b_grid = v_grid

        b_rk = 0.0
        b_ck = z_ck
        bDiff = b_ck - b_rk
        b_cathode = b_ck - gamma_k_mod * bDiff

        a_p = b_plate
        a_g = b_grid
        a_k = b_cathode

        # Newton-Raphson solve for Ip
        Ip = prev_ip
        for iteration in range(20):
            Vpk = (a_p - a_k) - (R_p + R_k) * Ip
            Vgk = a_g - a_k - R_k * Ip
            ip_model = koren_ip(Vpk, Vgk, **tube)
            f_val = Ip - ip_model
            if abs(f_val) < 1e-10:
                break
            dip_dvpk_v = koren_dip_dvpk(Vpk, Vgk, tube)
            dip_dvgk_v = koren_dip_dvgk(Vpk, Vgk, tube)
            df_dIp = 1.0 + dip_dvpk_v * (R_p + R_k) + dip_dvgk_v * R_k
            if abs(df_dIp) < 1e-15:
                break
            Ip -= f_val / df_dIp
            Ip = max(Ip, 0.0)

        prev_ip = Ip
        b_p = a_p - 2.0 * R_p * Ip
        b_k = a_k + 2.0 * R_k * Ip
        v_plate = (a_p + b_p) / 2.0
        out_vplate[n] = v_plate

        # Update capacitor state
        a_ck = b_k + b_cathode - b_ck
        z_ck = a_ck

    sl = slice(max(settle - 100, 0), settle)
    vp_dc = out_vplate[sl].mean()
    return out_vplate, vp_dc, lfo


def simulate_volume_tremolo(audio_in, rate_hz=4.0, depth=0.3, wave='sine',
                            tube_name='12AX7', settle=2000):
    """
    Simple volume tremolo: run clean preamp, then multiply output by LFO.

    This is the "cheap" approach -- just modulate amplitude. No change in
    harmonic content or operating point.
    """
    from amp_sim import simulate_preamp_stage

    vplate, vp_dc = simulate_preamp_stage(audio_in, tube_name=tube_name,
                                           use_bypass=True, settle=settle)
    ac_out = vplate - vp_dc

    n_total = len(audio_in)
    lfo = np.zeros(n_total)
    lfo[settle:] = generate_lfo(n_total - settle, rate_hz, wave, FS)

    # Volume tremolo: multiply by (1 - depth/2 + depth/2 * lfo)
    # This swings from (1-depth/2) to (1+depth/2), centered at 1.0
    mod = 1.0 - depth / 2.0 + (depth / 2.0) * lfo
    modulated = ac_out * mod

    return modulated, lfo


# ============================================================================
# WAV Output
# ============================================================================

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


# ============================================================================
# Harmonic Analysis Helper
# ============================================================================

def compute_harmonics_over_cycle(signal, rate_hz, fs, n_windows=8, fund_freq=82.0):
    """
    Compute harmonic content at different phases of the tremolo cycle.

    Splits one tremolo cycle into n_windows segments and computes the
    spectrum of each, returning harmonic amplitudes.
    """
    cycle_samples = int(fs / rate_hz)
    # Use a full cycle from the middle of the signal
    mid = len(signal) // 2
    start = mid - cycle_samples // 2
    start = max(0, start)

    window_len = cycle_samples // n_windows
    harmonics_list = []

    for i in range(n_windows):
        seg_start = start + i * window_len
        seg_end = seg_start + window_len
        if seg_end > len(signal):
            break
        segment = signal[seg_start:seg_end]
        w = np.hanning(len(segment))
        sp = np.abs(np.fft.rfft(segment * w))
        freqs = np.fft.rfftfreq(len(segment), 1.0 / fs)

        # Extract harmonics 1-6
        harm_amps = []
        for h in range(1, 7):
            target = fund_freq * h
            idx = np.argmin(np.abs(freqs - target))
            # Sum over a small window around the harmonic
            lo = max(0, idx - 2)
            hi = min(len(sp), idx + 3)
            harm_amps.append(np.max(sp[lo:hi]))
        harmonics_list.append(harm_amps)

    return np.array(harmonics_list)


# ============================================================================
# Main Demo
# ============================================================================

if __name__ == "__main__":
    demos_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "demos")
    os.makedirs(demos_dir, exist_ok=True)

    # ========================================================================
    # Input: sustained E power chord (82 + 123 + 165 Hz), 3 seconds
    # ========================================================================
    duration = 3.0
    n_settle = 2000
    n_audio = int(duration * FS)
    n_total = n_settle + n_audio
    t = np.arange(n_total) / FS

    audio_in = np.zeros(n_total)
    # Sustained chord with slight decay envelope for realism
    env = np.ones(n_total)
    env[n_settle:] = np.exp(-0.3 * (t[n_settle:] - t[n_settle]))
    audio_in[n_settle:] = env[n_settle:] * (
        0.15 * np.sin(2 * np.pi * 82.0 * t[n_settle:]) +
        0.12 * np.sin(2 * np.pi * 123.0 * t[n_settle:]) +
        0.10 * np.sin(2 * np.pi * 165.0 * t[n_settle:])
    )

    # ========================================================================
    # 1. Bias tremolo at different rates
    # ========================================================================
    rates = [2.0, 4.0, 6.0]
    bias_results = {}

    for rate in rates:
        print(f"Simulating bias tremolo at {rate} Hz...")
        vplate, vp_dc, lfo = simulate_preamp_bias_tremolo(
            audio_in, tube_name='12AX7', rate_hz=rate, depth=0.4,
            wave='sine', settle=n_settle)
        ac_out = vplate[n_settle:] - vp_dc
        bias_results[rate] = ac_out

        wav_path = os.path.join(demos_dir, f"tremolo_bias_{int(rate)}hz.wav")
        save_wav(wav_path, ac_out, int(FS))

    # ========================================================================
    # 2. Bias tremolo vs volume tremolo at 4 Hz
    # ========================================================================
    print("\nSimulating bias tremolo at 4 Hz (depth=0.4)...")
    vplate_bias, vp_dc_bias, lfo_bias = simulate_preamp_bias_tremolo(
        audio_in, tube_name='12AX7', rate_hz=4.0, depth=0.4,
        wave='sine', settle=n_settle)
    ac_bias = vplate_bias[n_settle:] - vp_dc_bias

    print("Simulating volume tremolo at 4 Hz (depth=0.4)...")
    ac_vol, lfo_vol = simulate_volume_tremolo(
        audio_in, rate_hz=4.0, depth=0.4, wave='sine',
        tube_name='12AX7', settle=n_settle)
    ac_vol = ac_vol[n_settle:]

    save_wav(os.path.join(demos_dir, "tremolo_volume_4hz.wav"), ac_vol, int(FS))

    # ========================================================================
    # 3. Harmonic content over tremolo cycle
    # ========================================================================
    print("\nAnalyzing harmonic content over tremolo cycle...")
    harm_bias = compute_harmonics_over_cycle(ac_bias, 4.0, FS, n_windows=8, fund_freq=82.0)
    harm_vol = compute_harmonics_over_cycle(ac_vol, 4.0, FS, n_windows=8, fund_freq=82.0)

    # ========================================================================
    # 4. Comparison Plot
    # ========================================================================
    print("\nGenerating comparison plot...")
    t_audio = np.arange(n_audio) / FS

    fig = plt.figure(figsize=(18, 16))
    fig.suptitle("Bias Tremolo vs Volume Tremolo -- E Power Chord, 4 Hz, depth=0.4",
                 fontsize=14, fontweight='bold')

    # --- Row 1: Waveform envelopes ---
    ax1 = fig.add_subplot(4, 2, 1)
    ax1.plot(t_audio, ac_bias, linewidth=0.3, color='#1f77b4', alpha=0.6)
    env_bias = np.abs(ac_bias)
    # Smooth envelope
    win = int(FS / 20)
    kernel = np.ones(win) / win
    env_bias_smooth = np.convolve(env_bias, kernel, mode='same')
    ax1.plot(t_audio, env_bias_smooth, 'k-', linewidth=1.5, label='Envelope')
    ax1.set_title("Bias Tremolo -- Waveform")
    ax1.set_ylabel("Voltage")
    ax1.set_xlim(0, duration)
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)

    ax2 = fig.add_subplot(4, 2, 2)
    ax2.plot(t_audio, ac_vol, linewidth=0.3, color='#d62728', alpha=0.6)
    env_vol = np.abs(ac_vol)
    env_vol_smooth = np.convolve(env_vol, kernel, mode='same')
    ax2.plot(t_audio, env_vol_smooth, 'k-', linewidth=1.5, label='Envelope')
    ax2.set_title("Volume Tremolo -- Waveform")
    ax2.set_ylabel("Voltage")
    ax2.set_xlim(0, duration)
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)

    # --- Row 2: Zoomed waveform (2 tremolo cycles) ---
    zoom_start = 0.5
    zoom_end = zoom_start + 2.0 / 4.0  # 2 cycles at 4 Hz
    mask = (t_audio >= zoom_start) & (t_audio < zoom_end)

    ax3 = fig.add_subplot(4, 2, 3)
    ax3.plot(t_audio[mask] * 1000, ac_bias[mask], linewidth=0.6, color='#1f77b4')
    ax3.set_title("Bias Tremolo -- Zoomed (2 cycles)")
    ax3.set_ylabel("Voltage")
    ax3.set_xlabel("Time (ms)")
    ax3.grid(True, alpha=0.3)

    ax4 = fig.add_subplot(4, 2, 4)
    ax4.plot(t_audio[mask] * 1000, ac_vol[mask], linewidth=0.6, color='#d62728')
    ax4.set_title("Volume Tremolo -- Zoomed (2 cycles)")
    ax4.set_ylabel("Voltage")
    ax4.set_xlabel("Time (ms)")
    ax4.grid(True, alpha=0.3)

    # --- Row 3: Spectra ---
    ax5 = fig.add_subplot(4, 2, 5)
    N = len(ac_bias)
    w = np.hanning(N)
    sp_bias = np.abs(np.fft.rfft(ac_bias * w))
    sp_bias_db = 20 * np.log10(sp_bias / (np.max(sp_bias) + 1e-12) + 1e-12)
    f = np.fft.rfftfreq(N, 1.0 / FS)
    ax5.plot(f, sp_bias_db, color='#1f77b4', linewidth=0.8)
    ax5.set_xlim(0, 4000)
    ax5.set_ylim(-80, 5)
    ax5.set_title("Bias Tremolo -- Spectrum")
    ax5.set_ylabel("dB")
    ax5.set_xlabel("Frequency (Hz)")
    for freq in [82, 123, 165]:
        ax5.axvline(freq, color='gray', alpha=0.3, ls='--', lw=0.5)
    ax5.grid(True, alpha=0.3)

    ax6 = fig.add_subplot(4, 2, 6)
    sp_vol = np.abs(np.fft.rfft(ac_vol * w))
    sp_vol_db = 20 * np.log10(sp_vol / (np.max(sp_vol) + 1e-12) + 1e-12)
    ax6.plot(f, sp_vol_db, color='#d62728', linewidth=0.8)
    ax6.set_xlim(0, 4000)
    ax6.set_ylim(-80, 5)
    ax6.set_title("Volume Tremolo -- Spectrum")
    ax6.set_ylabel("dB")
    ax6.set_xlabel("Frequency (Hz)")
    for freq in [82, 123, 165]:
        ax6.axvline(freq, color='gray', alpha=0.3, ls='--', lw=0.5)
    ax6.grid(True, alpha=0.3)

    # --- Row 4: Harmonic content over tremolo cycle ---
    ax7 = fig.add_subplot(4, 2, 7)
    phases = np.linspace(0, 360, harm_bias.shape[0], endpoint=False)
    for h in range(min(6, harm_bias.shape[1])):
        vals = harm_bias[:, h]
        if np.max(vals) > 1e-12:
            vals_norm = vals / np.max(vals)
        else:
            vals_norm = vals
        ax7.plot(phases, vals_norm, 'o-', linewidth=1.5, markersize=4,
                 label=f"H{h+1} ({82*(h+1)} Hz)")
    ax7.set_title("Bias Tremolo -- Harmonics vs LFO Phase")
    ax7.set_xlabel("LFO Phase (degrees)")
    ax7.set_ylabel("Normalized Amplitude")
    ax7.legend(fontsize=7, ncol=2)
    ax7.grid(True, alpha=0.3)

    ax8 = fig.add_subplot(4, 2, 8)
    for h in range(min(6, harm_vol.shape[1])):
        vals = harm_vol[:, h]
        if np.max(vals) > 1e-12:
            vals_norm = vals / np.max(vals)
        else:
            vals_norm = vals
        ax8.plot(phases, vals_norm, 'o-', linewidth=1.5, markersize=4,
                 label=f"H{h+1} ({82*(h+1)} Hz)")
    ax8.set_title("Volume Tremolo -- Harmonics vs LFO Phase")
    ax8.set_xlabel("LFO Phase (degrees)")
    ax8.set_ylabel("Normalized Amplitude")
    ax8.legend(fontsize=7, ncol=2)
    ax8.grid(True, alpha=0.3)

    plt.tight_layout()
    plot_path = os.path.join(demos_dir, "tremolo_comparison.png")
    plt.savefig(plot_path, dpi=150)
    print(f"  Saved: {plot_path}")
    plt.close()

    # ========================================================================
    # Summary
    # ========================================================================
    print("\n" + "=" * 60)
    print("Bias Tremolo Demo Complete")
    print("=" * 60)
    print(f"\nBias tremolo key difference: harmonic content changes with LFO phase.")
    print(f"  - At LFO peak (higher Rk): tube runs cooler, less gain, cleaner tone")
    print(f"  - At LFO trough (lower Rk): tube runs hotter, more gain, dirtier tone")
    print(f"\nVolume tremolo: all harmonics scale uniformly (just amplitude change).")

    # Quick stats
    bias_rms = np.sqrt(np.mean(ac_bias ** 2))
    vol_rms = np.sqrt(np.mean(ac_vol ** 2))
    print(f"\n  Bias tremolo RMS: {bias_rms:.2f} V")
    print(f"  Volume tremolo RMS: {vol_rms:.2f} V")

    # Measure modulation depth from envelope
    env_bias_mid = env_bias_smooth[len(env_bias_smooth)//4 : 3*len(env_bias_smooth)//4]
    env_vol_mid = env_vol_smooth[len(env_vol_smooth)//4 : 3*len(env_vol_smooth)//4]
    if len(env_bias_mid) > 0:
        bias_mod = (np.max(env_bias_mid) - np.min(env_bias_mid)) / (np.max(env_bias_mid) + np.min(env_bias_mid) + 1e-12)
        vol_mod = (np.max(env_vol_mid) - np.min(env_vol_mid)) / (np.max(env_vol_mid) + np.min(env_vol_mid) + 1e-12)
        print(f"  Bias tremolo modulation index: {bias_mod:.3f}")
        print(f"  Volume tremolo modulation index: {vol_mod:.3f}")

    print("\nDone.")
