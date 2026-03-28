r"""
oversample_demo.py
Demonstrates aliasing reduction with 2x oversampling on the tube nonlinearity.

Compares:
  1. 48kHz processing (current) — harmonics from Koren equation fold back as aliasing
  2. 96kHz internal / 48kHz output (2x oversampled) — aliasing pushed above audible range

Generates:
  - demos/oversample_comparison.png (FFT comparison)
  - demos/oversample_48k.wav
  - demos/oversample_96k.wav
"""

import numpy as np
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

try:
    from scipy.io import wavfile
    from scipy.signal import firwin, resample_poly
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

# =============================================================================
# Circuit Parameters (same as full_chain_demo.py)
# =============================================================================

VB  = 200.0
RP  = 100000.0
RG  = 1000000.0
RK  = 1500.0
CIN = 22e-9
CK  = 22e-6

MU, EX, KG1, KP, KVB = 100.0, 1.4, 1060.0, 600.0, 300.0


def make_hp_coeffs(fs):
    tau_hp = RG * CIN
    k_hp = 2.0 * fs * tau_hp
    c_hp = (k_hp - 1.0) / (k_hp + 1.0)
    hp_gain = (1.0 + c_hp) / 2.0
    return c_hp, hp_gain


def make_cathode_coeffs(fs):
    R_CK = 1.0 / (2.0 * fs * CK)
    G_rk = 1.0 / RK
    G_ck = 1.0 / R_CK
    G_total = G_rk + G_ck
    R_cathode_bypass = 1.0 / G_total
    gamma_k = G_rk / G_total
    return R_cathode_bypass, gamma_k


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
# Single-Stage Triode Simulation (parameterized by fs)
# =============================================================================

def simulate_single(audio_in, fs, use_bypass=True, settle=2000):
    n_total = len(audio_in)
    out_vplate = np.zeros(n_total)

    c_hp, hp_gain = make_hp_coeffs(fs)
    R_cathode_bypass, gamma_k = make_cathode_coeffs(fs)
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
            b_ck = z_ck
            b_cathode = b_ck - gamma_k * (b_ck - 0.0)
        else:
            b_cathode = 0.0

        a_p = b_plate
        a_g = b_grid
        a_k = b_cathode

        Ip = prev_ip
        for iteration in range(20):
            Vpk = (a_p - a_k) - (RP + R_k) * Ip
            Vgk = a_g - a_k - R_k * Ip
            ip_model = koren_ip(Vpk, Vgk)
            f_val = Ip - ip_model
            if abs(f_val) < 1e-10:
                break
            dip_dvpk = koren_dip_dvpk(Vpk, Vgk)
            dip_dvgk = koren_dip_dvgk(Vpk, Vgk)
            df_dIp = 1.0 + dip_dvpk * (RP + R_k) + dip_dvgk * R_k
            if abs(df_dIp) < 1e-15:
                break
            Ip -= f_val / df_dIp
            Ip = max(Ip, 0.0)

        prev_ip = Ip
        b_p = a_p - 2.0 * RP * Ip
        b_k = a_k + 2.0 * R_k * Ip
        v_plate = (a_p + b_p) / 2.0
        out_vplate[n] = v_plate

        if use_bypass:
            a_ck = b_k + b_cathode - b_ck
            z_ck = a_ck

    sl = slice(max(settle - 100, 0), settle)
    vp_dc = out_vplate[sl].mean()
    return out_vplate, vp_dc


def simulate_cascade(audio_in, fs, n_stages=2, interstage_atten_db=20.0,
                     use_bypass=True, settle=2000):
    signal = audio_in.copy()
    atten_lin = 10.0 ** (interstage_atten_db / 20.0)

    for stage in range(n_stages):
        vplate, vp_dc = simulate_single(signal, fs, use_bypass=use_bypass, settle=settle)
        ac_out = vplate - vp_dc
        if stage < n_stages - 1:
            signal = ac_out / atten_lin

    return vplate, vp_dc


# =============================================================================
# Save WAV
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

    FS = 48000.0
    FS_2X = 96000.0
    n_stages = 3
    duration = 1.0
    n_settle = 2000
    freq = 1000.0
    amplitude = 3.0  # 3V — heavy overdrive, 1kHz to push harmonics near Nyquist

    n_audio = int(duration * FS)
    n_total = n_settle + n_audio
    t = np.arange(n_total) / FS

    # Input: 440Hz sine at 2V amplitude (drives tube hard into clipping)
    audio_in = np.zeros(n_total)
    audio_in[n_settle:] = amplitude * np.sin(2 * np.pi * freq * t[n_settle:])

    # =========================================================================
    # Method 1: Standard 48kHz processing
    # =========================================================================
    print("Processing at 48kHz (no oversampling)...")
    vplate_48k, vp_dc_48k = simulate_cascade(
        audio_in, FS, n_stages=n_stages, use_bypass=True, settle=n_settle
    )
    ac_48k = vplate_48k[n_settle:] - vp_dc_48k

    # =========================================================================
    # Method 2: 2x oversampled (96kHz internal, 48kHz output)
    # =========================================================================
    print("Processing at 96kHz (2x oversampled)...")

    # Upsample input to 96kHz using linear interpolation (matches Verilog)
    n_total_2x = n_total * 2
    audio_in_2x = np.zeros(n_total_2x)
    for i in range(n_total):
        audio_in_2x[2 * i] = audio_in[i]
        if i > 0:
            audio_in_2x[2 * i - 1] = (audio_in[i - 1] + audio_in[i]) / 2.0
    # First interpolated sample
    audio_in_2x[1] = audio_in[0] / 2.0

    # Process triode at 96kHz
    vplate_96k, vp_dc_96k = simulate_cascade(
        audio_in_2x, FS_2X, n_stages=n_stages, use_bypass=True, settle=n_settle * 2
    )
    ac_96k_full = vplate_96k[n_settle * 2:] - vp_dc_96k

    # Downsample back to 48kHz: average pairs (matches Verilog decimation)
    n_out = len(ac_96k_full) // 2
    ac_96k = np.zeros(n_out)
    for i in range(n_out):
        ac_96k[i] = (ac_96k_full[2 * i] + ac_96k_full[2 * i + 1]) / 2.0

    # Ensure same length
    min_len = min(len(ac_48k), len(ac_96k))
    ac_48k = ac_48k[:min_len]
    ac_96k = ac_96k[:min_len]

    # =========================================================================
    # FFT Analysis
    # =========================================================================
    print("Computing FFTs...")
    N = len(ac_48k)

    # Use a window to reduce spectral leakage
    window = np.hanning(N)
    fft_48k = np.fft.rfft(ac_48k * window)
    fft_96k = np.fft.rfft(ac_96k * window)
    freqs = np.fft.rfftfreq(N, d=1.0 / FS)

    # Convert to dB (magnitude spectrum)
    mag_48k = 20 * np.log10(np.abs(fft_48k) + 1e-12)
    mag_96k = 20 * np.log10(np.abs(fft_96k) + 1e-12)

    # Normalize to peak
    peak_db = max(mag_48k.max(), mag_96k.max())
    mag_48k -= peak_db
    mag_96k -= peak_db

    # Compute aliasing energy (energy above 10kHz relative to total)
    freq_mask_high = freqs > 10000
    freq_mask_all = freqs > 0

    energy_48k_high = np.sum(np.abs(fft_48k[freq_mask_high]) ** 2)
    energy_48k_all = np.sum(np.abs(fft_48k[freq_mask_all]) ** 2)
    energy_96k_high = np.sum(np.abs(fft_96k[freq_mask_high]) ** 2)
    energy_96k_all = np.sum(np.abs(fft_96k[freq_mask_all]) ** 2)

    alias_ratio_48k = energy_48k_high / (energy_48k_all + 1e-30)
    alias_ratio_96k = energy_96k_high / (energy_96k_all + 1e-30)

    print(f"\n  Aliasing energy (>10kHz / total):")
    print(f"    48kHz:  {alias_ratio_48k * 100:.2f}%")
    print(f"    96kHz:  {alias_ratio_96k * 100:.2f}%")
    print(f"    Reduction: {alias_ratio_48k / max(alias_ratio_96k, 1e-30):.1f}x")

    # =========================================================================
    # Plot
    # =========================================================================
    print("Generating plot...")

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('2x Oversampling: Aliasing Reduction in Tube Nonlinearity',
                 fontsize=14, fontweight='bold')

    # Time-domain comparison (first 5ms)
    t_plot = np.arange(min(240, min_len)) / FS * 1000  # ms
    n_plot = len(t_plot)
    axes[0].plot(t_plot, ac_48k[:n_plot], 'b-', alpha=0.7, label='48kHz (no oversampling)')
    axes[0].plot(t_plot, ac_96k[:n_plot], 'r-', alpha=0.7, label='96kHz (2x oversampled)')
    axes[0].set_xlabel('Time (ms)')
    axes[0].set_ylabel('Amplitude (V)')
    axes[0].set_title('Time Domain (first 5ms of steady-state)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    # FFT comparison (full range)
    axes[1].plot(freqs / 1000, mag_48k, 'b-', alpha=0.7, linewidth=0.8,
                 label='48kHz (no oversampling)')
    axes[1].plot(freqs / 1000, mag_96k, 'r-', alpha=0.7, linewidth=0.8,
                 label='96kHz (2x oversampled)')
    axes[1].axvline(x=10, color='gray', linestyle='--', alpha=0.5, label='10kHz')
    axes[1].set_xlabel('Frequency (kHz)')
    axes[1].set_ylabel('Magnitude (dB)')
    axes[1].set_title(f'Spectrum Comparison ({int(freq)}Hz sine @ {amplitude:.0f}V through {n_stages}-stage 12AX7)')
    axes[1].set_xlim([0, 24])
    axes[1].set_ylim([-80, 5])
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    # FFT difference (aliasing reduction)
    diff_db = mag_48k - mag_96k
    axes[2].plot(freqs / 1000, diff_db, 'g-', alpha=0.7, linewidth=0.8)
    axes[2].axhline(y=0, color='gray', linestyle='-', alpha=0.3)
    axes[2].axvline(x=10, color='gray', linestyle='--', alpha=0.5, label='10kHz')
    axes[2].fill_between(freqs / 1000, 0, diff_db, where=diff_db > 0,
                         alpha=0.3, color='red', label='48kHz has more energy (aliasing)')
    axes[2].fill_between(freqs / 1000, 0, diff_db, where=diff_db < 0,
                         alpha=0.3, color='blue', label='96kHz has more energy')
    axes[2].set_xlabel('Frequency (kHz)')
    axes[2].set_ylabel('48kHz - 96kHz (dB)')
    axes[2].set_title(f'Aliasing Difference (>10kHz energy: '
                      f'48k={alias_ratio_48k*100:.1f}%, '
                      f'96k={alias_ratio_96k*100:.1f}%, '
                      f'{alias_ratio_48k/max(alias_ratio_96k,1e-30):.1f}x reduction)')
    axes[2].set_xlim([0, 24])
    axes[2].set_ylim([-20, 20])
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plot_path = os.path.join(demos_dir, "oversample_comparison.png")
    plt.savefig(plot_path, dpi=150)
    plt.close()
    print(f"  Saved: {plot_path}")

    # =========================================================================
    # Save WAVs
    # =========================================================================
    save_wav(os.path.join(demos_dir, "oversample_48k.wav"), ac_48k, fs=48000)
    save_wav(os.path.join(demos_dir, "oversample_96k.wav"), ac_96k, fs=48000)

    print("\nDone!")
