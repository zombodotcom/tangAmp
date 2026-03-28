r"""
cabinet_ir.py
Synthetic cabinet impulse response simulation.
Self-contained: duplicates the WDF triode sim core from wdf_triode_sim_v2.py.
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
# Cabinet IR
# =============================================================================

def make_cabinet_ir(cab_type, n_taps=257, fs=48000):
    """
    Create a synthetic cabinet impulse response as an FIR filter.

    Parameters
    ----------
    cab_type : str
        '1x12_open' or '4x12_closed'
    n_taps : int
        Number of FIR taps
    fs : float
        Sample rate

    Returns
    -------
    ir : ndarray
        FIR filter taps (n_taps,)
    """
    nyq = fs / 2.0
    n_freqs = 512
    freqs_hz = np.linspace(0, nyq, n_freqs)
    mag = np.ones(n_freqs)

    if cab_type == '1x12_open':
        # Low peak at 100Hz, Q=2
        _apply_peak(mag, freqs_hz, 100.0, 6.0, 2.0)
        # Presence at 2.5kHz, Q=1.5
        _apply_peak(mag, freqs_hz, 2500.0, 3.0, 1.5)
        # LPF at 5kHz
        _apply_lpf(mag, freqs_hz, 5000.0)
    elif cab_type == '4x12_closed':
        # Low peak at 80Hz, Q=3, more bass (+8dB)
        _apply_peak(mag, freqs_hz, 80.0, 8.0, 3.0)
        # Presence at 3.5kHz, Q=1
        _apply_peak(mag, freqs_hz, 3500.0, 2.0, 1.0)
        # LPF at 4.5kHz
        _apply_lpf(mag, freqs_hz, 4500.0)
    else:
        raise ValueError(f"Unknown cabinet type: {cab_type}")

    # Normalize frequencies to [0, 1] for firwin2
    freqs_norm = freqs_hz / nyq

    if HAS_SCIPY:
        # Use scipy.signal.firwin2 for frequency sampling FIR design
        ir = firwin2(n_taps, freqs_norm, mag)
    else:
        # Fallback: inverse FFT approach
        # Build full symmetric spectrum
        full_mag = np.zeros(n_taps)
        n_half = n_taps // 2 + 1
        freq_interp = np.linspace(0, 1, n_half)
        mag_interp = np.interp(freq_interp, freqs_norm, mag)
        full_mag[:n_half] = mag_interp
        # Mirror for negative frequencies
        full_mag[n_half:] = mag_interp[-2:0:-1]
        ir = np.real(np.fft.ifft(full_mag))
        ir = np.roll(ir, n_taps // 2)

    # Apply Hann window
    ir *= np.hanning(n_taps)

    # Normalize peak to 1
    peak = np.max(np.abs(ir))
    if peak > 1e-12:
        ir /= peak

    return ir


def _apply_peak(mag, freqs, fc, gain_db, Q):
    """Apply a resonant peak to a magnitude array."""
    gain_lin = 10.0 ** (gain_db / 20.0)
    bw = fc / Q
    shape = 1.0 / (1.0 + ((freqs - fc) / (bw / 2.0)) ** 2)
    mag *= (1.0 + (gain_lin - 1.0) * shape)


def _apply_lpf(mag, freqs, fc):
    """Apply a 2nd-order low-pass rolloff to a magnitude array."""
    ratio = freqs / fc
    atten = 1.0 / np.sqrt(1.0 + ratio ** 4)
    mag *= atten


def apply_cabinet(samples, ir_taps):
    """Convolve samples with cabinet IR."""
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
    demos_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "demos")
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

    # Generate cabinet IRs
    print("\nGenerating cabinet IRs...")
    ir_1x12 = make_cabinet_ir('1x12_open', n_taps=257, fs=FS)
    ir_4x12 = make_cabinet_ir('4x12_closed', n_taps=257, fs=FS)
    print(f"  1x12 open:   {len(ir_1x12)} taps")
    print(f"  4x12 closed: {len(ir_4x12)} taps")

    # Apply cabinets
    print("\nApplying cabinets...")
    wet_1x12 = apply_cabinet(ac_signal, ir_1x12)
    wet_4x12 = apply_cabinet(ac_signal, ir_4x12)

    # Save WAVs
    save_wav(os.path.join(demos_dir, "wet_nocab.wav"), ac_signal, int(FS))
    save_wav(os.path.join(demos_dir, "wet_cab_1x12.wav"), wet_1x12, int(FS))
    save_wav(os.path.join(demos_dir, "wet_cab_4x12.wav"), wet_4x12, int(FS))

    # Plot: cabinet IR waveforms + frequency responses
    print("\nPlotting cabinet IRs...")
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle("Synthetic Cabinet Impulse Responses", fontsize=14, fontweight='bold')

    cab_data = [
        ("1x12 Open Back (Fender)", ir_1x12, '#1f77b4'),
        ("4x12 Closed Back (Marshall)", ir_4x12, '#d62728'),
    ]

    for i, (label, ir, color) in enumerate(cab_data):
        # Time-domain IR
        ax = axes[i, 0]
        t_ir = np.arange(len(ir)) / FS * 1000  # ms
        ax.plot(t_ir, ir, color=color, linewidth=1.0)
        ax.set_xlabel("Time (ms)")
        ax.set_ylabel("Amplitude")
        ax.set_title(f"IR Waveform -- {label}")
        ax.grid(True, alpha=0.3)

        # Frequency response
        ax = axes[i, 1]
        n_fft = 4096
        ir_padded = np.zeros(n_fft)
        ir_padded[:len(ir)] = ir
        sp = np.abs(np.fft.rfft(ir_padded))
        sp_db = 20 * np.log10(sp + 1e-12)
        sp_db -= np.max(sp_db)  # normalize to 0 dB peak
        freqs = np.fft.rfftfreq(n_fft, 1.0 / FS)
        ax.semilogx(freqs[1:], sp_db[1:], color=color, linewidth=1.5)
        ax.set_xlim(20, 20000)
        ax.set_ylim(-40, 5)
        ax.set_xlabel("Frequency (Hz)")
        ax.set_ylabel("Gain (dB)")
        ax.set_title(f"Frequency Response -- {label}")
        ax.grid(True, alpha=0.3, which='both')

    plt.tight_layout()
    plot_path = os.path.join(demos_dir, "cabinet_ir.png")
    plt.savefig(plot_path, dpi=150)
    print(f"\nSaved: {plot_path}")
    print("Done.")
