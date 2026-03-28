r"""
wdf_triode_sim_v2.py
Enhanced WDF triode simulation with:
  1. Cathode bypass capacitor (Parallel adaptor for Rk || Ck)
  2. Cascaded stages (2-stage Fender, 3-stage Marshall)

The cathode bypass cap removes negative feedback at audio frequencies,
increasing gain from ~29dB to ~34dB per stage.
"""

import numpy as np
import math
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
# Circuit Parameters
# =============================================================================

VB  = 200.0       # B+ supply (V)
RP  = 100000.0    # Plate resistor (ohms)
RG  = 1000000.0   # Grid resistor (ohms)
RK  = 1500.0      # Cathode resistor (ohms)
CIN = 22e-9       # Input coupling cap (F) -- 22nF
CK  = 22e-6       # Cathode bypass cap (F) -- 22uF
FS  = 48000.0     # Sample rate (Hz)

# Koren 12AX7 constants
MU, EX, KG1, KP, KVB = 100.0, 1.4, 1060.0, 600.0, 300.0

# Derived: high-pass filter for input coupling
tau_hp = RG * CIN
k_hp = 2.0 * FS * tau_hp
c_hp = (k_hp - 1.0) / (k_hp + 1.0)
hp_gain = (1.0 + c_hp) / 2.0

# Cathode bypass cap WDF port resistance
R_CK = 1.0 / (2.0 * FS * CK)  # ~0.4735 ohm

# Parallel adaptor parameters for Rk || Ck
G_rk = 1.0 / RK
G_ck = 1.0 / R_CK
G_total = G_rk + G_ck
R_cathode_bypass = 1.0 / G_total  # port resistance seen by root
gamma_k = G_rk / G_total           # scattering coefficient

print(f"=== Cathode Bypass Parallel Adaptor ===")
print(f"Rk = {RK:.0f} ohm, R_Ck = {R_CK:.4f} ohm")
print(f"R_cathode (with bypass) = {R_cathode_bypass:.4f} ohm")
print(f"gamma_k = {gamma_k:.6f}")
print(f"Bypass crossover freq = {1/(2*np.pi*RK*CK):.1f} Hz")


# =============================================================================
# Koren Model
# =============================================================================

def koren_ip(vpk, vgk):
    """Plate current Ip in amps."""
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
    """
    Simulate one 12AX7 triode stage using WDF.

    Parameters
    ----------
    audio_in : ndarray
        Input voltage signal (full length including settle period).
    use_bypass : bool
        If True, use Parallel(Rk, Ck) at cathode. Otherwise simple Rk.
    settle : int
        Number of settle samples at the start.

    Returns
    -------
    out_vplate : ndarray
        Plate voltage for each sample.
    vp_dc : float
        DC quiescent plate voltage (averaged over last 100 settle samples).
    """
    n_total = len(audio_in)

    out_vplate = np.zeros(n_total)

    # Port resistances
    R_p = RP
    R_g = RG
    if use_bypass:
        R_k = R_cathode_bypass
    else:
        R_k = RK

    # State
    prev_ip = 0.5e-3
    hp_y = 0.0
    hp_x_prev = 0.0
    z_ck = 0.0  # capacitor state for Ck

    for n in range(n_total):
        vin = audio_in[n]

        # High-pass filter (input coupling)
        hp_y = c_hp * hp_y + hp_gain * (vin - hp_x_prev)
        hp_x_prev = vin
        v_grid_filtered = hp_y

        # --- UPWARD PASS ---
        b_plate = VB
        b_grid = v_grid_filtered

        if use_bypass:
            # Parallel adaptor: Rk || Ck
            b_rk = 0.0          # resistor reflects 0
            b_ck = z_ck         # capacitor reflects state
            bDiff = b_ck - b_rk
            b_cathode = b_ck - gamma_k * bDiff
        else:
            b_cathode = 0.0

        # Incident waves at root = reflected waves from leaves (single-leaf)
        a_p = b_plate
        a_g = b_grid
        a_k = b_cathode

        # --- ROOT: Newton-Raphson ---
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

        # Reflected waves from triode root
        b_p = a_p - 2.0 * R_p * Ip
        b_k = a_k + 2.0 * R_k * Ip

        v_plate = (a_p + b_p) / 2.0
        out_vplate[n] = v_plate

        # --- DOWNWARD PASS ---
        if use_bypass:
            # Push root's reflected wave back through parallel adaptor
            # b_k is the wave coming down from root into the cathode port
            # For the parallel adaptor, the incident wave to the adaptor = b_k
            a_ck = b_k + b_cathode - b_ck
            a_rk = a_ck + bDiff
            z_ck = a_ck  # update capacitor state

    # Compute DC
    sl = slice(max(settle - 100, 0), settle)
    vp_dc = out_vplate[sl].mean()

    return out_vplate, vp_dc


# =============================================================================
# Cascaded Stages
# =============================================================================

def simulate_cascade(audio_in, n_stages=2, interstage_atten=20.0, settle=2000):
    """
    Simulate N cascaded 12AX7 stages, each with cathode bypass.

    Between stages, the signal is AC-coupled and attenuated by interstage_atten.
    This models the interstage coupling network (voltage divider + coupling cap).

    Returns
    -------
    out_vplate : ndarray
        Final stage plate voltage.
    vp_dc : float
        Final stage DC quiescent plate voltage.
    stage_gains : list of float
        Gain in dB for each individual stage.
    """
    n_total = len(audio_in)
    signal = audio_in.copy()
    stage_gains = []

    for stage in range(n_stages):
        vplate, vp_dc = simulate_single(signal, use_bypass=True, settle=settle)

        # AC-coupled output
        ac_out = vplate - vp_dc

        # Measure gain for this stage
        audio_part = signal[settle:]
        out_part = ac_out[settle:]
        in_rms = np.sqrt(np.mean(audio_part**2))
        out_rms = np.sqrt(np.mean(out_part**2))
        if in_rms > 1e-12:
            g = 20 * np.log10(out_rms / in_rms)
        else:
            g = 0.0
        stage_gains.append(g)

        if stage < n_stages - 1:
            # Interstage: AC couple and attenuate
            signal = ac_out / interstage_atten

    return vplate, vp_dc, stage_gains


# =============================================================================
# THD calculation
# =============================================================================

def calc_thd(sig, f0=440.0, fs=48000.0, n_harmonics=8):
    """Calculate THD from FFT, returns percentage."""
    N = len(sig)
    w = np.hanning(N)
    sp = np.abs(np.fft.rfft(sig * w))
    freqs = np.fft.rfftfreq(N, 1.0 / fs)

    # Find fundamental and harmonic peaks (within +/- 5 bins)
    powers = []
    for h in range(1, n_harmonics + 1):
        target = f0 * h
        idx = np.argmin(np.abs(freqs - target))
        lo = max(0, idx - 5)
        hi = min(len(sp), idx + 6)
        powers.append(np.max(sp[lo:hi]))

    fund = powers[0]
    if fund < 1e-12:
        return 0.0
    harm_power = np.sqrt(sum(p**2 for p in powers[1:]))
    return 100.0 * harm_power / fund


# =============================================================================
# WAV output
# =============================================================================

def save_wav(filename, signal, fs=48000):
    """Save signal as 16-bit WAV, normalized to 0.9 peak."""
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
    # Setup
    n_settle = 2000
    n_audio  = 4800
    n_total  = n_settle + n_audio

    t = np.arange(n_total) / FS
    audio_in = np.zeros(n_total)
    audio_in[n_settle:] = 0.5 * np.sin(2 * np.pi * 440 * t[n_settle:])

    demos_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "demos")
    os.makedirs(demos_dir, exist_ok=True)

    results = {}

    # =========================================================================
    # (a) Single stage, no bypass -- should match ~29dB
    # =========================================================================
    print("\n" + "="*60)
    print("(a) Single stage, NO bypass cap")
    print("="*60)

    vplate_a, vp_dc_a = simulate_single(audio_in, use_bypass=False, settle=n_settle)
    ac_a = vplate_a[n_settle:] - vp_dc_a
    inp = audio_in[n_settle:]
    in_rms = np.sqrt(np.mean(inp**2))
    out_rms_a = np.sqrt(np.mean(ac_a**2))
    gain_a = 20 * np.log10(out_rms_a / (in_rms + 1e-12))
    thd_a = calc_thd(ac_a)
    print(f"  Vp_dc = {vp_dc_a:.2f}V")
    print(f"  Gain  = {gain_a:.1f} dB")
    print(f"  THD   = {thd_a:.2f}%")
    results['1stage_no_bypass'] = {'gain': gain_a, 'thd': thd_a, 'ac': ac_a}
    save_wav(os.path.join(demos_dir, "wet_1stage.wav"), ac_a, int(FS))

    # =========================================================================
    # (b) Single stage, WITH bypass cap -- should be ~34dB
    # =========================================================================
    print("\n" + "="*60)
    print("(b) Single stage, WITH bypass cap (Ck=22uF)")
    print("="*60)

    vplate_b, vp_dc_b = simulate_single(audio_in, use_bypass=True, settle=n_settle)
    ac_b = vplate_b[n_settle:] - vp_dc_b
    out_rms_b = np.sqrt(np.mean(ac_b**2))
    gain_b = 20 * np.log10(out_rms_b / (in_rms + 1e-12))
    thd_b = calc_thd(ac_b)
    print(f"  Vp_dc = {vp_dc_b:.2f}V")
    print(f"  Gain  = {gain_b:.1f} dB")
    print(f"  THD   = {thd_b:.2f}%")
    results['1stage_bypass'] = {'gain': gain_b, 'thd': thd_b, 'ac': ac_b}
    save_wav(os.path.join(demos_dir, "wet_1stage_bypass.wav"), ac_b, int(FS))

    # =========================================================================
    # (c) 2-stage cascade with bypass
    # =========================================================================
    print("\n" + "="*60)
    print("(c) 2-stage cascade, WITH bypass")
    print("="*60)

    vplate_c, vp_dc_c, gains_c = simulate_cascade(
        audio_in, n_stages=2, interstage_atten=20.0, settle=n_settle)
    ac_c = vplate_c[n_settle:] - vp_dc_c
    out_rms_c = np.sqrt(np.mean(ac_c**2))
    gain_c = 20 * np.log10(out_rms_c / (in_rms + 1e-12))
    thd_c = calc_thd(ac_c)
    print(f"  Stage gains: {[f'{g:.1f}dB' for g in gains_c]}")
    print(f"  Total gain  = {gain_c:.1f} dB")
    print(f"  THD         = {thd_c:.2f}%")
    results['2stage'] = {'gain': gain_c, 'thd': thd_c, 'ac': ac_c, 'stage_gains': gains_c}
    save_wav(os.path.join(demos_dir, "wet_2stage.wav"), ac_c, int(FS))

    # =========================================================================
    # (d) 3-stage cascade with bypass
    # =========================================================================
    print("\n" + "="*60)
    print("(d) 3-stage cascade, WITH bypass")
    print("="*60)

    vplate_d, vp_dc_d, gains_d = simulate_cascade(
        audio_in, n_stages=3, interstage_atten=20.0, settle=n_settle)
    ac_d = vplate_d[n_settle:] - vp_dc_d
    out_rms_d = np.sqrt(np.mean(ac_d**2))
    gain_d = 20 * np.log10(out_rms_d / (in_rms + 1e-12))
    thd_d = calc_thd(ac_d)
    print(f"  Stage gains: {[f'{g:.1f}dB' for g in gains_d]}")
    print(f"  Total gain  = {gain_d:.1f} dB")
    print(f"  THD         = {thd_d:.2f}%")
    results['3stage'] = {'gain': gain_d, 'thd': thd_d, 'ac': ac_d, 'stage_gains': gains_d}
    save_wav(os.path.join(demos_dir, "wet_3stage.wav"), ac_d, int(FS))

    # =========================================================================
    # Summary
    # =========================================================================
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    print(f"{'Config':<30s} {'Gain (dB)':>10s} {'THD (%)':>10s}")
    print("-"*52)
    print(f"{'1-stage, no bypass':<30s} {gain_a:>10.1f} {thd_a:>10.2f}")
    print(f"{'1-stage, with bypass':<30s} {gain_b:>10.1f} {thd_b:>10.2f}")
    print(f"{'2-stage cascade':<30s} {gain_c:>10.1f} {thd_c:>10.2f}")
    print(f"{'3-stage cascade':<30s} {gain_d:>10.1f} {thd_d:>10.2f}")

    # =========================================================================
    # Comparison Plot
    # =========================================================================
    fig, axes = plt.subplots(4, 2, figsize=(16, 14))
    fig.suptitle("WDF Triode Cascade Comparison -- 12AX7", fontsize=14, fontweight='bold')

    t_audio = np.arange(n_audio) / FS * 1000  # ms
    mask = t_audio < 10  # first 10ms

    configs = [
        ("1-stage, no bypass", ac_a, gain_a, thd_a),
        ("1-stage + bypass (Ck)", ac_b, gain_b, thd_b),
        ("2-stage cascade", ac_c, gain_c, thd_c),
        ("3-stage cascade", ac_d, gain_d, thd_d),
    ]

    for i, (label, ac, gain, thd) in enumerate(configs):
        # Waveform
        ax = axes[i, 0]
        ax.plot(t_audio[mask], ac[mask], 'r', linewidth=1.0)
        ax.set_ylabel("V")
        ax.set_title(f"{label} -- {gain:.1f}dB, THD={thd:.1f}%")
        ax.grid(True, alpha=0.3)
        if i == 3:
            ax.set_xlabel("Time (ms)")

        # FFT
        ax = axes[i, 1]
        N = len(ac)
        w = np.hanning(N)
        sp = np.abs(np.fft.rfft(ac * w))
        sp_db = 20 * np.log10(sp / (max(sp) + 1e-12) + 1e-12)
        f = np.fft.rfftfreq(N, 1.0 / FS)
        ax.plot(f, sp_db, 'r', linewidth=0.8)
        ax.set_xlim(0, 5000)
        ax.set_ylim(-80, 5)
        ax.set_ylabel("dB")
        ax.set_title(f"Spectrum -- {label}")
        ax.grid(True, alpha=0.3)
        for h in range(1, 8):
            ax.axvline(440 * h, color='gray', alpha=0.3, ls='--', lw=0.5)
        if i == 3:
            ax.set_xlabel("Frequency (Hz)")

    plt.tight_layout()
    plot_path = os.path.join(demos_dir, "cascade_comparison.png")
    plt.savefig(plot_path, dpi=150)
    print(f"\nSaved: {plot_path}")
    print("Done.")
