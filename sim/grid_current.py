"""
grid_current.py
Grid current modeling for WDF triode simulation.

When the guitar signal drives the grid positive (Vgk > 0), real tubes draw
grid current via the Langmuir-Child law. This causes:
  - Asymmetric clipping of positive peaks
  - Loading of the previous stage
  - "Blocking distortion" at high gain settings
  - Even harmonic content characteristic of cranked tube amps

This script compares the standard Ig=0 model against the grid current model
at multiple drive levels, producing waveform and THD comparison plots plus
WAV files at the highest drive level.
"""

import numpy as np
import math
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.io import wavfile

# =============================================================================
# Circuit Parameters (same as wdf_triode_sim_wdf.py)
# =============================================================================

VB  = 200.0       # B+ supply (V)
RP  = 100000.0    # Plate resistor (ohms)
RG  = 1000000.0   # Grid resistor (ohms)
RK  = 1500.0      # Cathode resistor (ohms)
CIN = 22e-9       # Input coupling cap (F) -- 22nF
CK  = 22e-6       # Cathode bypass cap (F) -- 22uF
FS  = 48000.0     # Sample rate (Hz)

# Cathode bypass cap WDF port resistance
R_CK = 1.0 / (2.0 * FS * CK)
G_rk = 1.0 / RK
G_ck = 1.0 / R_CK
G_total = G_rk + G_ck
R_CATH = 1.0 / G_total
GAMMA_CATH = G_rk / G_total

# Input coupling high-pass filter
tau_hp = RG * CIN
k_hp = 2.0 * FS * tau_hp
c_hp = (k_hp - 1.0) / (k_hp + 1.0)
hp_gain = (1.0 + c_hp) / 2.0

# =============================================================================
# Koren 12AX7 Triode Model
# =============================================================================

MU, EX, KG1, KP, KVB = 100.0, 1.4, 1060.0, 600.0, 300.0

def koren_ip(vpk, vgk):
    """Plate current Ip in amps."""
    vpk = float(np.clip(vpk, 0.0, 500.0))
    vgk = float(np.clip(vgk, -10.0, 5.0))
    if vpk <= 0:
        return 0.0
    inner = KP * (1.0 / MU + vgk / math.sqrt(KVB + vpk * vpk))
    inner = float(np.clip(inner, -500, 500))
    Ed = (vpk / KP) * math.log1p(math.exp(inner))
    if Ed <= 0:
        return 0.0
    return (Ed ** EX) / KG1

def koren_dip_dvpk(vpk, vgk, h=0.01):
    """Numerical dIp/dVpk."""
    return (koren_ip(vpk + h, vgk) - koren_ip(vpk - h, vgk)) / (2 * h)

def koren_dip_dvgk(vpk, vgk, h=0.01):
    """Numerical dIp/dVgk."""
    return (koren_ip(vpk, vgk + h) - koren_ip(vpk, vgk - h)) / (2 * h)

# =============================================================================
# Grid Current Model (Langmuir-Child law for grid emission)
# =============================================================================

IG_MAX = 0.002    # Max grid current scaling (A)
VG_ONSET = 0.0    # Onset voltage (V) -- grid current starts at Vgk=0

def grid_current(vgk, ig_max=IG_MAX, vg_onset=VG_ONSET):
    """
    Grid current model based on Langmuir-Child law.
    When Vgk > vg_onset, the grid draws current from the signal source.

    Ig = ig_max * max(0, Vgk - vg_onset)^1.5

    This models the grid acting as a small anode when driven positive,
    collecting electrons from the cathode space charge. The 3/2 power
    law comes from the same physics as the Child-Langmuir law for
    space-charge-limited current.

    Parameters:
        vgk:       Grid-to-cathode voltage (V)
        ig_max:    Scaling factor (A) -- depends on tube geometry
        vg_onset:  Onset voltage (V) -- typically 0V or slightly negative

    Returns:
        Ig in amps (always >= 0)
    """
    if vgk <= vg_onset:
        return 0.0
    return ig_max * (vgk - vg_onset) ** 1.5

def grid_current_deriv(vgk, ig_max=IG_MAX, vg_onset=VG_ONSET):
    """
    Derivative dIg/dVgk for Newton-Raphson solver.
    dIg/dVgk = 1.5 * ig_max * max(0, Vgk - vg_onset)^0.5
    """
    if vgk <= vg_onset:
        return 0.0
    return 1.5 * ig_max * (vgk - vg_onset) ** 0.5

# =============================================================================
# WDF Simulation with Optional Grid Current
# =============================================================================

def simulate_wdf(audio_in, enable_grid_current=False):
    """
    Run WDF triode simulation.

    When enable_grid_current=True, the root solver accounts for grid current:
      - Ig = grid_current(Vgk)  instead of Ig = 0
      - b_g = a_g - 2*R_g*Ig   instead of b_g = a_g
      - KCL at cathode: Ik = -(Ip + Ig), so cathode sees more current
      - Newton-Raphson solves coupled (Ip, Ig) system

    The Newton-Raphson iteration becomes a 2-equation system:
      f1(Ip, Ig) = Ip - koren_ip(Vpk, Vgk)
      f2(Ig)     = Ig - grid_current(Vgk)

    Where:
      Vpk = (a_p - a_k) - (R_p + R_k)*(Ip + Ig) + R_k*Ig
          = (a_p - a_k) - R_p*(Ip + Ig) - R_k*Ip
      Actually, let's be precise. Cathode current Ik = Ip + Ig (both flow
      through cathode resistor), so:
        v_k = R_k * Ik = R_k * (Ip + Ig)
      The WDF formulation with cathode port resistance R_k:
        Vpk = (a_p - a_k)/2 ... no, let's use the standard WDF root equations.

      With grid current, the reflected waves become:
        b_p = a_p - 2*R_p*Ip         (plate current only through plate resistor)
        b_g = a_g - 2*R_g*Ig         (grid current through grid resistor)
        b_k = a_k + 2*R_k*(Ip + Ig)  (total current through cathode)

      Node voltages from wave variables:
        v_p = (a_p + b_p)/2 = a_p - R_p*Ip
        v_g = (a_g + b_g)/2 = a_g - R_g*Ig
        v_k = (a_k + b_k)/2 = a_k + R_k*(Ip + Ig)

      So:
        Vpk = v_p - v_k = (a_p - a_k) - R_p*Ip - R_k*(Ip + Ig)
        Vgk = v_g - v_k = (a_g - a_k) - R_g*Ig - R_k*(Ip + Ig)

      We solve iteratively: since Ig depends only on Vgk, and Ip depends
      on both Vpk and Vgk, we can use a sequential approach within each
      Newton iteration.
    """
    n_total = len(audio_in)
    out_vplate = np.zeros(n_total)
    out_ip = np.zeros(n_total)
    out_ig = np.zeros(n_total)
    out_vgrid = np.zeros(n_total)
    out_vk = np.zeros(n_total)

    prev_ip = 0.5e-3
    prev_ig = 0.0
    hp_y = 0.0
    hp_x_prev = 0.0
    z_ck = 0.0

    R_p = RP
    R_g = RG
    R_k = R_CATH

    for n in range(n_total):
        vin = audio_in[n]

        # High-pass filter (input coupling cap)
        hp_y = c_hp * hp_y + hp_gain * (vin - hp_x_prev)
        hp_x_prev = vin
        v_grid_filtered = hp_y

        # Upward pass: reflected waves from leaves
        b_plate = VB
        b_grid = v_grid_filtered

        # Cathode: parallel adaptor Rk || Ck
        b_rk = 0.0
        b_ck = z_ck
        bDiff = b_ck - b_rk
        b_cathode = b_ck - GAMMA_CATH * bDiff

        # Root incident waves
        a_p = b_plate
        a_g = b_grid
        a_k = b_cathode

        # Newton-Raphson at root
        Ip = prev_ip
        Ig = prev_ig if enable_grid_current else 0.0

        for iteration in range(30):
            # Compute node voltages
            Vpk = (a_p - a_k) - R_p * Ip - R_k * (Ip + Ig)
            Vgk = (a_g - a_k) - R_g * Ig - R_k * (Ip + Ig)

            # Evaluate models
            ip_model = koren_ip(Vpk, Vgk)
            f1 = Ip - ip_model

            if enable_grid_current:
                ig_model = grid_current(Vgk)
                f2 = Ig - ig_model
            else:
                f2 = 0.0
                ig_model = 0.0

            # Convergence check
            if abs(f1) < 1e-10 and abs(f2) < 1e-10:
                break

            # Jacobian elements
            dip_dvpk_val = koren_dip_dvpk(Vpk, Vgk)
            dip_dvgk_val = koren_dip_dvgk(Vpk, Vgk)

            # dVpk/dIp = -(R_p + R_k), dVpk/dIg = -R_k
            # dVgk/dIp = -R_k,          dVgk/dIg = -(R_g + R_k)
            #
            # df1/dIp = 1 - dip_dvpk*dVpk/dIp - dip_dvgk*dVgk/dIp
            #         = 1 + dip_dvpk*(R_p + R_k) + dip_dvgk*R_k
            # df1/dIg = -dip_dvpk*dVpk/dIg - dip_dvgk*dVgk/dIg
            #         = dip_dvpk*R_k + dip_dvgk*(R_g + R_k)

            J11 = 1.0 + dip_dvpk_val * (R_p + R_k) + dip_dvgk_val * R_k
            J12 = dip_dvpk_val * R_k + dip_dvgk_val * (R_g + R_k)

            if enable_grid_current:
                dig_dvgk = grid_current_deriv(Vgk)
                # df2/dIp = -dig_dvgk * dVgk/dIp = dig_dvgk * R_k
                # df2/dIg = 1 - dig_dvgk * dVgk/dIg = 1 + dig_dvgk*(R_g + R_k)
                J21 = dig_dvgk * R_k
                J22 = 1.0 + dig_dvgk * (R_g + R_k)

                # Solve 2x2 system: J * [dIp, dIg]^T = [f1, f2]^T
                det = J11 * J22 - J12 * J21
                if abs(det) < 1e-30:
                    break
                dIp = (J22 * f1 - J12 * f2) / det
                dIg = (J11 * f2 - J21 * f1) / det

                Ip -= dIp
                Ig -= dIg
                Ip = max(Ip, 0.0)
                Ig = max(Ig, 0.0)
            else:
                # Scalar Newton for Ip only (Ig=0)
                if abs(J11) < 1e-15:
                    break
                Ip -= f1 / J11
                Ip = max(Ip, 0.0)

        prev_ip = Ip
        if enable_grid_current:
            prev_ig = Ig

        # Reflected waves from triode
        b_p = a_p - 2.0 * R_p * Ip
        if enable_grid_current:
            b_g_out = a_g - 2.0 * R_g * Ig
        else:
            b_g_out = a_g
        b_k = a_k + 2.0 * R_k * (Ip + Ig)

        # Downward pass: update capacitor state
        a_ck = b_k + b_cathode - b_ck
        z_ck = a_ck

        # Node voltages
        v_plate = (a_p + b_p) / 2.0
        v_grid = (a_g + b_g_out) / 2.0
        v_cathode = (a_k + b_k) / 2.0

        out_vplate[n] = v_plate
        out_vgrid[n] = v_grid
        out_vk[n] = v_cathode
        out_ip[n] = Ip
        out_ig[n] = Ig

    return out_vplate, out_ip, out_ig, out_vgrid, out_vk

# =============================================================================
# Run Comparison at Multiple Drive Levels
# =============================================================================

drive_levels = [0.5, 1.0, 2.0, 4.0]
freq = 440.0
n_settle = 2000
n_audio = 4800
n_total = n_settle + n_audio
t = np.arange(n_total) / FS

results_off = {}
results_on = {}

for drive in drive_levels:
    print(f"\n{'='*60}")
    print(f"Drive level: {drive}V peak")
    print(f"{'='*60}")

    audio_in = np.zeros(n_total)
    audio_in[n_settle:] = drive * np.sin(2 * np.pi * freq * t[n_settle:])

    # Without grid current
    print("  Simulating Ig=0 ...")
    vp_off, ip_off, ig_off, vg_off, vk_off = simulate_wdf(audio_in, enable_grid_current=False)

    # With grid current
    print("  Simulating with grid current ...")
    vp_on, ip_on, ig_on, vg_on, vk_on = simulate_wdf(audio_in, enable_grid_current=True)

    # DC operating point
    sl = slice(n_settle - 100, n_settle)
    vp_dc_off = vp_off[sl].mean()
    vp_dc_on = vp_on[sl].mean()

    ac_off = vp_off[n_settle:] - vp_dc_off
    ac_on = vp_on[n_settle:] - vp_dc_on
    inp = audio_in[n_settle:]

    in_rms = np.sqrt(np.mean(inp**2))
    out_rms_off = np.sqrt(np.mean(ac_off**2))
    out_rms_on = np.sqrt(np.mean(ac_on**2))
    gain_off = 20 * np.log10(out_rms_off / (in_rms + 1e-12))
    gain_on = 20 * np.log10(out_rms_on / (in_rms + 1e-12))

    ig_peak = ig_on[n_settle:].max()

    print(f"  Ig=0:            gain={gain_off:.1f}dB  out_rms={out_rms_off:.2f}V")
    print(f"  With Ig:         gain={gain_on:.1f}dB  out_rms={out_rms_on:.2f}V")
    print(f"  Peak Ig:         {ig_peak*1e6:.1f} uA")
    print(f"  Gain difference: {gain_on - gain_off:.2f} dB")

    results_off[drive] = {
        'audio_in': audio_in, 'vplate': vp_off, 'ip': ip_off, 'ig': ig_off,
        'vgrid': vg_off, 'vk': vk_off, 'ac_out': ac_off, 'vp_dc': vp_dc_off,
        'gain_db': gain_off, 'out_rms': out_rms_off,
    }
    results_on[drive] = {
        'audio_in': audio_in, 'vplate': vp_on, 'ip': ip_on, 'ig': ig_on,
        'vgrid': vg_on, 'vk': vk_on, 'ac_out': ac_on, 'vp_dc': vp_dc_on,
        'gain_db': gain_on, 'out_rms': out_rms_on,
    }

# =============================================================================
# Compute THD for Each Configuration
# =============================================================================

def compute_thd(signal, fs, fundamental_freq, n_harmonics=8):
    """Compute THD as ratio of harmonic power to fundamental power."""
    N = len(signal)
    w = np.hanning(N)
    sp = np.abs(np.fft.rfft(signal * w))
    freqs = np.fft.rfftfreq(N, 1.0 / fs)

    # Find fundamental peak
    fund_idx = np.argmin(np.abs(freqs - fundamental_freq))
    search_width = max(3, int(N * 5 / fs))  # +/- 5 Hz

    fund_power = 0.0
    lo = max(0, fund_idx - search_width)
    hi = min(len(sp), fund_idx + search_width + 1)
    fund_power = np.max(sp[lo:hi]) ** 2

    harm_power = 0.0
    for h in range(2, n_harmonics + 1):
        h_freq = fundamental_freq * h
        h_idx = np.argmin(np.abs(freqs - h_freq))
        lo = max(0, h_idx - search_width)
        hi = min(len(sp), h_idx + search_width + 1)
        harm_power += np.max(sp[lo:hi]) ** 2

    if fund_power < 1e-30:
        return 0.0
    return 100.0 * math.sqrt(harm_power / fund_power)

print(f"\n{'='*60}")
print("THD Comparison")
print(f"{'='*60}")
print(f"{'Drive':>8s}  {'THD (Ig=0)':>12s}  {'THD (Ig on)':>12s}  {'Diff':>8s}")
print(f"{'-'*44}")

thd_off_list = []
thd_on_list = []
for drive in drive_levels:
    thd_off = compute_thd(results_off[drive]['ac_out'], FS, freq)
    thd_on = compute_thd(results_on[drive]['ac_out'], FS, freq)
    thd_off_list.append(thd_off)
    thd_on_list.append(thd_on)
    print(f"  {drive:5.1f}V    {thd_off:10.2f}%    {thd_on:10.2f}%    {thd_on-thd_off:+.2f}%")

# =============================================================================
# Comparison Plot
# =============================================================================

os.makedirs("demos", exist_ok=True)

fig, axes = plt.subplots(4, 2, figsize=(16, 18))
fig.suptitle("Grid Current Effect on 12AX7 Triode Stage\n"
             f"Ig model: Langmuir-Child, ig_max={IG_MAX*1000:.1f}mA, onset={VG_ONSET}V",
             fontsize=14, fontweight='bold')

t_audio = np.arange(n_audio) / FS * 1000  # ms
mask = t_audio < 8  # show ~3.5 cycles of 440Hz

for i, drive in enumerate(drive_levels):
    ax_wave = axes[i, 0]
    ax_fft = axes[i, 1]

    ac_off = results_off[drive]['ac_out']
    ac_on = results_on[drive]['ac_out']

    # Normalize to same scale for shape comparison
    peak = max(np.max(np.abs(ac_off)), np.max(np.abs(ac_on)), 1e-6)

    # Waveform comparison
    ax_wave.plot(t_audio[mask], ac_off[mask], 'b', linewidth=1.2,
                 label=f'Ig=0 ({results_off[drive]["gain_db"]:.1f}dB)', alpha=0.8)
    ax_wave.plot(t_audio[mask], ac_on[mask], 'r', linewidth=1.2,
                 label=f'Ig on ({results_on[drive]["gain_db"]:.1f}dB)', alpha=0.8)
    ax_wave.set_ylabel("Output (V)")
    ax_wave.set_title(f"Drive = {drive}V peak")
    ax_wave.legend(fontsize=8, loc='upper right')
    ax_wave.grid(True, alpha=0.3)
    if i == len(drive_levels) - 1:
        ax_wave.set_xlabel("Time (ms)")

    # Also show grid current on secondary axis for highest drive
    if drive == drive_levels[-1]:
        ig_sig = results_on[drive]['ig'][n_settle:]
        ax2 = ax_wave.twinx()
        ax2.plot(t_audio[mask], ig_sig[mask] * 1e6, 'g--', linewidth=0.8,
                 alpha=0.6, label='Ig (uA)')
        ax2.set_ylabel("Grid Current (uA)", color='g')
        ax2.tick_params(axis='y', labelcolor='g')
        ax2.legend(fontsize=7, loc='lower right')

    # FFT comparison
    N = len(ac_off)
    w = np.hanning(N)
    sp_off = np.abs(np.fft.rfft(ac_off * w))
    sp_on = np.abs(np.fft.rfft(ac_on * w))
    f = np.fft.rfftfreq(N, 1.0 / FS)

    ref = max(sp_off.max(), sp_on.max(), 1e-12)
    sp_off_db = 20 * np.log10(sp_off / ref + 1e-12)
    sp_on_db = 20 * np.log10(sp_on / ref + 1e-12)

    ax_fft.plot(f, sp_off_db, 'b', linewidth=0.8, alpha=0.7,
                label=f'Ig=0 (THD={thd_off_list[i]:.1f}%)')
    ax_fft.plot(f, sp_on_db, 'r', linewidth=0.8, alpha=0.7,
                label=f'Ig on (THD={thd_on_list[i]:.1f}%)')
    ax_fft.set_xlim(0, 5000)
    ax_fft.set_ylim(-80, 5)
    ax_fft.set_ylabel("dB")
    ax_fft.set_title(f"Spectrum @ {drive}V")
    ax_fft.legend(fontsize=7, loc='upper right')
    ax_fft.grid(True, alpha=0.3)
    for h in range(1, 10):
        ax_fft.axvline(freq * h, color='gray', alpha=0.2, ls='--', lw=0.5)
    if i == len(drive_levels) - 1:
        ax_fft.set_xlabel("Frequency (Hz)")

plt.tight_layout()
plt.savefig("demos/grid_current_comparison.png", dpi=150, bbox_inches='tight')
print(f"\nSaved: demos/grid_current_comparison.png")

# =============================================================================
# WAV Output (4V drive, highest grid current effect)
# =============================================================================

def save_wav(filename, signal, fs=48000):
    """Save signal as 16-bit WAV, normalized to -1dB headroom."""
    peak = np.max(np.abs(signal))
    if peak > 0:
        signal_norm = signal / peak * 0.89  # -1dB headroom
    else:
        signal_norm = signal
    data = np.clip(signal_norm * 32767, -32768, 32767).astype(np.int16)
    wavfile.write(filename, fs, data)
    print(f"Saved: {filename} ({len(data)} samples, peak={peak:.3f}V)")

ac_off_4v = results_off[4.0]['ac_out']
ac_on_4v = results_on[4.0]['ac_out']

save_wav("demos/grid_current_off_4v.wav", ac_off_4v, int(FS))
save_wav("demos/grid_current_on_4v.wav", ac_on_4v, int(FS))

# =============================================================================
# Summary
# =============================================================================

print(f"\n{'='*60}")
print("Summary")
print(f"{'='*60}")
print(f"Grid current model: Ig = {IG_MAX*1000:.1f}mA * max(0, Vgk - {VG_ONSET})^1.5")
print(f"Effect increases with drive level:")
for i, drive in enumerate(drive_levels):
    ig_peak = results_on[drive]['ig'][n_settle:].max()
    gain_diff = results_on[drive]['gain_db'] - results_off[drive]['gain_db']
    thd_diff = thd_on_list[i] - thd_off_list[i]
    print(f"  {drive}V: peak Ig = {ig_peak*1e6:.1f}uA, "
          f"gain change = {gain_diff:+.2f}dB, "
          f"THD change = {thd_diff:+.2f}%")
print(f"\nFiles:")
print(f"  demos/grid_current_comparison.png  -- waveform + spectrum comparison")
print(f"  demos/grid_current_off_4v.wav      -- 4V drive, Ig=0")
print(f"  demos/grid_current_on_4v.wav       -- 4V drive, with grid current")
print("Done.")
