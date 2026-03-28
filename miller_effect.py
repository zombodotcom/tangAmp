r"""
miller_effect.py
Miller effect and parasitic capacitance modeling for WDF triode stages.

Real triodes have small inter-electrode capacitances:
  - Cgk (grid-cathode): directly across the input
  - Cgp (grid-plate): multiplied by stage gain via Miller effect
  - Cpk (plate-cathode): shunts the output (minor effect)

The Miller effect multiplies Cgp by (1 + Av), creating a significant input
capacitance that forms a low-pass filter with the source impedance:

  C_miller = Cgp * (1 + Av)
  f_miller = 1 / (2*pi * R_source * C_miller)

For a 12AX7 with Av=50 and R_source=100k:
  C_miller = 1.7pF * 51 = 86.7pF
  f_miller = 1 / (2*pi * 100e3 * 86.7e-12) = 18.4 kHz

This is why multi-stage high-gain amps sound progressively "darker" --
each stage rolls off treble via its Miller capacitance. A 3-stage cascade
at f_miller = 18 kHz is already -3 dB around 10 kHz.

Outputs:
  demos/miller_effect.png       -- frequency response comparison plot
  demos/miller_off_3stage.wav   -- 3-stage cascade without Miller effect
  demos/miller_on_3stage.wav    -- 3-stage cascade with Miller effect
  miller_verilog_spec.txt       -- Verilog implementation notes
"""

import numpy as np
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

try:
    from scipy.io import wavfile
    from scipy.signal import freqz
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

from amp_sim import (
    FS, TUBES, koren_ip, koren_dip_dvpk, koren_dip_dvgk,
    simulate_preamp_stage, save_wav,
    PREAMP_VB, PREAMP_RP, PREAMP_RG, PREAMP_RK, PREAMP_CIN, PREAMP_CK,
)

# =============================================================================
# Parasitic Capacitances (datasheet values)
# =============================================================================

PARASITIC_CAPS = {
    # Cgk (pF), Cgp (pF), Cpk (pF), typical Av (voltage gain at operating point)
    "12AX7": dict(Cgk=1.6e-12, Cgp=1.7e-12, Cpk=0.46e-12, Av=50.0),
    "12AU7": dict(Cgk=1.5e-12, Cgp=1.5e-12, Cpk=0.50e-12, Av=17.0),
    "6SL7":  dict(Cgk=3.0e-12, Cgp=3.2e-12, Cpk=1.1e-12,  Av=44.0),
    "EL34":  dict(Cgk=10.0e-12, Cgp=0.9e-12, Cpk=7.0e-12,  Av=5.0),
    "6L6":   dict(Cgk=10.0e-12, Cgp=0.9e-12, Cpk=7.0e-12,  Av=5.0),
    "300B":  dict(Cgk=7.5e-12,  Cgp=8.5e-12, Cpk=4.0e-12,  Av=3.5),
}


def compute_miller(tube_name, r_source=None):
    """
    Compute the Miller capacitance and resulting -3dB frequency.

    Args:
        tube_name: key into PARASITIC_CAPS
        r_source: source impedance in ohms (default: plate resistor of prev stage)

    Returns:
        dict with c_miller, c_input_total, f_miller, r_source
    """
    caps = PARASITIC_CAPS[tube_name]
    Cgk = caps['Cgk']
    Cgp = caps['Cgp']
    Cpk = caps['Cpk']
    Av = caps['Av']

    # Miller capacitance: Cgp reflected to input
    c_miller = Cgp * (1.0 + Av)

    # Total input capacitance seen at grid
    c_input_total = Cgk + c_miller

    # Source impedance: typically the plate resistor of the previous stage
    if r_source is None:
        r_source = PREAMP_RP  # 100k

    # -3dB frequency of the resulting RC low-pass filter
    f_miller = 1.0 / (2.0 * np.pi * r_source * c_input_total)

    return dict(
        c_miller=c_miller,
        c_input_total=c_input_total,
        f_miller=f_miller,
        r_source=r_source,
        Cgk=Cgk, Cgp=Cgp, Cpk=Cpk, Av=Av,
    )


def miller_lpf_coeffs(f_3dB, fs=FS):
    """
    Compute 1st-order IIR low-pass filter coefficients for the Miller effect.
    Uses bilinear transform of H(s) = 1 / (1 + s/(2*pi*f_3dB)).

    Returns (b0, b1, a1) for: y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
    """
    wc = 2.0 * np.pi * f_3dB
    # Bilinear pre-warp
    wd = 2.0 * fs * np.tan(wc / (2.0 * fs))
    # Bilinear transform: s = 2*fs*(z-1)/(z+1)
    # H(z) = (wd/(2fs+wd)) * (1 + z^-1) / (1 - ((2fs-wd)/(2fs+wd))*z^-1)
    alpha = wd / (2.0 * fs + wd)
    b0 = alpha
    b1 = alpha
    a1 = -(2.0 * fs - wd) / (2.0 * fs + wd)  # note: negative for y[n] = b0*x + b1*x1 - a1*y1
    return b0, b1, a1


def apply_miller_filter(signal, f_3dB, fs=FS):
    """Apply the Miller effect low-pass filter to a signal."""
    b0, b1, a1 = miller_lpf_coeffs(f_3dB, fs)
    out = np.zeros_like(signal)
    x_prev = 0.0
    y_prev = 0.0
    for n in range(len(signal)):
        out[n] = b0 * signal[n] + b1 * x_prev - a1 * y_prev
        x_prev = signal[n]
        y_prev = out[n]
    return out


def frequency_response_db(f_3dB, fs=FS, n_points=4096):
    """Compute the magnitude frequency response in dB."""
    b0, b1, a1 = miller_lpf_coeffs(f_3dB, fs)
    # Transfer function: H(z) = (b0 + b1*z^-1) / (1 + a1*z^-1)
    w = np.linspace(0, np.pi, n_points)
    z = np.exp(1j * w)
    H = (b0 + b1 / z) / (1.0 + a1 / z)
    freqs = w * fs / (2.0 * np.pi)
    mag_db = 20.0 * np.log10(np.abs(H) + 1e-30)
    return freqs, mag_db


# =============================================================================
# Main: compute, plot, render audio
# =============================================================================

def main():
    os.makedirs('demos', exist_ok=True)

    # -------------------------------------------------------------------------
    # 1. Print Miller capacitance summary for all tubes
    # -------------------------------------------------------------------------
    print("=" * 72)
    print("Miller Effect Summary")
    print("=" * 72)
    print(f"{'Tube':>8s}  {'Cgk':>7s}  {'Cgp':>7s}  {'Av':>5s}  {'C_miller':>10s}  {'C_total':>10s}  {'f_miller':>10s}")
    print("-" * 72)

    tube_names = ['12AX7', '12AU7', '6SL7', 'EL34', '6L6', '300B']
    miller_data = {}
    for name in tube_names:
        d = compute_miller(name)
        miller_data[name] = d
        print(f"{name:>8s}  {d['Cgk']*1e12:6.1f}pF  {d['Cgp']*1e12:6.1f}pF  {d['Av']:5.1f}"
              f"  {d['c_miller']*1e12:8.1f}pF  {d['c_input_total']*1e12:8.1f}pF"
              f"  {d['f_miller']:8.0f} Hz")

    # -------------------------------------------------------------------------
    # 2. Frequency response plot: single stage for each preamp tube
    # -------------------------------------------------------------------------
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Panel (0,0): single-stage response for all preamp tubes
    ax = axes[0, 0]
    preamp_tubes = ['12AX7', '12AU7', '6SL7']
    for name in preamp_tubes:
        d = miller_data[name]
        freqs, mag_db = frequency_response_db(d['f_miller'])
        ax.semilogx(freqs, mag_db, label=f"{name} (f_3dB={d['f_miller']:.0f} Hz)")
    ax.set_xlim(20, 24000)
    ax.set_ylim(-12, 1)
    ax.set_xlabel('Frequency (Hz)')
    ax.set_ylabel('Magnitude (dB)')
    ax.set_title('Miller LPF per Stage (R_source=100k)')
    ax.axhline(-3, color='gray', linestyle='--', alpha=0.5, label='-3 dB')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel (0,1): 12AX7 cascaded 1, 2, 3 stages
    ax = axes[0, 1]
    d = miller_data['12AX7']
    freqs, mag_1 = frequency_response_db(d['f_miller'])
    mag_2 = 2.0 * mag_1
    mag_3 = 3.0 * mag_1
    ax.semilogx(freqs, mag_1, label='1 stage')
    ax.semilogx(freqs, mag_2, label='2 stages')
    ax.semilogx(freqs, mag_3, label='3 stages')
    ax.set_xlim(20, 24000)
    ax.set_ylim(-18, 1)
    ax.set_xlabel('Frequency (Hz)')
    ax.set_ylabel('Magnitude (dB)')
    ax.set_title('12AX7 Cascade: Cumulative Miller Rolloff')
    ax.axhline(-3, color='gray', linestyle='--', alpha=0.5, label='-3 dB')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel (1,0): effect of source impedance on 12AX7
    ax = axes[1, 0]
    r_sources = [47e3, 100e3, 220e3, 470e3]
    for rs in r_sources:
        d_rs = compute_miller('12AX7', r_source=rs)
        freqs, mag = frequency_response_db(d_rs['f_miller'])
        ax.semilogx(freqs, mag, label=f'R_src={rs/1e3:.0f}k (f={d_rs["f_miller"]:.0f} Hz)')
    ax.set_xlim(20, 24000)
    ax.set_ylim(-12, 1)
    ax.set_xlabel('Frequency (Hz)')
    ax.set_ylabel('Magnitude (dB)')
    ax.set_title('12AX7 Miller: Effect of Source Impedance')
    ax.axhline(-3, color='gray', linestyle='--', alpha=0.5, label='-3 dB')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Panel (1,1): power tubes
    ax = axes[1, 1]
    power_tubes = ['6L6', 'EL34', '300B']
    for name in power_tubes:
        d = miller_data[name]
        freqs, mag = frequency_response_db(d['f_miller'])
        ax.semilogx(freqs, mag, label=f"{name} (f_3dB={d['f_miller']:.0f} Hz)")
    ax.set_xlim(20, 24000)
    ax.set_ylim(-6, 1)
    ax.set_xlabel('Frequency (Hz)')
    ax.set_ylabel('Magnitude (dB)')
    ax.set_title('Power Tube Miller LPF (R_source=100k)')
    ax.axhline(-3, color='gray', linestyle='--', alpha=0.5, label='-3 dB')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.suptitle('Miller Effect: Parasitic Capacitance Frequency Response', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig('demos/miller_effect.png', dpi=150)
    plt.close()
    print(f"\nSaved demos/miller_effect.png")

    # -------------------------------------------------------------------------
    # 3. Audio comparison: 3-stage cascade with and without Miller effect
    # -------------------------------------------------------------------------
    print("\nRendering 3-stage cascade audio...")

    duration = 3.0
    n_samples = int(duration * FS)

    # Guitar-like input: mix of harmonics with gentle attack envelope
    t = np.arange(n_samples) / FS
    f0 = 82.41  # low E
    audio_in = np.zeros(n_samples)
    for h, amp in [(1, 1.0), (2, 0.7), (3, 0.5), (4, 0.3), (5, 0.2), (6, 0.12), (7, 0.08)]:
        audio_in += amp * np.sin(2.0 * np.pi * f0 * h * t)
    # Normalize and apply envelope
    audio_in /= np.max(np.abs(audio_in))
    env = np.minimum(t / 0.01, 1.0) * np.exp(-t / 1.5)
    audio_in *= env * 0.5  # moderate drive

    # Add a second note halfway through
    t2_start = int(1.5 * FS)
    f1 = 110.0  # A2
    note2 = np.zeros(n_samples)
    for h, amp in [(1, 1.0), (2, 0.6), (3, 0.4), (4, 0.2), (5, 0.15)]:
        note2 += amp * np.sin(2.0 * np.pi * f1 * h * t)
    note2 /= np.max(np.abs(note2))
    env2 = np.zeros(n_samples)
    t2 = t[t2_start:]
    env2[t2_start:] = np.minimum((t2 - t2[0]) / 0.01, 1.0) * np.exp(-(t2 - t2[0]) / 1.0)
    audio_in += note2 * env2 * 0.4

    settle = 2000

    # --- Without Miller effect: straight cascade ---
    sig_no_miller = audio_in.copy()
    interstage_atten = 10.0 ** (-20.0 / 20.0)  # -20 dB between stages

    for stage in range(3):
        padded = np.concatenate([np.zeros(settle), sig_no_miller])
        vp, vp_dc = simulate_preamp_stage(padded, tube_name='12AX7', use_bypass=True, settle=settle)
        vp_ac = vp[settle:] - vp_dc
        sig_no_miller = vp_ac * interstage_atten

    # Normalize
    peak = np.max(np.abs(sig_no_miller))
    if peak > 0:
        sig_no_miller = sig_no_miller / peak * 0.9

    save_wav('demos/miller_off_3stage.wav', sig_no_miller, FS)
    print("  Saved demos/miller_off_3stage.wav")

    # --- With Miller effect: LPF between each stage ---
    sig_miller = audio_in.copy()
    d_12ax7 = miller_data['12AX7']

    for stage in range(3):
        # Apply Miller LPF at input of this stage (models R_source from previous stage)
        if stage > 0:
            sig_miller = apply_miller_filter(sig_miller, d_12ax7['f_miller'])

        padded = np.concatenate([np.zeros(settle), sig_miller])
        vp, vp_dc = simulate_preamp_stage(padded, tube_name='12AX7', use_bypass=True, settle=settle)
        vp_ac = vp[settle:] - vp_dc
        sig_miller = vp_ac * interstage_atten

    # Normalize
    peak = np.max(np.abs(sig_miller))
    if peak > 0:
        sig_miller = sig_miller / peak * 0.9

    save_wav('demos/miller_on_3stage.wav', sig_miller, FS)
    print("  Saved demos/miller_on_3stage.wav")

    # -------------------------------------------------------------------------
    # 4. Compare spectra of the two versions
    # -------------------------------------------------------------------------
    print("\nMiller effect comparison:")
    # Measure spectral energy above 5 kHz vs total
    def high_freq_ratio(sig, cutoff_hz=5000):
        spectrum = np.abs(np.fft.rfft(sig))
        freqs = np.fft.rfftfreq(len(sig), 1.0 / FS)
        total = np.sum(spectrum ** 2)
        hf = np.sum(spectrum[freqs >= cutoff_hz] ** 2)
        return hf / (total + 1e-30)

    hf_off = high_freq_ratio(sig_no_miller)
    hf_on = high_freq_ratio(sig_miller)
    print(f"  HF energy (>5kHz) without Miller: {hf_off*100:.2f}%")
    print(f"  HF energy (>5kHz) with Miller:    {hf_on*100:.2f}%")
    print(f"  HF reduction:                     {(1 - hf_on/hf_off)*100:.1f}%")

    print("\nDone.")


if __name__ == '__main__':
    main()
