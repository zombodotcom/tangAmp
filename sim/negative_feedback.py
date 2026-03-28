r"""
negative_feedback.py
Global negative feedback loop simulation for tube amplifiers.

Real tube amps feed a fraction of the output transformer secondary back to
the cathode of a preamp stage (typically the second stage or the first stage
cathode). This:
  - Reduces overall gain (more NFB = more headroom)
  - Tightens bass response (feedback linearizes the low end)
  - Reduces THD at low-to-moderate drive levels
  - Changes the amp's "feel" (spongy vs tight)

Classic NFB amounts:
  - Fender (heavy NFB ~0.3): clean headroom, tight bass, low distortion
  - Marshall (moderate NFB ~0.1): balanced breakup
  - Vox AC30 (no NFB = 0.0): maximum distortion, loose/saggy feel

The feedback signal is delayed by 1 sample (causality) and subtracted from
the input of the first preamp stage.
"""

import numpy as np
import os
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

try:
    from scipy.io import wavfile
    from scipy.signal import firwin2
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

# Import from amp_sim for the building blocks
from amp_sim import (
    FS, TUBES, koren_ip, koren_dip_dvpk, koren_dip_dvgk,
    simulate_preamp_stage, simulate_power_amp,
    tone_stack, make_cabinet_ir, apply_cabinet, save_wav,
    PREAMP_VB, PREAMP_RP, PREAMP_RG, PREAMP_RK, PREAMP_CIN, PREAMP_CK,
)


# =============================================================================
# Global Negative Feedback Amp Simulation
# =============================================================================

def simulate_with_nfb(audio_in, preamp_stages=2, preamp_tube='12AX7',
                      interstage_atten_db=20.0, use_bypass=True,
                      tone_bass=5, tone_mid=5, tone_treble=5,
                      power_tube='6L6', power_vb=400.0, power_rp=2000.0,
                      power_rk=250.0, power_sag=0.3, power_clip=200.0,
                      cabinet='1x12_open', input_gain=1.0,
                      master_vol=0.25, nfb_amount=0.1,
                      settle=2000, fs=48000.0):
    """
    Full amp simulation with global negative feedback loop.

    NFB takes a fraction of the power amp output (post-transformer, pre-cabinet)
    and subtracts it from the input of the first preamp stage, with a 1-sample
    delay. The feedback signal is normalized by the output transformer turns
    ratio so it is at the same voltage scale as the guitar input.

    The implementation uses iterative batch processing: run the full chain,
    compute the NFB correction signal, re-run with the corrected input.
    Two iterations converge well for NFB amounts up to 0.3.

    Parameters
    ----------
    audio_in : ndarray
        Input signal (including settle samples at start).
    preamp_stages : int
        Number of cascaded 12AX7 preamp stages.
    nfb_amount : float
        Feedback coefficient. 0.0 = no feedback (Vox-like),
        0.1 = moderate (Marshall), 0.3 = heavy (Fender).
        This represents the fraction of the output transformer secondary
        voltage fed back, after the transformer ratio normalization.
    settle : int
        Number of settle samples at start of audio_in.

    Returns
    -------
    final_out : ndarray
        Output signal (settle samples removed).
    """
    n_total = len(audio_in)

    # Pre-build cabinet IR
    cab_ir = make_cabinet_ir(cabinet, n_taps=257, fs=int(fs))

    # Scale input by gain
    x_raw = audio_in * input_gain

    # The NFB loop in a real amp works like this:
    #   v_preamp_in = v_guitar - beta * v_output
    # where beta = nfb_amount, and v_output is the transformer secondary
    # voltage (already scaled to guitar-level voltages by the turns ratio).
    #
    # We simulate this by running the chain, measuring the effective gain,
    # then computing the closed-loop equivalent. For a linear system:
    #   A_closed = A_open / (1 + beta * A_open)
    # For the nonlinear triode, we iterate: run the chain, compute NFB
    # correction, re-run. Converges in 2-3 iterations for beta < 0.5.

    # Helper: run the full chain on a given input signal
    def run_chain(x_in):
        interstage_lin = 10.0 ** (interstage_atten_db / 20.0)
        signal = x_in.copy()

        for stage in range(preamp_stages):
            vplate, vp_dc = simulate_preamp_stage(
                signal, tube_name=preamp_tube, use_bypass=use_bypass,
                settle=settle)
            ac_out = vplate - vp_dc
            if stage < preamp_stages - 1:
                signal = ac_out / interstage_lin
            else:
                signal = ac_out

        # Tone stack (on audio portion)
        preamp_out = signal[settle:]
        toned = tone_stack(preamp_out, tone_bass, tone_mid, tone_treble,
                           fs=fs)

        # Master volume
        toned = toned * master_vol

        # Power amp
        power_in = np.concatenate([np.zeros(settle), toned])
        power_out, _ = simulate_power_amp(
            power_in, tube_name=power_tube, vb=power_vb, rp=power_rp,
            rk=power_rk, sag_amount=power_sag, clip_level=power_clip,
            fs=fs, settle=settle)

        # power_out is AC-coupled and soft-clipped already
        return power_out  # full length including settle

    if nfb_amount <= 0.0:
        # No feedback -- single pass
        power_out = run_chain(x_raw)
        power_audio = power_out[settle:]
        cab_out = apply_cabinet(power_audio, cab_ir)
        return cab_out

    # --- Iterative NFB ---
    # The feedback signal from the power amp output needs to be scaled
    # to input-level voltages. In a real amp, the output transformer
    # turns ratio (e.g., 25:1) does this. We normalize by computing
    # the ratio of output RMS to input RMS from the first (open-loop) pass.
    #
    # Phase: with 2 preamp stages + 1 power amp stage = 3 inversions (odd),
    # the output is inverted relative to the input. In real amps, the
    # output transformer is wired to correct this, so the NFB signal is
    # in the correct polarity. We detect and correct for this automatically.
    #
    # First pass: open-loop
    power_out_0 = run_chain(x_raw)
    power_audio_0 = power_out_0[settle:]

    # Compute normalization: scale feedback to input level
    in_rms = np.sqrt(np.mean(x_raw[settle:] ** 2)) + 1e-12
    out_rms = np.sqrt(np.mean(power_audio_0 ** 2)) + 1e-12
    turns_ratio = out_rms / in_rms  # effective voltage gain

    # Detect phase: if output is inverted relative to input, flip the
    # feedback polarity (as the output transformer would in a real amp)
    corr = np.corrcoef(x_raw[settle:settle + 4000],
                       power_out_0[settle:settle + 4000])[0, 1]
    phase_sign = 1.0 if corr >= 0 else -1.0

    # NFB iterations: re-run chain with feedback applied to input
    # 2 iterations is sufficient for convergence at NFB <= 0.3
    for iteration in range(2):
        if iteration == 0:
            pa_out = power_out_0
        else:
            pa_out = power_out_i

        # Normalize to input voltage scale, apply 1-sample delay,
        # correct phase, and scale by nfb_amount
        fb_normalized = np.zeros(n_total)
        fb_normalized[1:] = phase_sign * pa_out[:-1] / turns_ratio
        nfb_signal = nfb_amount * fb_normalized

        # Negative feedback: subtract from input
        x_with_nfb = x_raw - nfb_signal
        power_out_i = run_chain(x_with_nfb)

    # Final output through cabinet
    power_audio = power_out_i[settle:]
    cab_out = apply_cabinet(power_audio, cab_ir)
    return cab_out


# =============================================================================
# THD Measurement
# =============================================================================

def measure_thd(signal, fundamental_freq, fs=48000.0, n_harmonics=8):
    """
    Measure Total Harmonic Distortion of a signal.

    Returns THD as a percentage and individual harmonic levels in dB.
    """
    N = len(signal)
    w = np.hanning(N)
    sp = np.abs(np.fft.rfft(signal * w))
    freqs = np.fft.rfftfreq(N, 1.0 / fs)

    def get_harmonic_power(harm_freq, bandwidth=5.0):
        mask = np.abs(freqs - harm_freq) < bandwidth
        if not np.any(mask):
            return 0.0
        return np.sum(sp[mask] ** 2)

    fund_power = get_harmonic_power(fundamental_freq)
    if fund_power < 1e-20:
        return 0.0, []

    harm_power_total = 0.0
    harm_levels = []
    for h in range(2, n_harmonics + 1):
        hp = get_harmonic_power(fundamental_freq * h)
        harm_power_total += hp
        level_db = 10.0 * np.log10(hp / fund_power + 1e-20)
        harm_levels.append((h, level_db))

    thd = np.sqrt(harm_power_total / fund_power) * 100.0
    return thd, harm_levels


# =============================================================================
# Main: NFB Comparison Demos
# =============================================================================

if __name__ == "__main__":
    demos_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "demos")
    os.makedirs(demos_dir, exist_ok=True)

    # Input: 82Hz E2 note (single sine for clean THD measurement), 0.5 second
    duration = 0.5
    n_settle = 2000
    n_audio = int(duration * FS)
    n_total = n_settle + n_audio
    t = np.arange(n_total) / FS

    # Single sine for THD measurement
    audio_sine = np.zeros(n_total)
    audio_sine[n_settle:] = 0.15 * np.sin(2.0 * np.pi * 82.0 * t[n_settle:])

    # Power chord for WAV demos (1 second -- kept short for sim speed)
    duration_wav = 1.0
    n_audio_wav = int(duration_wav * FS)
    n_total_wav = n_settle + n_audio_wav
    t_wav = np.arange(n_total_wav) / FS

    audio_chord = np.zeros(n_total_wav)
    audio_chord[n_settle:] = (
        0.15 * np.sin(2.0 * np.pi * 82.0 * t_wav[n_settle:]) +
        0.15 * np.sin(2.0 * np.pi * 123.0 * t_wav[n_settle:]) +
        0.15 * np.sin(2.0 * np.pi * 165.0 * t_wav[n_settle:]))

    # =========================================================================
    # 1. NFB sweep comparison (same amp, varying NFB)
    # =========================================================================
    print("=" * 60)
    print("Negative Feedback Comparison")
    print("=" * 60)

    nfb_levels = [0.0, 0.05, 0.1, 0.2, 0.3]
    sweep_results = []

    # Common amp settings for the sweep
    sweep_params = dict(
        preamp_stages=2, preamp_tube='12AX7',
        interstage_atten_db=20.0, use_bypass=True,
        tone_bass=5, tone_mid=5, tone_treble=5,
        power_tube='6L6', power_vb=400.0, power_rp=2000.0,
        power_rk=250.0, power_sag=0.3, power_clip=200.0,
        cabinet='1x12_open', input_gain=8.0, master_vol=0.25,
        settle=n_settle,
    )

    for nfb in nfb_levels:
        print(f"\n  NFB = {nfb:.2f} ...")
        out = simulate_with_nfb(audio_sine, nfb_amount=nfb, **sweep_params)

        peak = np.max(np.abs(out))
        rms = np.sqrt(np.mean(out ** 2))
        thd, harmonics = measure_thd(out, 82.0, fs=FS)
        gain_db = 20.0 * np.log10(rms + 1e-12) - 20.0 * np.log10(0.15 + 1e-12)

        print(f"    Peak={peak:.1f}V, RMS={rms:.2f}V, "
              f"Gain={gain_db:.1f}dB, THD={thd:.1f}%")

        sweep_results.append({
            'nfb': nfb, 'peak': peak, 'rms': rms,
            'gain_db': gain_db, 'thd': thd,
            'harmonics': harmonics, 'signal': out,
        })

    # =========================================================================
    # 2. Amp presets with NFB
    # =========================================================================
    print(f"\n{'='*60}")
    print("Amp Presets with Negative Feedback")
    print("=" * 60)

    presets = {
        'fender': {
            'name': 'Fender Deluxe (NFB=0.3)',
            'params': dict(
                preamp_stages=2, preamp_tube='12AX7',
                interstage_atten_db=20.0, use_bypass=True,
                tone_bass=6, tone_mid=5, tone_treble=7,
                power_tube='6L6', power_vb=400.0, power_rp=2000.0,
                power_rk=250.0, power_sag=0.3, power_clip=200.0,
                cabinet='1x12_open', input_gain=8.0, master_vol=0.25,
                nfb_amount=0.3, settle=n_settle,
            ),
        },
        'marshall': {
            'name': 'Marshall JCM800 (NFB=0.1)',
            'params': dict(
                preamp_stages=3, preamp_tube='12AX7',
                interstage_atten_db=16.0, use_bypass=True,
                tone_bass=5, tone_mid=8, tone_treble=6,
                power_tube='EL34', power_vb=450.0, power_rp=1700.0,
                power_rk=200.0, power_sag=0.2, power_clip=180.0,
                cabinet='4x12_closed', input_gain=12.0, master_vol=0.35,
                nfb_amount=0.1, settle=n_settle,
            ),
        },
        'vox': {
            'name': 'Vox AC30 (NFB=0.0)',
            'params': dict(
                preamp_stages=2, preamp_tube='12AX7',
                interstage_atten_db=18.0, use_bypass=True,
                tone_bass=7, tone_mid=4, tone_treble=8,
                power_tube='EL34', power_vb=380.0, power_rp=1800.0,
                power_rk=220.0, power_sag=0.25, power_clip=170.0,
                cabinet='1x12_open', input_gain=10.0, master_vol=0.35,
                nfb_amount=0.0, settle=n_settle,
            ),
        },
    }

    preset_results = {}

    for key, preset in presets.items():
        name = preset['name']
        p = preset['params']
        nfb = p['nfb_amount']
        print(f"\n  {name} ...")

        # THD measurement (single sine, 1 sec)
        out_thd = simulate_with_nfb(audio_sine, **p)
        thd, harmonics = measure_thd(out_thd, 82.0, fs=FS)
        peak = np.max(np.abs(out_thd))
        rms = np.sqrt(np.mean(out_thd ** 2))
        print(f"    Peak={peak:.1f}V, RMS={rms:.2f}V, THD={thd:.1f}%")

        # WAV demo (power chord, 3 sec)
        out_wav = simulate_with_nfb(audio_chord, **p)

        wav_path = os.path.join(demos_dir, f"nfb_{key}_{nfb:.1f}.wav")
        save_wav(wav_path, out_wav, int(FS))

        preset_results[key] = {
            'name': f"{key.title()} (NFB={nfb})",
            'nfb': nfb, 'thd': thd, 'peak': peak, 'rms': rms,
            'harmonics': harmonics, 'signal_thd': out_thd,
            'signal_wav': out_wav,
        }

    # =========================================================================
    # 3. Comparison Plot: Gain and THD vs NFB Amount
    # =========================================================================
    print(f"\n{'='*60}")
    print("Generating comparison plot...")
    print("=" * 60)

    fig = plt.figure(figsize=(18, 16))

    # --- Top row: Gain and THD vs NFB ---
    ax1 = fig.add_subplot(3, 2, 1)
    nfbs = [r['nfb'] for r in sweep_results]
    gains = [r['gain_db'] for r in sweep_results]
    ax1.plot(nfbs, gains, 'o-', color='#1f77b4', linewidth=2, markersize=8)
    ax1.set_xlabel("NFB Amount")
    ax1.set_ylabel("Gain (dB)")
    ax1.set_title("Gain vs Negative Feedback")
    ax1.grid(True, alpha=0.3)
    # Annotate amp types
    ax1.axvspan(-0.01, 0.02, alpha=0.1, color='red', label='Vox (no NFB)')
    ax1.axvspan(0.08, 0.12, alpha=0.1, color='orange', label='Marshall')
    ax1.axvspan(0.25, 0.35, alpha=0.1, color='blue', label='Fender')
    ax1.legend(fontsize=8)

    ax2 = fig.add_subplot(3, 2, 2)
    thds = [r['thd'] for r in sweep_results]
    ax2.plot(nfbs, thds, 's-', color='#d62728', linewidth=2, markersize=8)
    ax2.set_xlabel("NFB Amount")
    ax2.set_ylabel("THD (%)")
    ax2.set_title("Total Harmonic Distortion vs Negative Feedback")
    ax2.grid(True, alpha=0.3)
    ax2.axvspan(-0.01, 0.02, alpha=0.1, color='red')
    ax2.axvspan(0.08, 0.12, alpha=0.1, color='orange')
    ax2.axvspan(0.25, 0.35, alpha=0.1, color='blue')

    # --- Middle row: Waveform comparison (NFB=0 vs NFB=0.3) ---
    t_audio = np.arange(n_audio) / FS
    mask = t_audio < 0.05  # 50ms

    ax3 = fig.add_subplot(3, 2, 3)
    sig_no_nfb = sweep_results[0]['signal']  # NFB=0
    sig_hi_nfb = sweep_results[-1]['signal']  # NFB=0.3
    # Normalize for shape comparison
    norm_no = sig_no_nfb / (np.max(np.abs(sig_no_nfb)) + 1e-12)
    norm_hi = sig_hi_nfb / (np.max(np.abs(sig_hi_nfb)) + 1e-12)
    ax3.plot(t_audio[mask] * 1000, norm_no[mask], color='red', linewidth=0.8,
             alpha=0.8, label='NFB=0.0 (Vox)')
    ax3.plot(t_audio[mask] * 1000, norm_hi[mask], color='blue', linewidth=0.8,
             alpha=0.8, label='NFB=0.3 (Fender)')
    ax3.set_xlabel("Time (ms)")
    ax3.set_ylabel("Normalized Amplitude")
    ax3.set_title("Waveform Shape: No NFB vs Heavy NFB")
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)

    # --- Middle row right: Spectrum comparison ---
    ax4 = fig.add_subplot(3, 2, 4)
    for i, r in enumerate(sweep_results):
        sig = r['signal']
        N = len(sig)
        w = np.hanning(N)
        sp = np.abs(np.fft.rfft(sig * w))
        sp_db = 20.0 * np.log10(sp / (np.max(sp) + 1e-12) + 1e-12)
        f = np.fft.rfftfreq(N, 1.0 / FS)
        alpha = 0.4 + 0.15 * i
        ax4.plot(f, sp_db, linewidth=0.8, alpha=alpha,
                 label=f"NFB={r['nfb']:.2f}")
    ax4.set_xlim(0, 4000)
    ax4.set_ylim(-80, 5)
    ax4.set_xlabel("Frequency (Hz)")
    ax4.set_ylabel("dB (normalized)")
    ax4.set_title("Spectrum vs NFB Amount (82Hz fundamental)")
    ax4.legend(fontsize=7)
    ax4.grid(True, alpha=0.3)
    for freq in [82, 164, 246, 328, 410]:
        ax4.axvline(freq, color='gray', alpha=0.2, ls='--', lw=0.5)

    # --- Bottom row: Preset waveforms ---
    ax5 = fig.add_subplot(3, 2, 5)
    colors_preset = {'fender': '#1f77b4', 'marshall': '#d62728', 'vox': '#2ca02c'}
    for key, pr in preset_results.items():
        sig = pr['signal_thd']
        norm = sig / (np.max(np.abs(sig)) + 1e-12)
        ax5.plot(t_audio[mask] * 1000, norm[mask], color=colors_preset[key],
                 linewidth=0.8, alpha=0.8, label=pr['name'])
    ax5.set_xlabel("Time (ms)")
    ax5.set_ylabel("Normalized Amplitude")
    ax5.set_title("Amp Preset Waveforms (82Hz)")
    ax5.legend(fontsize=7)
    ax5.grid(True, alpha=0.3)

    # --- Bottom row right: Preset spectra ---
    ax6 = fig.add_subplot(3, 2, 6)
    for key, pr in preset_results.items():
        sig = pr['signal_thd']
        N = len(sig)
        w = np.hanning(N)
        sp = np.abs(np.fft.rfft(sig * w))
        sp_db = 20.0 * np.log10(sp / (np.max(sp) + 1e-12) + 1e-12)
        f = np.fft.rfftfreq(N, 1.0 / FS)
        ax6.plot(f, sp_db, color=colors_preset[key], linewidth=0.8,
                 alpha=0.8, label=f"{pr['name']} THD={pr['thd']:.1f}%")
    ax6.set_xlim(0, 4000)
    ax6.set_ylim(-80, 5)
    ax6.set_xlabel("Frequency (Hz)")
    ax6.set_ylabel("dB (normalized)")
    ax6.set_title("Amp Preset Spectra")
    ax6.legend(fontsize=7)
    ax6.grid(True, alpha=0.3)

    fig.suptitle("Negative Feedback Loop -- Effect on Tube Amp Character",
                 fontsize=15, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.96])

    plot_path = os.path.join(demos_dir, "nfb_comparison.png")
    plt.savefig(plot_path, dpi=150)
    print(f"\nSaved: {plot_path}")
    plt.close()

    # =========================================================================
    # Summary
    # =========================================================================
    print(f"\n{'='*60}")
    print("NFB Sweep Summary")
    print("=" * 60)
    print(f"{'NFB':>6} {'Gain(dB)':>10} {'THD(%)':>10} {'Peak(V)':>10} {'RMS(V)':>10}")
    print("-" * 50)
    for r in sweep_results:
        print(f"{r['nfb']:>6.2f} {r['gain_db']:>10.1f} {r['thd']:>10.1f} "
              f"{r['peak']:>10.1f} {r['rms']:>10.2f}")

    print(f"\n{'='*60}")
    print("Preset Summary")
    print("=" * 60)
    for key, pr in preset_results.items():
        print(f"  {pr['name']}: THD={pr['thd']:.1f}%, "
              f"Peak={pr['peak']:.1f}V, RMS={pr['rms']:.2f}V")

    print("\nDone.")
