r"""
presence_resonance.py
Presence and Resonance controls for the power amp feedback loop.

Real tube amps (Fender, Marshall, Mesa, Peavey) place frequency-shaping
networks in the negative feedback path between the output transformer
secondary and the cathode of an early preamp stage. This creates
frequency-dependent feedback that shapes the power amp's frequency response:

  - **Presence**: A capacitor across the NFB resistor rolls off high-frequency
    feedback, causing a high-frequency BOOST in the output. Found on nearly
    all Fender, Marshall, and Mesa amps. Shelf around 3-5kHz, 0 to +10dB.

  - **Resonance**: An inductor (or active RC filter) in the NFB path reduces
    low-frequency feedback, causing a low-frequency BOOST. Found on Mesa
    Rectifiers, Peavey 5150, some Marshalls. Shelf around 80-100Hz, 0 to +10dB.

The key insight: these operate via FEEDBACK, not direct EQ. Increasing
presence means LESS high-frequency feedback -> more highs. Increasing
resonance means LESS low-frequency feedback -> more bass. This gives a
different character than tone stack EQ because it also changes the distortion
characteristics and feel of the power amp at those frequencies.

Implementation: We model this by applying shelf filters to the NFB signal
before it is subtracted from the input. When presence is turned up, the
high-shelf ATTENUATES the high frequencies in the feedback path, so
high frequencies get less negative feedback -> they are boosted in the
output. Same principle for resonance with a low-shelf.
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

from amp_sim import (
    FS, TUBES, koren_ip,
    simulate_preamp_stage, simulate_power_amp,
    tone_stack, make_cabinet_ir, apply_cabinet, save_wav,
)


# =============================================================================
# Shelf Filter Design (1st-order, bilinear transform)
# =============================================================================

def design_shelf_filter(shelf_freq, gain_db, shelf_type='high', fs=48000.0):
    """
    Design a 1st-order shelf filter using bilinear transform.

    For presence/resonance in the NFB path, we need filters that ATTENUATE
    the feedback signal at certain frequencies:
      - Presence: high-shelf CUT on the feedback (= boost in output)
      - Resonance: low-shelf CUT on the feedback (= boost in output)

    Parameters
    ----------
    shelf_freq : float
        Shelf transition frequency in Hz.
    gain_db : float
        Shelf gain in dB. Positive = boost that frequency range in the
        feedback signal. For presence/resonance knobs, we pass NEGATIVE
        values here to cut the feedback -> boost the output.
    shelf_type : str
        'high' for high-shelf, 'low' for low-shelf.
    fs : float
        Sample rate.

    Returns
    -------
    b : ndarray [b0, b1]
        Numerator coefficients.
    a : ndarray [1, a1]
        Denominator coefficients (a[0] = 1 always).
    """
    A = 10.0 ** (gain_db / 20.0)
    wc = 2.0 * np.pi * shelf_freq
    # Pre-warp
    wd = 2.0 * fs * np.tan(wc / (2.0 * fs))

    if shelf_type == 'high':
        # High-shelf: H(s) = (s + A*wc) / (s + wc)
        # Bilinear: s = 2*fs*(z-1)/(z+1)
        # Numerator:   2*fs*(z-1) + A*wc*(z+1)
        # Denominator: 2*fs*(z-1) + wc*(z+1)
        b0 = 2.0 * fs + A * wd
        b1 = A * wd - 2.0 * fs
        a0 = 2.0 * fs + wd
        a1 = wd - 2.0 * fs
    else:
        # Low-shelf: H(s) = (A*s + wc) / (s + wc)
        # Bilinear:
        # Numerator:   A*2*fs*(z-1) + wc*(z+1)
        # Denominator: 2*fs*(z-1) + wc*(z+1)
        b0 = A * 2.0 * fs + wd
        b1 = wd - A * 2.0 * fs
        a0 = 2.0 * fs + wd
        a1 = wd - 2.0 * fs

    # Normalize so a[0] = 1
    b = np.array([b0 / a0, b1 / a0])
    a = np.array([1.0, a1 / a0])
    return b, a


def apply_iir_filter(signal, b, a):
    """Apply a 1st-order IIR filter (b, a) to a signal, sample by sample."""
    y = np.zeros_like(signal)
    x_prev = 0.0
    y_prev = 0.0
    for n in range(len(signal)):
        y[n] = b[0] * signal[n] + b[1] * x_prev - a[1] * y_prev
        x_prev = signal[n]
        y_prev = y[n]
    return y


# =============================================================================
# Presence/Resonance Feedback Shaping
# =============================================================================

def shape_nfb_signal(nfb_signal, presence=0.0, resonance=0.0, fs=48000.0):
    """
    Apply presence and resonance shaping to the NFB signal.

    Parameters
    ----------
    nfb_signal : ndarray
        The raw negative feedback signal (from cabinet output).
    presence : float
        Presence knob 0-10. 0 = flat feedback, 10 = maximum high-frequency
        cut in feedback (= maximum treble boost in output, ~+10dB at 5kHz).
    resonance : float
        Resonance knob 0-10. 0 = flat feedback, 10 = maximum low-frequency
        cut in feedback (= maximum bass boost in output, ~+10dB at 90Hz).
    fs : float
        Sample rate.

    Returns
    -------
    shaped_nfb : ndarray
        Shaped feedback signal.
    """
    shaped = nfb_signal.copy()

    if presence > 0.01:
        # Presence: cut high frequencies in the feedback path
        # This means high frequencies get less NFB -> they are louder
        # Knob 0-10 maps to 0 to -10 dB cut at high frequencies
        cut_db = -presence  # knob=10 -> -10dB cut in feedback highs
        b_pres, a_pres = design_shelf_filter(
            shelf_freq=4000.0, gain_db=cut_db, shelf_type='high', fs=fs)
        shaped = apply_iir_filter(shaped, b_pres, a_pres)

    if resonance > 0.01:
        # Resonance: cut low frequencies in the feedback path
        # This means low frequencies get less NFB -> they are louder
        cut_db = -resonance  # knob=10 -> -10dB cut in feedback lows
        b_res, a_res = design_shelf_filter(
            shelf_freq=90.0, gain_db=cut_db, shelf_type='low', fs=fs)
        shaped = apply_iir_filter(shaped, b_res, a_res)

    return shaped


# =============================================================================
# Amp Simulation with Presence/Resonance
# =============================================================================

def simulate_with_presence_resonance(
        audio_in, preamp_stages=2, preamp_tube='12AX7',
        interstage_atten_db=20.0, use_bypass=True,
        tone_bass=5, tone_mid=5, tone_treble=5,
        power_tube='6L6', power_vb=400.0, power_rp=2000.0,
        power_rk=250.0, power_sag=0.3, power_clip=200.0,
        cabinet='1x12_open', input_gain=1.0,
        master_vol=0.25, nfb_amount=0.1,
        presence=5.0, resonance=5.0,
        settle=2000, fs=48000.0):
    """
    Full amp simulation with frequency-shaped negative feedback.

    The NFB signal passes through presence/resonance shelf filters before
    being subtracted from the input, creating frequency-dependent feedback.

    Parameters
    ----------
    presence : float
        Presence knob 0-10. Higher = more treble cut through the mix.
    resonance : float
        Resonance knob 0-10. Higher = more bass boom / thump.
    nfb_amount : float
        Base NFB coefficient (0.0-0.3 typical).
    """
    n_total = len(audio_in)
    n_audio = n_total - settle

    cab_ir = make_cabinet_ir(cabinet, n_taps=257, fs=int(fs))

    x_raw = audio_in * input_gain
    nfb_signal = np.zeros(n_total)

    for nfb_iter in range(3 if nfb_amount > 0 else 1):
        # Shape the NFB signal with presence/resonance filters
        shaped_nfb = shape_nfb_signal(nfb_signal, presence, resonance, fs)

        # Apply shaped NFB: subtract delayed, filtered feedback from input
        x = x_raw - shaped_nfb

        # Preamp stages
        interstage_lin = 10.0 ** (interstage_atten_db / 20.0)
        signal = x.copy()

        for stage in range(preamp_stages):
            vplate, vp_dc = simulate_preamp_stage(
                signal, tube_name=preamp_tube, use_bypass=use_bypass,
                settle=settle)
            ac_out = vplate - vp_dc
            if stage < preamp_stages - 1:
                signal = ac_out / interstage_lin
            else:
                signal = ac_out

        # Tone stack
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
        power_audio = power_out[settle:]

        # Cabinet IR
        cab_out = apply_cabinet(power_audio, cab_ir)

        # Update NFB signal for next iteration (1-sample delay)
        full_output = np.concatenate([np.zeros(settle), cab_out])
        nfb_signal = np.zeros(n_total)
        nfb_signal[1:] = nfb_amount * full_output[:-1]

    return cab_out


# =============================================================================
# Frequency Response Measurement
# =============================================================================

def measure_frequency_response(presence=5.0, resonance=5.0, nfb_amount=0.1,
                               fs=48000.0, n_points=4096):
    """
    Measure the effective frequency response of the presence/resonance
    feedback shaping by comparing shaped vs flat NFB transfer functions.

    Returns frequencies (Hz) and gain difference (dB).
    """
    # Create a white noise signal to measure filter response
    np.random.seed(42)
    noise = np.random.randn(n_points)

    # Flat NFB (no presence/resonance)
    flat = noise.copy()

    # Shaped NFB
    shaped = shape_nfb_signal(noise, presence, resonance, fs)

    # The difference in feedback means the output sees the inverse effect.
    # Less feedback at a frequency -> more output at that frequency.
    # Feedback ratio = shaped / flat, so output effect = flat / shaped.
    N = len(noise)
    w = np.hanning(N)

    sp_flat = np.fft.rfft(flat * w)
    sp_shaped = np.fft.rfft(shaped * w)

    freqs = np.fft.rfftfreq(N, 1.0 / fs)

    # Avoid division by zero
    eps = 1e-12
    # The output boost is the inverse of the feedback cut
    # If feedback is cut by X dB, output is boosted by X dB (approximately,
    # for moderate NFB amounts)
    ratio_db = 20.0 * np.log10(
        (np.abs(sp_flat) + eps) / (np.abs(sp_shaped) + eps))

    # Smooth with a moving average for cleaner plot
    kernel = np.ones(32) / 32
    ratio_smooth = np.convolve(ratio_db, kernel, mode='same')

    return freqs, ratio_smooth


# =============================================================================
# Main: Presence/Resonance Demo
# =============================================================================

if __name__ == "__main__":
    demos_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "demos")
    os.makedirs(demos_dir, exist_ok=True)

    # =========================================================================
    # 1. Frequency response curves for various presence/resonance settings
    # =========================================================================
    print("=" * 60)
    print("Presence & Resonance -- Frequency Response Curves")
    print("=" * 60)

    fig, axes = plt.subplots(2, 2, figsize=(16, 12))

    # --- Presence sweep (resonance=0) ---
    ax = axes[0, 0]
    for pres in [0, 2, 5, 7, 10]:
        freqs, gain_db = measure_frequency_response(
            presence=pres, resonance=0, nfb_amount=0.15)
        ax.semilogx(freqs[1:], gain_db[1:], linewidth=1.5,
                     label=f'Presence={pres}')
    ax.set_xlim(20, 20000)
    ax.set_ylim(-2, 12)
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Output Boost (dB)")
    ax.set_title("Presence Sweep (Resonance=0)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, which='both')
    ax.axvline(4000, color='gray', ls='--', lw=0.5, alpha=0.5)
    ax.text(4200, 11, '4kHz shelf', fontsize=7, color='gray')

    # --- Resonance sweep (presence=0) ---
    ax = axes[0, 1]
    for res in [0, 2, 5, 7, 10]:
        freqs, gain_db = measure_frequency_response(
            presence=0, resonance=res, nfb_amount=0.15)
        ax.semilogx(freqs[1:], gain_db[1:], linewidth=1.5,
                     label=f'Resonance={res}')
    ax.set_xlim(20, 20000)
    ax.set_ylim(-2, 12)
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Output Boost (dB)")
    ax.set_title("Resonance Sweep (Presence=0)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, which='both')
    ax.axvline(90, color='gray', ls='--', lw=0.5, alpha=0.5)
    ax.text(95, 11, '90Hz shelf', fontsize=7, color='gray')

    # --- Combined: presence + resonance ---
    ax = axes[1, 0]
    combos = [
        (0, 0, 'Flat (P=0, R=0)'),
        (5, 0, 'P=5, R=0'),
        (0, 5, 'P=0, R=5'),
        (5, 5, 'P=5, R=5 (scooped middle)'),
        (10, 10, 'P=10, R=10 (extreme)'),
    ]
    for pres, res, label in combos:
        freqs, gain_db = measure_frequency_response(
            presence=pres, resonance=res, nfb_amount=0.15)
        ax.semilogx(freqs[1:], gain_db[1:], linewidth=1.5, label=label)
    ax.set_xlim(20, 20000)
    ax.set_ylim(-2, 14)
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Output Boost (dB)")
    ax.set_title("Combined Presence + Resonance")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3, which='both')

    # --- NFB amount interaction ---
    ax = axes[1, 1]
    for nfb in [0.05, 0.1, 0.2, 0.3]:
        freqs, gain_db = measure_frequency_response(
            presence=7, resonance=7, nfb_amount=nfb)
        ax.semilogx(freqs[1:], gain_db[1:], linewidth=1.5,
                     label=f'NFB={nfb:.2f}')
    ax.set_xlim(20, 20000)
    ax.set_ylim(-2, 14)
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Output Boost (dB)")
    ax.set_title("P=7, R=7 at Different NFB Amounts")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, which='both')

    fig.suptitle("Presence & Resonance Controls -- Frequency Response",
                 fontsize=14, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.96])

    freq_plot_path = os.path.join(demos_dir, "presence_resonance_freq.png")
    plt.savefig(freq_plot_path, dpi=150)
    print(f"Saved: {freq_plot_path}")
    plt.close()

    # =========================================================================
    # 2. Full amp simulation demos
    # =========================================================================
    print(f"\n{'='*60}")
    print("Presence & Resonance -- Full Amp Simulation")
    print("=" * 60)

    duration = 1.0
    n_settle = 2000
    n_audio = int(duration * FS)
    n_total = n_settle + n_audio
    t = np.arange(n_total) / FS

    # 82Hz E2 for THD + bass measurement, plus 330Hz E4 for presence
    audio_in = np.zeros(n_total)
    audio_in[n_settle:] = (
        0.12 * np.sin(2.0 * np.pi * 82.0 * t[n_settle:]) +
        0.08 * np.sin(2.0 * np.pi * 330.0 * t[n_settle:]) +
        0.05 * np.sin(2.0 * np.pi * 1200.0 * t[n_settle:]))

    # Base amp settings (Marshall-ish with moderate NFB)
    amp_params = dict(
        preamp_stages=2, preamp_tube='12AX7',
        interstage_atten_db=20.0, use_bypass=True,
        tone_bass=5, tone_mid=5, tone_treble=5,
        power_tube='EL34', power_vb=450.0, power_rp=1700.0,
        power_rk=200.0, power_sag=0.2, power_clip=180.0,
        cabinet='4x12_closed', input_gain=10.0, master_vol=0.3,
        nfb_amount=0.15, settle=n_settle,
    )

    # Sweep: presence at 0, 5, 10 with resonance at 0, 5, 10
    presence_vals = [0, 5, 10]
    resonance_vals = [0, 5, 10]

    results = {}

    for pres in presence_vals:
        for res in resonance_vals:
            key = f"P{pres}_R{res}"
            print(f"\n  Presence={pres}, Resonance={res} ...")

            out = simulate_with_presence_resonance(
                audio_in, presence=float(pres), resonance=float(res),
                **amp_params)

            peak = np.max(np.abs(out))
            rms = np.sqrt(np.mean(out ** 2))
            print(f"    Peak={peak:.1f}V, RMS={rms:.2f}V")

            results[key] = {
                'presence': pres, 'resonance': res,
                'peak': peak, 'rms': rms, 'signal': out,
            }

    # =========================================================================
    # 3. Waveform and spectrum comparison plots
    # =========================================================================
    print(f"\n{'='*60}")
    print("Generating comparison plots...")
    print("=" * 60)

    t_audio = np.arange(n_audio) / FS
    mask_time = t_audio < 0.04  # 40ms window

    fig, axes = plt.subplots(3, 3, figsize=(18, 14))

    for i, pres in enumerate(presence_vals):
        for j, res in enumerate(resonance_vals):
            ax = axes[i, j]
            key = f"P{pres}_R{res}"
            sig = results[key]['signal']

            # Plot waveform
            ax_w = ax
            norm = sig / (np.max(np.abs(sig)) + 1e-12)
            ax_w.plot(t_audio[mask_time] * 1000, norm[mask_time],
                      color='#1f77b4', linewidth=0.6, alpha=0.8)
            ax_w.set_ylim(-1.2, 1.2)
            ax_w.set_xlabel("Time (ms)", fontsize=7)
            ax_w.set_ylabel("Amplitude", fontsize=7)
            ax_w.set_title(f"Presence={pres}, Resonance={res}\n"
                           f"Peak={results[key]['peak']:.1f}V, "
                           f"RMS={results[key]['rms']:.2f}V",
                           fontsize=8)
            ax_w.grid(True, alpha=0.2)
            ax_w.tick_params(labelsize=6)

    fig.suptitle("Presence & Resonance -- Waveform Grid\n"
                 "(rows: Presence 0/5/10, cols: Resonance 0/5/10)",
                 fontsize=13, fontweight='bold', y=1.0)
    plt.tight_layout(rect=[0, 0, 1, 0.95])

    wave_plot_path = os.path.join(demos_dir, "presence_resonance_waveforms.png")
    plt.savefig(wave_plot_path, dpi=150)
    print(f"Saved: {wave_plot_path}")
    plt.close()

    # --- Spectrum comparison ---
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))

    # Presence sweep at resonance=0
    ax = axes[0]
    for pres in presence_vals:
        key = f"P{pres}_R0"
        sig = results[key]['signal']
        N = len(sig)
        w = np.hanning(N)
        sp = np.abs(np.fft.rfft(sig * w))
        sp_db = 20.0 * np.log10(sp / (np.max(sp) + 1e-12) + 1e-12)
        f = np.fft.rfftfreq(N, 1.0 / FS)
        ax.plot(f, sp_db, linewidth=0.8, label=f'Presence={pres}')
    ax.set_xlim(20, 10000)
    ax.set_ylim(-70, 5)
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("dB (normalized)")
    ax.set_title("Presence Sweep (Resonance=0)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Resonance sweep at presence=0
    ax = axes[1]
    for res in resonance_vals:
        key = f"P0_R{res}"
        sig = results[key]['signal']
        N = len(sig)
        w = np.hanning(N)
        sp = np.abs(np.fft.rfft(sig * w))
        sp_db = 20.0 * np.log10(sp / (np.max(sp) + 1e-12) + 1e-12)
        f = np.fft.rfftfreq(N, 1.0 / FS)
        ax.plot(f, sp_db, linewidth=0.8, label=f'Resonance={res}')
    ax.set_xlim(20, 10000)
    ax.set_ylim(-70, 5)
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("dB (normalized)")
    ax.set_title("Resonance Sweep (Presence=0)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Combined extremes
    ax = axes[2]
    highlight_keys = ['P0_R0', 'P10_R0', 'P0_R10', 'P10_R10']
    for key in highlight_keys:
        r = results[key]
        sig = r['signal']
        N = len(sig)
        w = np.hanning(N)
        sp = np.abs(np.fft.rfft(sig * w))
        sp_db = 20.0 * np.log10(sp / (np.max(sp) + 1e-12) + 1e-12)
        f = np.fft.rfftfreq(N, 1.0 / FS)
        ax.plot(f, sp_db, linewidth=0.8,
                label=f"P={r['presence']}, R={r['resonance']}")
    ax.set_xlim(20, 10000)
    ax.set_ylim(-70, 5)
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("dB (normalized)")
    ax.set_title("Extreme Combinations")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.suptitle("Presence & Resonance -- Spectral Comparison",
                 fontsize=13, fontweight='bold')
    plt.tight_layout()

    spec_plot_path = os.path.join(demos_dir, "presence_resonance_spectra.png")
    plt.savefig(spec_plot_path, dpi=150)
    print(f"Saved: {spec_plot_path}")
    plt.close()

    # =========================================================================
    # 4. WAV demos: three key settings
    # =========================================================================
    print(f"\n{'='*60}")
    print("Generating WAV demos...")
    print("=" * 60)

    duration_wav = 3.0
    n_audio_wav = int(duration_wav * FS)
    n_total_wav = n_settle + n_audio_wav
    t_wav = np.arange(n_total_wav) / FS

    # Power chord input
    audio_chord = np.zeros(n_total_wav)
    audio_chord[n_settle:] = (
        0.15 * np.sin(2.0 * np.pi * 82.0 * t_wav[n_settle:]) +
        0.15 * np.sin(2.0 * np.pi * 123.0 * t_wav[n_settle:]) +
        0.15 * np.sin(2.0 * np.pi * 165.0 * t_wav[n_settle:]))

    wav_presets = [
        ('flat',     0,  0,  'No presence/resonance (flat NFB)'),
        ('presence', 8,  0,  'High presence (bright, cutting)'),
        ('resonance', 0, 8,  'High resonance (boomy, thick)'),
        ('both',     7,  7,  'Both cranked (scooped, modern high-gain)'),
    ]

    for name, pres, res, desc in wav_presets:
        print(f"\n  {desc} ...")
        out = simulate_with_presence_resonance(
            audio_chord, presence=float(pres), resonance=float(res),
            **amp_params)

        wav_path = os.path.join(demos_dir, f"presence_resonance_{name}.wav")
        save_wav(wav_path, out, int(FS))
        print(f"    Saved: {wav_path}")

    # =========================================================================
    # Summary
    # =========================================================================
    print(f"\n{'='*60}")
    print("Presence & Resonance -- Summary")
    print("=" * 60)
    print(f"{'Setting':<12} {'Peak(V)':>10} {'RMS(V)':>10}")
    print("-" * 35)
    for key in sorted(results.keys()):
        r = results[key]
        print(f"{key:<12} {r['peak']:>10.1f} {r['rms']:>10.2f}")

    print(f"\nOutput files:")
    print(f"  {freq_plot_path}")
    print(f"  {wave_plot_path}")
    print(f"  {spec_plot_path}")

    print("\nDone.")
