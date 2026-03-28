"""
coupling_cap_blocking.py
Interstage coupling capacitor blocking distortion model.

Between each preamp stage there's a coupling capacitor (typically 22nF).
When the signal is heavily distorted, asymmetric clipping creates a DC offset
that charges the coupling cap. This:
  - Shifts the bias point of the next stage
  - Causes "blocking distortion" -- the next stage cuts off temporarily
  - Creates a gated, sputtery sound at high gain settings
  - Is why high-gain amps have a particular "chug" on palm mutes
  - Recovers over time as the cap discharges through the grid resistor

This script models the coupling cap as a proper WDF capacitor that accumulates
DC from asymmetric signals, and compares against a simple high-pass filter
that only models the frequency response.
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

# Cathode bypass cap WDF port resistance
R_CK = 1.0 / (2.0 * FS * CK)
G_rk = 1.0 / RK
G_ck = 1.0 / R_CK
G_total = G_rk + G_ck
R_CATH = 1.0 / G_total
GAMMA_CATH = G_rk / G_total

# Koren 12AX7 constants
MU, EX, KG1, KP, KVB = 100.0, 1.4, 1060.0, 600.0, 300.0


def koren_ip(vpk, vgk):
    """Plate current Ip in amps."""
    vpk = float(np.clip(vpk, 0.0, 500.0))
    vgk = float(np.clip(vgk, -50.0, 5.0))
    if vpk <= 0:
        return 0.0
    inner = KP * (1.0 / MU + vgk / math.sqrt(KVB + vpk * vpk))
    inner = float(np.clip(inner, -500, 500))
    Ed = (vpk / KP) * math.log1p(math.exp(inner))
    if Ed <= 0:
        return 0.0
    return (Ed ** EX) / KG1


def koren_dip_dvpk(vpk, vgk, h=0.01):
    return (koren_ip(vpk + h, vgk) - koren_ip(vpk - h, vgk)) / (2 * h)


def koren_dip_dvgk(vpk, vgk, h=0.01):
    return (koren_ip(vpk, vgk + h) - koren_ip(vpk, vgk - h)) / (2 * h)


# =============================================================================
# WDF Triode Stage -- Two Coupling Cap Modes
# =============================================================================

def simulate_stage_wdf(audio_in, blocking=False):
    """
    Simulate a single preamp triode stage with WDF.

    The coupling cap between stages is modeled two ways:
      blocking=False: Simple 1st-order HP filter (IIR). Models frequency response
                      but does NOT accumulate DC offset from asymmetric signals.
      blocking=True:  WDF capacitor. The cap state z_cc stores the voltage across
                      the cap. Asymmetric clipping charges the cap, shifting the
                      next stage's bias point. The cap discharges through Rg with
                      time constant tau = Rg * Cin (~22ms for 1M * 22nF).

    Returns: (ac_output, cap_voltage_history)
    """
    n_total = len(audio_in)
    out_vplate = np.zeros(n_total)
    cap_voltage = np.zeros(n_total)

    R_p = RP
    R_k = R_CATH

    # Input coupling cap -- simple HP filter coefficients
    tau_hp = RG * CIN
    k_hp = 2.0 * FS * tau_hp
    c_hp_coeff = (k_hp - 1.0) / (k_hp + 1.0)
    hp_gain = (1.0 + c_hp_coeff) / 2.0

    # WDF coupling cap port resistance: R_cc = 1 / (2 * Fs * C)
    R_cc = 1.0 / (2.0 * FS * CIN)
    # The coupling cap + grid resistor form a series adaptor:
    #   R_parent = R_cc + R_g
    #   gamma_cc = R_cc / (R_cc + R_g)
    # But simpler: model as WDF capacitor feeding the grid.
    # The cap sees the grid resistor Rg as its load.
    # WDF parallel adaptor: cap || Rg
    G_cc = 1.0 / R_cc
    G_rg = 1.0 / RG
    G_cc_total = G_cc + G_rg
    gamma_cc = G_cc / G_cc_total

    # State variables
    prev_ip = 0.5e-3
    hp_y = 0.0
    hp_x_prev = 0.0
    z_ck = 0.0   # cathode bypass cap state
    z_cc = 0.0   # coupling cap state (WDF mode)

    for n in range(n_total):
        vin = audio_in[n]

        if blocking:
            # WDF coupling cap model:
            # The coupling cap is a WDF capacitor element. Its reflected wave
            # is the stored state: b_cc = z_cc.
            # The grid resistor Rg is a WDF resistor: b_rg = 0.
            # These form a parallel adaptor. The output voltage at the grid
            # is the junction voltage: v_grid = (a_cc + b_cc) / 2 in WDF terms.
            #
            # But the cap is driven by vin on one side. We model the series
            # connection: vin -> cap -> Rg -> ground.
            # The voltage across the cap is v_cap. The grid voltage is:
            #   v_grid = vin - v_cap
            # The current through the circuit is:
            #   i = v_grid / Rg = (vin - v_cap) / Rg
            # The cap voltage evolves as:
            #   dv_cap/dt = i / C = (vin - v_cap) / (Rg * C)
            #
            # Bilinear transform discretization:
            #   v_cap[n] = alpha * v_cap[n-1] + (1-alpha) * vin
            #   where alpha = (2*Fs*Rg*C - 1) / (2*Fs*Rg*C + 1) = c_hp_coeff
            #
            # Wait -- this IS the same math as the HP filter, but we track
            # v_cap explicitly, and the grid voltage is vin - v_cap.
            #
            # The KEY difference: in the simple HP filter, the state tracks
            # the AC-coupled signal directly. In the WDF cap model, the
            # cap state accumulates the actual DC charging from the FULL
            # signal including the asymmetric output of the previous stage.
            #
            # For cascaded stages: the input to the coupling cap is the
            # PLATE VOLTAGE (not AC-coupled), which has a large DC component
            # plus asymmetric AC. The cap charges to the DC level. When the
            # signal is symmetric, v_cap = DC and v_grid = AC perfectly.
            # When the signal is asymmetric (clipped), the cap charges
            # differently on positive vs negative half-cycles, creating
            # a bias shift.

            # WDF capacitor: b_cc = z_cc (reflected = stored state)
            b_cc = z_cc
            # WDF resistor Rg: b_rg = 0
            b_rg = 0.0

            # Series adaptor: cap in series with Rg, driven by voltage source vin
            # Voltage source: b_vs = 2*vin - a_vs (with small source resistance)
            # Simpler: use the direct formulation.
            # The cap + Rg in series, driven by vin:
            #   v_grid = junction voltage at cap-Rg node
            #
            # Using WDF series adaptor (3-port): ports are cap, Rg, source
            # R_source ~ 0 (voltage source), R_cap = R_cc, R_rg = RG
            # gamma = R_cc / (R_cc + RG)   [source is adapted, R_parent = R_cc + RG]
            #
            # Actually, let's use the standard approach:
            # The input coupling network is: Vin --[C]-- Vgrid --[Rg]-- GND
            # In WDF, C and Rg are in series from Vin's perspective.
            # The series adaptor combines them:
            gamma_ser = R_cc / (R_cc + RG)

            # The source (previous stage output) drives the parent port.
            # Parent wave: a_parent = 2*vin (assuming adapted source)
            # Actually, treat vin as a known voltage source with R_s -> 0.
            # For a voltage source: b_source = 2*V - a_source.
            # In practice, for the series adaptor {cap, Rg} with parent = source:
            #   b_parent (upward) = -(b_cc + b_rg) = -z_cc
            #   a_parent (from source) = 2*vin - b_parent = 2*vin + z_cc
            #   (voltage source: a = 2*V - b)
            #
            # Downward scatter:
            #   a_cc = b_cc - gamma_ser * (a_parent + b_cc + b_rg)
            #   a_rg = -(a_parent + a_cc)  [series constraint: sum of a's = 0... no]
            #
            # Let me use the standard 3-port series adaptor formulas from CLAUDE.md:
            #   With parent port 0, child ports 1 (cap) and 2 (Rg):
            #   gamma = R1 / (R1 + R2) = R_cc / (R_cc + RG)
            #   Upward: b0 = -(b1 + b2)
            #   Downward: x = a0 + b1 + b2
            #              a1 = b1 - gamma * x
            #              a2 = -(x + a1)   [wait, that's a2 = -(a0 + a1)]
            #              Actually from CLAUDE.md: a2 = -(x + a1)

            # Upward pass
            b_parent = -(b_cc + b_rg)  # = -z_cc

            # Voltage source drives the parent port
            a_parent = 2.0 * vin - b_parent  # = 2*vin + z_cc

            # Downward scatter
            x = a_parent + b_cc + b_rg  # = 2*vin + z_cc + z_cc = 2*vin + 2*z_cc
            a_cc = b_cc - gamma_ser * x
            a_rg = -(x + a_cc)

            # Grid voltage = voltage across Rg = (a_rg + b_rg) / 2
            v_grid = (a_rg + b_rg) / 2.0

            # Update cap state
            z_cc = a_cc

            # Track cap voltage for plotting
            cap_voltage[n] = (a_cc + b_cc) / 2.0
        else:
            # Simple HP filter -- no DC accumulation tracking
            hp_y = c_hp_coeff * hp_y + hp_gain * (vin - hp_x_prev)
            hp_x_prev = vin
            v_grid = hp_y
            cap_voltage[n] = 0.0  # not tracked

        # --- WDF triode stage (same for both modes) ---

        # Upward pass: reflected waves from leaves
        b_plate = VB
        b_grid = v_grid

        # Cathode: parallel adaptor Rk || Ck
        b_rk_cath = 0.0
        b_ck = z_ck
        bDiff = b_ck - b_rk_cath
        b_cathode = b_ck - GAMMA_CATH * bDiff

        # Root incident waves
        a_p = b_plate
        a_g = b_grid
        a_k = b_cathode

        # Newton-Raphson at root
        Ip = prev_ip
        for iteration in range(20):
            Vpk = (a_p - a_k) - (R_p + R_k) * Ip
            Vgk = (a_g - a_k) - R_k * Ip
            ip_model = koren_ip(Vpk, Vgk)
            f_val = Ip - ip_model
            if abs(f_val) < 1e-10:
                break
            dip_dvpk_v = koren_dip_dvpk(Vpk, Vgk)
            dip_dvgk_v = koren_dip_dvgk(Vpk, Vgk)
            df_dIp = 1.0 + dip_dvpk_v * (R_p + R_k) + dip_dvgk_v * R_k
            if abs(df_dIp) < 1e-15:
                break
            Ip -= f_val / df_dIp
            Ip = max(Ip, 0.0)

        prev_ip = Ip

        # Reflected waves
        b_p = a_p - 2.0 * R_p * Ip
        b_k = a_k + 2.0 * R_k * Ip

        # Node voltages
        v_plate = (a_p + b_p) / 2.0
        out_vplate[n] = v_plate

        # Update cathode bypass cap
        a_ck = b_k + b_cathode - b_ck
        z_ck = a_ck

    return out_vplate, cap_voltage


# =============================================================================
# Multi-Stage Cascade with Coupling Caps
# =============================================================================

def simulate_cascade(audio_in, n_stages=3, blocking=False, settle=4000):
    """
    Simulate a cascaded preamp chain with interstage coupling caps.

    Each stage:
      1. Input goes through coupling cap (WDF or simple HP)
      2. Triode amplifies with WDF Newton-Raphson
      3. Plate voltage output feeds the next stage's coupling cap

    The coupling cap between stages is where blocking distortion happens:
    asymmetric tube clipping charges the cap, shifting the next stage's bias.

    Returns: list of (plate_voltage, cap_voltage) per stage
    """
    n_total = len(audio_in) + settle
    full_in = np.concatenate([np.zeros(settle), audio_in])

    stage_outputs = []
    current_signal = full_in

    for stage in range(n_stages):
        vplate, vcap = simulate_stage_wdf(current_signal, blocking=blocking)

        # Remove settle and store
        stage_outputs.append((vplate, vcap))

        # Feed plate voltage to next stage (full signal, not AC-coupled --
        # the coupling cap of the next stage handles AC coupling)
        current_signal = vplate

    # Extract post-settle results
    results = []
    for vplate, vcap in stage_outputs:
        # Find DC operating point from settle region
        sl = slice(max(settle - 200, 0), settle)
        vp_dc = vplate[sl].mean()
        ac = vplate[settle:] - vp_dc
        results.append({
            'ac_out': ac,
            'vplate_full': vplate[settle:],
            'vp_dc': vp_dc,
            'cap_voltage': vcap[settle:],
        })

    return results


# =============================================================================
# Generate Palm Mute Pattern
# =============================================================================

def make_palm_mute(duration_sec=1.5, fs=48000.0, freq=82.41, amplitude=4.0):
    """
    Generate a palm-muted power chord pattern: short staccato hits with gaps.
    Low E (82.41 Hz) with 5th (123.47 Hz) for a chunky power chord.
    """
    n_total = int(duration_sec * fs)
    t = np.arange(n_total) / fs
    signal = np.zeros(n_total)

    # Hit parameters
    hit_duration = 0.08   # 80ms per hit
    gap_duration = 0.15   # 150ms gap between hits
    n_hits = 6

    for i in range(n_hits):
        start = int((i * (hit_duration + gap_duration)) * fs)
        end = min(start + int(hit_duration * fs), n_total)
        if start >= n_total:
            break
        t_hit = np.arange(end - start) / fs

        # Power chord: root + fifth
        chord = (np.sin(2 * np.pi * freq * t_hit) +
                 0.7 * np.sin(2 * np.pi * freq * 1.498 * t_hit))  # ~perfect 5th

        # Palm mute envelope: sharp attack, fast decay
        env = np.exp(-t_hit / 0.02) * (1 - np.exp(-t_hit / 0.001))
        signal[start:end] = amplitude * chord * env

    return signal


# =============================================================================
# Normalize and Save WAV
# =============================================================================

def save_wav(filename, signal, fs=48000):
    """Normalize and save as 16-bit WAV."""
    if not HAS_SCIPY:
        print(f"  [skip] scipy not available, cannot save {filename}")
        return
    peak = np.max(np.abs(signal))
    if peak > 0:
        signal = signal / peak * 0.9
    data = np.clip(signal * 32767, -32768, 32767).astype(np.int16)
    wavfile.write(filename, int(fs), data)
    print(f"  Saved: {filename}")


# =============================================================================
# Main: Run Comparisons and Generate Outputs
# =============================================================================

if __name__ == '__main__':
    os.makedirs("demos", exist_ok=True)

    print("=" * 70)
    print("Coupling Cap Blocking Distortion Demo")
    print("=" * 70)

    # =========================================================================
    # Test 1: Sustained overdriven signal (4V, 3 stages)
    # =========================================================================
    print("\n--- Test 1: Sustained overdrive (4V, 3 stages) ---")

    freq = 200.0  # Hz -- low enough to see blocking clearly
    duration = 0.3  # seconds
    n_samples = int(duration * FS)
    t_audio = np.arange(n_samples) / FS

    # Envelope: sharp attack, sustain
    env = np.ones(n_samples)
    attack_samples = int(0.005 * FS)
    env[:attack_samples] = np.linspace(0, 1, attack_samples)
    audio_in = 4.0 * np.sin(2 * np.pi * freq * t_audio) * env

    print("  Simulating WITHOUT blocking (simple HP filter)...")
    results_off = simulate_cascade(audio_in, n_stages=3, blocking=False, settle=4000)

    print("  Simulating WITH blocking (WDF coupling cap)...")
    results_on = simulate_cascade(audio_in, n_stages=3, blocking=True, settle=4000)

    # Compute RMS in windows to show the blocking effect
    window_ms = 10
    window_samples = int(window_ms * FS / 1000)

    def windowed_rms(signal, window_size):
        n_windows = len(signal) // window_size
        rms = np.zeros(n_windows)
        for i in range(n_windows):
            chunk = signal[i * window_size:(i + 1) * window_size]
            rms[i] = np.sqrt(np.mean(chunk ** 2))
        return rms

    # =========================================================================
    # Test 2: Palm mute pattern
    # =========================================================================
    print("\n--- Test 2: Palm mute pattern ---")
    palm_signal = make_palm_mute(duration_sec=1.5, amplitude=4.0)

    print("  Simulating WITHOUT blocking...")
    palm_off = simulate_cascade(palm_signal, n_stages=3, blocking=False, settle=4000)

    print("  Simulating WITH blocking...")
    palm_on = simulate_cascade(palm_signal, n_stages=3, blocking=True, settle=4000)

    # =========================================================================
    # Plot: Blocking Comparison
    # =========================================================================
    print("\n--- Generating plots ---")

    fig, axes = plt.subplots(4, 2, figsize=(16, 16))
    fig.suptitle("Interstage Coupling Cap Blocking Distortion\n"
                 "22nF cap, 1M grid resistor, 3-stage 12AX7 cascade, 4V input",
                 fontsize=14, fontweight='bold')

    t_ms = t_audio * 1000

    for stage_idx in range(3):
        ax = axes[stage_idx, 0]
        ac_off = results_off[stage_idx]['ac_out']
        ac_on = results_on[stage_idx]['ac_out']

        ax.plot(t_ms, ac_off, 'b', linewidth=0.8, alpha=0.7, label='No blocking (HP filter)')
        ax.plot(t_ms, ac_on, 'r', linewidth=0.8, alpha=0.7, label='With blocking (WDF cap)')
        ax.set_ylabel(f"Stage {stage_idx+1} Out (V)")
        ax.legend(fontsize=8, loc='upper right')
        ax.grid(True, alpha=0.3)

        # Cap voltage on right axis (blocking mode only)
        ax2 = axes[stage_idx, 1]
        cv = results_on[stage_idx]['cap_voltage']
        ax2.plot(t_ms, cv, 'g', linewidth=1.0)
        ax2.set_ylabel(f"Stage {stage_idx+1} Cap V")
        ax2.set_title(f"Coupling Cap Voltage (stage {stage_idx+1} input)")
        ax2.grid(True, alpha=0.3)

    # Bottom row: RMS envelope comparison for final stage
    ax_rms = axes[3, 0]
    rms_off = windowed_rms(results_off[2]['ac_out'], window_samples)
    rms_on = windowed_rms(results_on[2]['ac_out'], window_samples)
    t_rms = np.arange(len(rms_off)) * window_ms

    ax_rms.plot(t_rms, rms_off, 'b-', linewidth=2, label='No blocking')
    ax_rms.plot(t_rms, rms_on, 'r-', linewidth=2, label='With blocking')
    ax_rms.set_xlabel("Time (ms)")
    ax_rms.set_ylabel("RMS Level (V)")
    ax_rms.set_title("Output RMS Envelope (Stage 3)")
    ax_rms.legend(fontsize=10)
    ax_rms.grid(True, alpha=0.3)

    # Info text
    ax_info = axes[3, 1]
    ax_info.axis('off')
    tau = RG * CIN * 1000  # ms
    info_text = (
        f"Coupling cap: C = {CIN*1e9:.0f} nF\n"
        f"Grid resistor: Rg = {RG/1e6:.0f} M ohm\n"
        f"Time constant: tau = Rg*C = {tau:.1f} ms\n\n"
        f"Blocking distortion occurs when asymmetric\n"
        f"tube clipping charges the coupling cap,\n"
        f"shifting the next stage's DC bias point.\n\n"
        f"The cap charges quickly during clipped peaks\n"
        f"but discharges slowly through Rg (tau={tau:.0f}ms).\n"
        f"This creates a gated, sputtery decay on\n"
        f"sustained high-gain notes and a characteristic\n"
        f"'chug' on palm-muted power chords."
    )
    ax_info.text(0.1, 0.5, info_text, fontsize=11, family='monospace',
                 verticalalignment='center', transform=ax_info.transAxes,
                 bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    axes[0, 0].set_title("Waveform Comparison")
    axes[2, 0].set_xlabel("Time (ms)")
    axes[2, 1].set_xlabel("Time (ms)")

    plt.tight_layout()
    plt.savefig("demos/blocking_comparison.png", dpi=150, bbox_inches='tight')
    print("  Saved: demos/blocking_comparison.png")
    plt.close()

    # =========================================================================
    # Save WAV files
    # =========================================================================
    print("\n--- Saving WAV files ---")

    # Sustained overdrive
    save_wav("demos/blocking_off.wav", results_off[2]['ac_out'], int(FS))
    save_wav("demos/blocking_on.wav", results_on[2]['ac_out'], int(FS))

    # Palm mute
    palm_out_on = palm_on[2]['ac_out']
    save_wav("demos/blocking_palm_mute.wav", palm_out_on, int(FS))

    # =========================================================================
    # Palm Mute Comparison Plot (bonus)
    # =========================================================================
    n_palm = len(palm_signal)
    t_palm = np.arange(len(palm_off[2]['ac_out'])) / FS * 1000

    fig2, axes2 = plt.subplots(3, 1, figsize=(14, 10))
    fig2.suptitle("Palm Mute Pattern: Blocking Distortion Effect\n"
                  "6 staccato hits, 82Hz power chord, 4V input, 3 stages",
                  fontsize=13, fontweight='bold')

    # Input signal
    t_in = np.arange(n_palm) / FS * 1000
    axes2[0].plot(t_in, palm_signal, 'k', linewidth=0.5)
    axes2[0].set_ylabel("Input (V)")
    axes2[0].set_title("Input Signal")
    axes2[0].grid(True, alpha=0.3)

    # Without blocking
    axes2[1].plot(t_palm, palm_off[2]['ac_out'], 'b', linewidth=0.5)
    axes2[1].set_ylabel("Output (V)")
    axes2[1].set_title("Without Blocking (simple HP filter)")
    axes2[1].grid(True, alpha=0.3)

    # With blocking
    axes2[2].plot(t_palm, palm_on[2]['ac_out'], 'r', linewidth=0.5)
    axes2[2].set_ylabel("Output (V)")
    axes2[2].set_title("With Blocking (WDF coupling cap)")
    axes2[2].set_xlabel("Time (ms)")
    axes2[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("demos/blocking_palm_mute.png", dpi=150, bbox_inches='tight')
    print("  Saved: demos/blocking_palm_mute.png")
    plt.close()

    # =========================================================================
    # Summary Statistics
    # =========================================================================
    print("\n" + "=" * 70)
    print("Summary")
    print("=" * 70)

    for stage in range(3):
        ac_off = results_off[stage]['ac_out']
        ac_on = results_on[stage]['ac_out']

        # Compare first 50ms vs last 50ms
        n50 = int(0.05 * FS)
        rms_early_off = np.sqrt(np.mean(ac_off[:n50] ** 2))
        rms_late_off = np.sqrt(np.mean(ac_off[-n50:] ** 2))
        rms_early_on = np.sqrt(np.mean(ac_on[:n50] ** 2))
        rms_late_on = np.sqrt(np.mean(ac_on[-n50:] ** 2))

        ratio_off = rms_late_off / (rms_early_off + 1e-12)
        ratio_on = rms_late_on / (rms_early_on + 1e-12)

        print(f"\n  Stage {stage+1}:")
        print(f"    No blocking:   early RMS={rms_early_off:.2f}V  late RMS={rms_late_off:.2f}V  ratio={ratio_off:.3f}")
        print(f"    With blocking: early RMS={rms_early_on:.2f}V  late RMS={rms_late_on:.2f}V  ratio={ratio_on:.3f}")
        if ratio_off > 0:
            print(f"    Blocking causes {(1 - ratio_on/ratio_off)*100:.1f}% additional decay")

    print("\nDone.")
