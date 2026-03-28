r"""
power_supply_sag.py
Power supply sag modeling for tube amplifier emulation.

Models the rectifier + filter cap power supply behavior:
- Current draw causes voltage drop (sag)
- Filter caps smooth the recovery (bloom)
- Different rectifier types: tube (5U4/GZ34), solid-state, none

This is the defining "feel" characteristic of tube amps:
  play soft -> clean, dig in -> compress, let off -> bloom back.
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

# =============================================================================
# Tube Constants (from power_amp.py)
# =============================================================================

TUBES = {
    "12AX7": dict(mu=100.0, ex=1.4, kg1=1060.0, kp=600.0, kvb=300.0),
    "12AU7": dict(mu=27.48, ex=1.03, kg1=466.13, kp=135.10, kvb=24224.55),
    "6SL7":  dict(mu=90.41, ex=1.25, kg1=597.32, kp=511.97, kvb=6747.79),
    "EL34":  dict(mu=10.98, ex=1.42, kg1=249.65, kp=43.2,   kvb=333.0),
    "6L6":   dict(mu=10.11, ex=1.37, kg1=406.6,  kp=31.2,   kvb=640.7),
    "300B":  dict(mu=3.95,  ex=1.4,  kg1=1550.0, kp=65.0,   kvb=300.0),
}

FS = 48000.0


# =============================================================================
# Koren Model
# =============================================================================

def koren_ip(vpk, vgk, mu, ex, kg1, kp, kvb):
    """Plate current Ip in amps."""
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


# =============================================================================
# Power Supply Model
# =============================================================================

class PowerSupply:
    """
    Models a tube rectifier power supply with sag and recovery.

    The supply voltage drops under load (sag) and recovers through
    the filter capacitor time constant (bloom). This creates the
    natural compression that defines tube amp feel.

    Parameters
    ----------
    v_nominal : float
        Unloaded B+ supply voltage (V).
    sag_amount : float
        Voltage drop per mA of plate current (V/mA).
        Higher = more compression. Tube rectifiers ~0.5, solid-state ~0.1.
    recovery_rate : float
        Filter cap recovery speed (per-sample IIR coefficient).
        Lower = slower recovery = more bloom. Tube rect ~0.0005, SS ~0.005.
    v_min_ratio : float
        Minimum supply voltage as fraction of nominal (safety floor).
    """

    # Preset rectifier characters
    PRESETS = {
        "none": dict(sag_amount=0.0, recovery_rate=0.01),
        "solid_state": dict(sag_amount=0.1, recovery_rate=0.005),
        "tube_gz34": dict(sag_amount=0.35, recovery_rate=0.0008),
        "tube_5u4": dict(sag_amount=0.5, recovery_rate=0.0005),
    }

    def __init__(self, v_nominal=400.0, sag_amount=0.3, recovery_rate=0.001,
                 v_min_ratio=0.7):
        self.v_nominal = v_nominal
        self.v_supply = v_nominal
        self.sag_amount = sag_amount
        self.recovery_rate = recovery_rate
        self.v_min = v_nominal * v_min_ratio

    @classmethod
    def from_preset(cls, preset_name, v_nominal=400.0):
        """Create a PowerSupply from a named preset."""
        if preset_name not in cls.PRESETS:
            raise ValueError(f"Unknown preset '{preset_name}'. "
                             f"Choose from: {list(cls.PRESETS.keys())}")
        params = cls.PRESETS[preset_name]
        return cls(v_nominal=v_nominal, **params)

    def reset(self):
        """Reset supply to nominal voltage."""
        self.v_supply = self.v_nominal

    def update(self, current_draw_ma):
        """
        Called each sample. Returns current supply voltage.

        Parameters
        ----------
        current_draw_ma : float
            Instantaneous plate current draw in milliamps.

        Returns
        -------
        float
            Current supply voltage after sag/recovery.
        """
        # Target voltage: drops linearly with current draw
        v_target = self.v_nominal - self.sag_amount * current_draw_ma
        v_target = max(v_target, self.v_min)

        # Filter cap smoothing: exponential approach to target
        # This is the single-pole IIR that creates the bloom effect
        self.v_supply += self.recovery_rate * (v_target - self.v_supply)

        return self.v_supply


# =============================================================================
# Power Amp with Explicit Sag Model
# =============================================================================

def simulate_power_amp_with_sag(audio_in, tube_name='6L6', vb=400.0,
                                 rp=2000.0, rk=250.0, rg=1e6, cin=22e-9,
                                 power_supply=None, clip_level=200.0,
                                 fs=48000.0, settle=2000):
    """
    Power amp simulation with explicit PowerSupply sag model.

    Instead of the simplified sag in power_amp.py, this feeds actual plate
    current into the PowerSupply model, which returns the dynamic B+ voltage
    for each sample -- creating a proper feedback loop.

    Returns
    -------
    ac_out : ndarray
        AC-coupled output voltage.
    vp_dc : float
        DC quiescent plate voltage.
    v_supply_trace : ndarray
        Supply voltage over time (for plotting sag behavior).
    ip_trace : ndarray
        Plate current over time in mA (for diagnostics).
    """
    tube = TUBES[tube_name]
    n_total = len(audio_in)
    out_vplate = np.zeros(n_total)
    v_supply_trace = np.zeros(n_total)
    ip_trace = np.zeros(n_total)

    # If no power supply model given, use constant voltage (no sag)
    if power_supply is None:
        power_supply = PowerSupply(v_nominal=vb, sag_amount=0.0)

    # High-pass coupling filter
    tau_hp = rg * cin
    k_hp = 2.0 * fs * tau_hp
    c_hp = (k_hp - 1.0) / (k_hp + 1.0)
    hp_gain = (1.0 + c_hp) / 2.0

    # Newton-Raphson state
    prev_ip = 1.0e-3
    hp_y = 0.0
    hp_x_prev = 0.0

    R_p = rp
    R_k = rk

    for n in range(n_total):
        vin = audio_in[n]

        # High-pass filter (input coupling)
        hp_y = c_hp * hp_y + hp_gain * (vin - hp_x_prev)
        hp_x_prev = vin
        v_grid = hp_y

        # Get current supply voltage from power supply model
        vb_eff = power_supply.v_supply  # use current state (updated at end of loop)

        # Upward pass
        a_p = vb_eff
        a_g = v_grid
        a_k = 0.0

        # Newton-Raphson at root
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

        # Record traces
        ip_ma = Ip * 1000.0  # convert to mA for the sag model
        ip_trace[n] = ip_ma
        v_supply_trace[n] = vb_eff

        # Feed current draw back into power supply (the sag feedback loop)
        power_supply.update(ip_ma)

    # DC quiescent point
    sl = slice(max(settle - 100, 0), settle)
    vp_dc = out_vplate[sl].mean()

    # AC-couple
    ac_out = out_vplate - vp_dc

    # Transformer saturation: soft clip with tanh
    ac_out = np.tanh(ac_out / clip_level) * clip_level

    return ac_out, vp_dc, v_supply_trace, ip_trace


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
# Main: Sag comparison demo
# =============================================================================

if __name__ == "__main__":
    demos_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "demos")
    os.makedirs(demos_dir, exist_ok=True)

    # -------------------------------------------------------------------------
    # Input: Volume swell from 0.1V to 3V over 3 seconds
    # -------------------------------------------------------------------------
    duration = 3.0
    n_settle = 2000
    n_audio = int(duration * FS)
    n_total = n_settle + n_audio
    t_total = np.arange(n_total) / FS

    # E power chord (82 + 123 + 165 Hz)
    chord = (np.sin(2 * np.pi * 82.0 * t_total) +
             np.sin(2 * np.pi * 123.0 * t_total) +
             np.sin(2 * np.pi * 165.0 * t_total)) / 3.0

    # Volume envelope: linear swell from 0.1V to 3V during audio portion
    envelope = np.zeros(n_total)
    envelope[:n_settle] = 0.1  # settle at low level
    envelope[n_settle:] = np.linspace(0.1, 3.0, n_audio)

    audio_in = chord * envelope

    # -------------------------------------------------------------------------
    # Three sag configurations
    # -------------------------------------------------------------------------
    configs = [
        ("none",             "No Sag (constant supply)",
         PowerSupply.from_preset("none", v_nominal=400.0)),
        ("solidstate",       "Solid-State Rectifier",
         PowerSupply.from_preset("solid_state", v_nominal=400.0)),
        ("tube_rectifier",   "Tube Rectifier (5U4 -- maximum squish)",
         PowerSupply.from_preset("tube_5u4", v_nominal=400.0)),
    ]

    print("=" * 60)
    print("Power Supply Sag Comparison")
    print("=" * 60)
    print(f"  Input: E power chord, volume swell 0.1V -> 3V over {duration}s")
    print(f"  Tube: 6L6 power amp, VB=400V, RP=2000, RK=250")
    print()

    results = []

    for tag, desc, psu in configs:
        print(f"--- {desc} ---")
        print(f"  sag_amount={psu.sag_amount}, recovery_rate={psu.recovery_rate}")

        psu.reset()
        ac_out, vp_dc, v_supply, ip_trace = simulate_power_amp_with_sag(
            audio_in, tube_name='6L6', vb=400.0, rp=2000.0, rk=250.0,
            power_supply=psu, clip_level=200.0, settle=n_settle)

        audio_part = ac_out[n_settle:]
        peak = np.max(np.abs(audio_part))
        rms = np.sqrt(np.mean(audio_part ** 2))
        v_supply_min = np.min(v_supply[n_settle:])
        v_supply_max = np.max(v_supply[n_settle:])
        ip_max = np.max(ip_trace[n_settle:])

        print(f"  Vp_dc = {vp_dc:.1f}V")
        print(f"  Output peak = {peak:.1f}V, RMS = {rms:.1f}V")
        print(f"  Supply range: {v_supply_min:.1f}V - {v_supply_max:.1f}V")
        print(f"  Peak current: {ip_max:.1f}mA")
        print()

        # Save WAV
        wav_path = os.path.join(demos_dir, f"sag_{tag}.wav")
        save_wav(wav_path, audio_part, int(FS))

        results.append((tag, desc, ac_out, v_supply, ip_trace, vp_dc))

    # -------------------------------------------------------------------------
    # Plot: sag_comparison.png
    # -------------------------------------------------------------------------
    fig, axes = plt.subplots(4, 1, figsize=(14, 16), sharex=True)
    fig.suptitle("Power Supply Sag Comparison\n"
                 "6L6 power amp, E chord volume swell 0.1V -> 3V",
                 fontsize=14, fontweight='bold')

    t_audio = np.arange(n_audio) / FS
    colors = ['#2ca02c', '#1f77b4', '#d62728']
    labels_short = ['No sag', 'Solid-state', 'Tube rect (5U4)']

    # Panel 1: Input envelope
    ax = axes[0]
    ax.plot(t_audio, envelope[n_settle:], color='gray', linewidth=1.5)
    ax.set_ylabel("Input Amplitude (V)")
    ax.set_title("Input Volume Swell")
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 3.5)

    # Panel 2: Supply voltage over time
    ax = axes[1]
    for i, (tag, desc, ac_out, v_supply, ip_trace, vp_dc) in enumerate(results):
        ax.plot(t_audio, v_supply[n_settle:], color=colors[i],
                linewidth=1.2, label=labels_short[i])
    ax.set_ylabel("Supply Voltage (V)")
    ax.set_title("B+ Supply Voltage (sag under load)")
    ax.legend(loc='lower left')
    ax.grid(True, alpha=0.3)

    # Panel 3: Plate current
    ax = axes[2]
    for i, (tag, desc, ac_out, v_supply, ip_trace, vp_dc) in enumerate(results):
        # Downsample for plotting (too many points otherwise)
        ds = 48  # plot every 48th sample = 1kHz display rate
        ax.plot(t_audio[::ds], ip_trace[n_settle::ds], color=colors[i],
                linewidth=0.8, label=labels_short[i], alpha=0.8)
    ax.set_ylabel("Plate Current (mA)")
    ax.set_title("Plate Current Draw")
    ax.legend(loc='upper left')
    ax.grid(True, alpha=0.3)

    # Panel 4: Output envelope comparison
    ax = axes[3]
    for i, (tag, desc, ac_out, v_supply, ip_trace, vp_dc) in enumerate(results):
        # Compute RMS envelope with ~10ms window
        win = int(0.010 * FS)
        audio_part = ac_out[n_settle:]
        env = np.sqrt(np.convolve(audio_part ** 2, np.ones(win) / win, mode='same'))
        ax.plot(t_audio, env, color=colors[i], linewidth=1.2, label=labels_short[i])
    ax.set_ylabel("Output Envelope (V RMS)")
    ax.set_title("Output Envelope -- Sag Compresses Loud Passages")
    ax.set_xlabel("Time (s)")
    ax.legend(loc='upper left')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plot_path = os.path.join(demos_dir, "sag_comparison.png")
    plt.savefig(plot_path, dpi=150)
    print(f"Saved: {plot_path}")
    plt.close()

    # -------------------------------------------------------------------------
    # Verilog implementation spec
    # -------------------------------------------------------------------------
    spec_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "sag_verilog_spec.txt")
    with open(spec_path, 'w') as f:
        f.write("""\
Power Supply Sag -- FPGA Implementation Specification
=====================================================

Overview
--------
Power supply sag is a single-pole IIR filter on the B+ supply voltage,
driven by the instantaneous plate current. On the Tang Nano 20K (27MHz,
Q16.16 fixed-point), this requires ~3 clock cycles per sample.

Algorithm (per audio sample)
----------------------------
1. MULTIPLY: v_drop = sag_amount * Ip_mA          (1 clock, 16x16 multiply)
2. SUBTRACT: v_target = V_nominal - v_drop         (1 clock, clamped to v_min)
3. IIR:      v_supply += alpha * (v_target - v_supply)  (1 clock, multiply-accumulate)

Where alpha = recovery_rate is a small fractional coefficient.

Fixed-Point Format
------------------
- v_supply, v_nominal, v_target: Q16.16 (signed 32-bit), range 0-600V
- sag_amount: Q0.16 unsigned (0.0 to ~1.0 V/mA)
- recovery_rate (alpha): Q0.16 unsigned (0.0 to ~1.0)
- Ip_mA: Q16.16 from the WDF solver output (convert from amps: shift left by ~10)

Registers (4 total)
--------------------
- REG_V_NOMINAL   [31:0]  -- Unloaded supply voltage (Q16.16)
- REG_SAG_AMOUNT  [15:0]  -- Sag depth coefficient (Q0.16)
- REG_ALPHA       [15:0]  -- Recovery rate / filter coefficient (Q0.16)
- REG_V_SUPPLY    [31:0]  -- Current supply voltage state (Q16.16, read/write)

Preset Values (Q0.16 hex)
--------------------------
No sag:           SAG_AMOUNT=0x0000, ALPHA=0x0147  (0.005)
Solid-state rect: SAG_AMOUNT=0x199A, ALPHA=0x0147  (0.1 V/mA, 0.005)
Tube GZ34:        SAG_AMOUNT=0x599A, ALPHA=0x0034  (0.35 V/mA, 0.0008)
Tube 5U4:         SAG_AMOUNT=0x8000, ALPHA=0x0021  (0.5 V/mA, 0.0005)

Pipeline Integration
--------------------
The sag module sits between the WDF solver output (Ip) and the VB input
to the next sample's WDF computation:

    [WDF Solver] --Ip--> [Sag IIR] --v_supply--> [WDF Solver (next sample)]
                              ^
                         REG_V_NOMINAL, REG_SAG_AMOUNT, REG_ALPHA

Total added latency: 3 clocks out of 562 available per 48kHz sample.
Resource cost: 1 multiplier, 4 registers. Negligible on GW2A.

Verilog Pseudocode
------------------
    // Stage 1: compute voltage drop
    wire signed [31:0] v_drop = (sag_amount * ip_ma) >>> 16;

    // Stage 2: compute target, clamp
    wire signed [31:0] v_target = v_nominal - v_drop;
    wire signed [31:0] v_target_clamped =
        (v_target < v_min) ? v_min : v_target;

    // Stage 3: IIR update
    wire signed [31:0] v_diff = v_target_clamped - v_supply;
    wire signed [31:0] v_delta = (alpha * v_diff) >>> 16;
    always @(posedge clk) begin
        if (sample_tick)
            v_supply <= v_supply + v_delta;
    end
""")
    print(f"Saved: {spec_path}")

    print("\nDone. Power supply sag demo complete.")
