"""
compute_tonestack.py
Compute IIR biquad coefficients from actual Fender Bassman and Marshall
tone stack circuit topology.

The Fender/Marshall tone stack is a passive RC ladder network with 3 pots
(treble, mid, bass). The analog transfer function H(s) is derived from
circuit analysis, then bilinear-transformed to H(z) and factored into
cascaded second-order sections (SOS) for the FPGA biquad engine.

References:
  - David Yeh, "Digital Implementation of Musical Distortion Circuits
    by Analysis and Simulation," Stanford CCRMA, 2009
  - https://www.guitarscience.net/tsc/info.htm (tone stack calculator)

Component values:
  Fender Bassman: R1=250k, R2=1M, R3=25k, R4=56k, C1=250pF, C2=20nF, C3=20nF
  Marshall JCM800: R1=220k, R2=1M, R3=25k, R4=33k, C1=470pF, C2=22nF, C3=22nF
"""

import numpy as np

try:
    from scipy.signal import bilinear_zpk, zpk2sos, sosfreqz, tf2zpk
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


# =============================================================================
# Tone Stack Circuit Analysis
# =============================================================================

def tonestack_tf(t, m, l, R1, R2, R3, R4, C1, C2, C3):
    """
    Compute the analog transfer function H(s) = B(s)/A(s) of a
    Fender/Marshall tone stack.

    Parameters
    ----------
    t : float  Treble pot position (0-1)
    m : float  Mid pot position (0-1)
    l : float  Bass pot position (0-1)
    R1-R4, C1-C3 : float  Component values

    Returns
    -------
    b, a : ndarray  Numerator and denominator polynomial coefficients (descending power of s)
    """
    # Following Yeh's analysis of the Fender tone stack.
    # The transfer function is 3rd order: H(s) = (b3*s^3 + b2*s^2 + b1*s + b0) /
    #                                             (a3*s^3 + a2*s^2 + a1*s + a0)

    # Intermediate terms from circuit analysis
    b1 = t * C1 * R1 + m * C3 * R3 + l * (C1 * R2 + C2 * R2) + (C1 * R3 + C2 * R3)
    b2 = (t * C1 * C2 * R1 * R4
          + t * C1 * C3 * R1 * R3
          - m * m * C1 * C3 * R3 * R3
          + m * C1 * C3 * R3 * R3     # m*(1-m)*C1*C3*R3^2 = m*C1*C3*R3^2 - m^2*C1*C3*R3^2
          + l * (C1 * C2 * R1 * R2 + C1 * C2 * R2 * R4 + C1 * C3 * R2 * R3)
          + l * m * C1 * C3 * R2 * R3
          + C1 * C2 * R1 * R3 + C1 * C2 * R3 * R4 + C1 * C3 * R3 * R3)
    b3 = (l * m * C1 * C2 * C3 * R1 * R2 * R3
          + l * C1 * C2 * C3 * R2 * R3 * R4
          + t * C1 * C2 * C3 * R1 * R3 * R4
          - t * m * C1 * C2 * C3 * R1 * R3 * R3
          + t * m * C1 * C2 * C3 * R1 * R3 * R3)  # cancels

    # Denominator
    a0 = 1.0
    a1 = ((C1 * R1 + C1 * R3 + C2 * R3 + C2 * R4 + C3 * R4)
          + m * C3 * R3
          + l * (C1 * R2 + C2 * R2))
    a2 = (m * C1 * C3 * R1 * R3
          - m * m * C1 * C3 * R3 * R3
          + m * C1 * C3 * R3 * R3
          + C1 * C2 * R1 * R4
          + C1 * C3 * R1 * R4
          + C1 * C2 * R3 * R4
          + C1 * C2 * R1 * R3
          + C1 * C3 * R3 * R4
          + C2 * C3 * R3 * R4
          + l * (C1 * C2 * R1 * R2
                + C1 * C2 * R2 * R4
                + C1 * C3 * R2 * R4
                + C2 * C3 * R2 * R4)
          + l * m * C1 * C3 * R2 * R3)
    a3 = (l * m * C1 * C2 * C3 * R1 * R2 * R3
          + l * C1 * C2 * C3 * R2 * R3 * R4
          + m * C1 * C2 * C3 * R1 * R3 * R4
          + C1 * C2 * C3 * R1 * R3 * R4
          - m * m * C1 * C2 * C3 * R1 * R3 * R3
          + m * C1 * C2 * C3 * R1 * R3 * R3)

    # b0 = 0 for a passive network (no DC pass)
    b = np.array([b3, b2, b1, 0.0])
    a = np.array([a3, a2, a1, a0])

    return b, a


# =============================================================================
# Fender Bassman Component Values
# =============================================================================

FENDER = dict(
    R1=250e3, R2=1e6, R3=25e3, R4=56e3,
    C1=250e-12, C2=20e-9, C3=20e-9,
)

MARSHALL = dict(
    R1=220e3, R2=1e6, R3=25e3, R4=33e3,
    C1=470e-12, C2=22e-9, C3=22e-9,
)


def compute_biquad_coefficients(t, m, l, components, fs=48000):
    """
    Compute 3 biquad sections (cascaded SOS) for a tone stack setting.

    Returns array of shape (3, 6): [b0, b1, b2, 1, a1, a2] per section,
    or (2, 6) if the order reduces.
    """
    if not HAS_SCIPY:
        raise ImportError("scipy required for tone stack computation")

    b_s, a_s = tonestack_tf(t, m, l, **components)

    # Convert analog TF to zero-pole-gain form
    z_s, p_s, k_s = tf2zpk(b_s, a_s)

    # Bilinear transform analog zpk -> digital zpk
    z_d, p_d, k_d = bilinear_zpk(z_s, p_s, k_s, fs=fs)

    # Convert to cascaded second-order sections
    sos = zpk2sos(z_d, p_d, k_d, pairing='nearest')

    return sos


def sos_to_q214(sos):
    """
    Quantize SOS coefficients to Q2.14 signed 16-bit.
    Returns integer array same shape as sos.
    """
    # Q2.14: multiply by 2^14, clamp to [-32768, 32767]
    q = np.round(sos * (1 << 14)).astype(np.int64)
    q = np.clip(q, -32768, 32767).astype(np.int16)
    return q


def format_verilog_coefficients(name, sos_q, section_labels=None):
    """Format SOS coefficients as Verilog localparams."""
    lines = []
    for i, row in enumerate(sos_q):
        b0, b1, b2, _, a1, a2 = row  # a0 is always 1.0 (=16384 in Q2.14)
        label = section_labels[i] if section_labels else f"BQ{i}"
        lines.append(f"// {name} {label}")
        lines.append(f"localparam signed [15:0] {name}_{label}_B0 = 16'sd{int(b0)};")
        lines.append(f"localparam signed [15:0] {name}_{label}_B1 = 16'sd{int(b1)};")
        lines.append(f"localparam signed [15:0] {name}_{label}_B2 = 16'sd{int(b2)};")
        lines.append(f"localparam signed [15:0] {name}_{label}_A1 = 16'sd{int(a1)};")
        lines.append(f"localparam signed [15:0] {name}_{label}_A2 = 16'sd{int(a2)};")
        lines.append("")
    return "\n".join(lines)


# =============================================================================
# Preset Tone Settings
# =============================================================================

PRESETS = {
    "fender_flat":   (0.5, 0.5, 0.5, FENDER),
    "fender_scooped":(0.7, 0.2, 0.6, FENDER),    # classic Fender scoop
    "fender_bright": (0.8, 0.5, 0.3, FENDER),
    "marshall_flat": (0.5, 0.5, 0.5, MARSHALL),
    "marshall_crunch":(0.6, 0.7, 0.5, MARSHALL),   # Marshall mid-heavy
    "marshall_metal":(0.8, 0.3, 0.7, MARSHALL),
}


# =============================================================================
# Main
# =============================================================================

if __name__ == "__main__":
    if not HAS_SCIPY:
        print("ERROR: scipy required. Install with: pip install scipy")
        exit(1)

    fs = 48000
    section_labels = ["LO", "MID", "HI"]

    print("=" * 70)
    print("Tone Stack Biquad Coefficient Generator")
    print("=" * 70)

    # Compute and display for each preset
    all_sos = {}
    for name, (t, m, l, comp) in PRESETS.items():
        comp_name = "Fender" if comp is FENDER else "Marshall"
        print(f"\n--- {name} (T={t}, M={m}, L={l}, {comp_name}) ---")
        sos = compute_biquad_coefficients(t, m, l, comp, fs=fs)
        sos_q = sos_to_q214(sos)
        all_sos[name] = (sos, sos_q)

        # Pad to 3 sections if needed (a 3rd-order system gives 2 SOS)
        if len(sos_q) < 3:
            # Add unity passthrough section
            unity = np.array([[16384, 0, 0, 16384, 0, 0]], dtype=np.int16)
            sos_q = np.vstack([sos_q, unity])
            # Also pad float sos for plotting
            unity_f = np.array([[1.0, 0, 0, 1, 0, 0]])
            sos = np.vstack([sos, unity_f])
            all_sos[name] = (sos, sos_q)

        print(format_verilog_coefficients(name.upper(), sos_q, section_labels))

    # ── Generate default coefficients for Verilog ────────────────────────
    # Use fender_flat as default, marshall_flat as alternative
    print("\n" + "=" * 70)
    print("Verilog coefficient output (for tone_stack_iir.v)")
    print("=" * 70)

    for preset_name in ["fender_flat", "marshall_flat"]:
        sos, sos_q = all_sos[preset_name]
        print(f"\n// === {preset_name.upper()} ===")

        for i in range(min(3, len(sos_q))):
            b0, b1, b2, _, a1, a2 = sos_q[i]
            label = section_labels[i]
            print(f"// Biquad {i} ({label})")
            print(f"//   b0={b0:6d}  b1={b1:6d}  b2={b2:6d}  a1={a1:6d}  a2={a2:6d}")

    # ── Frequency response plot ──────────────────────────────────────────
    print("\nGenerating frequency response plot...")
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Tone Stack Frequency Response (circuit-derived)", fontsize=13)

    for ax_idx, (amp_name, comp) in enumerate([("Fender Bassman", FENDER), ("Marshall JCM800", MARSHALL)]):
        ax = axes[ax_idx]
        settings = [
            (0.5, 0.5, 0.5, "Flat (5/5/5)", '-'),
            (0.0, 0.5, 0.0, "T=0 M=5 L=0", '--'),
            (1.0, 0.5, 1.0, "T=10 M=5 L=10", '--'),
            (0.7, 0.2, 0.6, "Scooped", '-.'),
            (0.5, 1.0, 0.5, "Mid boost", ':'),
        ]

        for t, m, l, label, ls in settings:
            sos = compute_biquad_coefficients(t, m, l, comp, fs=fs)
            # Pad to 3 sections
            if len(sos) < 3:
                unity_f = np.array([[1.0, 0, 0, 1, 0, 0]])
                sos = np.vstack([sos, unity_f])

            w, h = sosfreqz(sos, worN=4096, fs=fs)
            mag_db = 20 * np.log10(np.abs(h) + 1e-12)
            ax.semilogx(w[1:], mag_db[1:], label=label, linestyle=ls)

        ax.set_xlim(20, 20000)
        ax.set_ylim(-40, 5)
        ax.set_xlabel("Frequency (Hz)")
        ax.set_ylabel("Gain (dB)")
        ax.set_title(amp_name)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3, which='both')

    plt.tight_layout()
    plt.savefig("tonestack_response.png", dpi=150, bbox_inches='tight')
    print("Saved: tonestack_response.png")

    # ── Print the actual coefficients to use in Verilog ──────────────────
    print("\n" + "=" * 70)
    print("COPY THESE INTO tone_stack_iir.v")
    print("=" * 70)

    for preset_name in ["fender_flat", "marshall_flat"]:
        sos, sos_q = all_sos[preset_name]
        bq_labels = ["BASS", "MID", "TREB"]
        print(f"\n// {preset_name.upper()} tone stack coefficients (Q2.14)")
        for i in range(min(3, len(sos_q))):
            b0, b1, b2, _, a1, a2 = sos_q[i]
            prefix = bq_labels[i]
            print(f"localparam signed [15:0] {prefix}_B0 = 16'sd{int(b0):>6};")
            print(f"localparam signed [15:0] {prefix}_B1 = 16'sd{int(b1):>6};")
            print(f"localparam signed [15:0] {prefix}_B2 = 16'sd{int(b2):>6};")
            print(f"localparam signed [15:0] {prefix}_A1 = 16'sd{int(a1):>6};")
            print(f"localparam signed [15:0] {prefix}_A2 = 16'sd{int(a2):>6};")
            print()

    print("Done.")
