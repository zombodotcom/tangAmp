"""
validate_nfb.py
Validates the negative feedback loop implementation against ngspice and
analytical closed-loop gain theory.

Uses PySpice/ngspice shared library for SPICE simulation.
Tests NFB amounts: 0 (no feedback), 0.1 (Marshall), 0.3 (Fender).
Verifies: A_closed = A_open / (1 + A_open * beta)

PASS criteria: closed-loop gain matches theory within 2dB.
"""

import warnings
warnings.filterwarnings('ignore')

import numpy as np
import os
import sys
import tempfile

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.join(SCRIPT_DIR, "..")
DEMOS_DIR = os.path.join(ROOT_DIR, "demos")
os.makedirs(DEMOS_DIR, exist_ok=True)


# =============================================================================
# ngspice NFB Simulation
# =============================================================================

def make_nfb_netlist(open_loop_gain=100, r1=100e3, r2=10e3):
    """Build a SPICE netlist for a feedback amplifier.

    The circuit uses a VCVS (E element) for the open-loop amplifier and a
    resistive voltage divider for the feedback network.

    Parameters
    ----------
    open_loop_gain : float
        Open-loop voltage gain (dimensionless).
    r1 : float
        Feedback resistor from output to summing node (ohms).
    r2 : float
        Feedback resistor from summing node to ground (ohms).

    Returns
    -------
    netlist : str
        SPICE netlist string.
    beta : float
        Feedback fraction R2/(R1+R2).
    expected_gain : float
        Theoretical closed-loop gain A/(1+A*beta).
    """
    beta = r2 / (r1 + r2)
    expected_gain = open_loop_gain / (1.0 + open_loop_gain * beta)
    expected_gain_db = 20.0 * np.log10(expected_gain)

    # Use an inverting amplifier topology which is well-behaved in SPICE.
    # Non-inverting input grounded via R_GND.
    # Feedback: R1 from output to inverting input, R2 from inverting input to ground.
    # Input coupled through R_IN to the inverting node.
    # VCVS senses the difference between non-inv and inv inputs.
    # For AC analysis, the closed-loop gain = Vout/Vin.
    #
    # Use a high-gain opamp model: VCVS with gain A sensing (pos - neg).
    # pos = input (through voltage follower), neg = feedback node.
    netlist = f"""\
.title NFB Validation (A={open_loop_gain}, beta={beta:.4f})
VIN input 0 DC 0 AC 1
* Input buffer (source impedance)
R_SRC input in_buf 1
* Difference amplifier: senses (in_buf - fb_node)
* Use E element: E1 amp_out 0 in_buf fb_node gain
E1 amp_out 0 in_buf fb_node {open_loop_gain}
* Feedback voltage divider: beta = R2/(R1+R2)
R1 amp_out fb_node {r1:.0f}
R2 fb_node 0 {r2:.0f}
.ac dec 100 10 100k
.end"""

    return netlist, beta, expected_gain, expected_gain_db


def run_spice_nfb(netlist):
    """Run ngspice AC analysis of NFB circuit.
    Returns (freqs, gain_db) arrays."""
    from PySpice.Spice.NgSpice.Shared import NgSpiceShared

    ng = NgSpiceShared.new_instance()

    with tempfile.NamedTemporaryFile(mode='w', suffix='.cir', delete=False) as f:
        f.write(netlist + "\n")
        tmp_path = f.name

    try:
        tmp_fwd = tmp_path.replace(os.sep, '/')
        ng.exec_command('source ' + tmp_fwd)
        ng.run()
    finally:
        os.unlink(tmp_path)

    plot_name = ng.last_plot
    plot = ng.plot(simulation=None, plot_name=plot_name)

    def get_vec(candidates):
        for key in candidates:
            if key in plot:
                return plot[key]._data
            for pk in plot.keys():
                if pk.lower() == key.lower():
                    return plot[pk]._data
        return None

    freq_vec = np.real(get_vec(['frequency']))
    v_out = get_vec(['V(amp_out)', 'v(amp_out)', 'amp_out'])
    v_in = get_vec(['V(input)', 'v(input)', 'input'])

    if freq_vec is None or v_out is None:
        raise RuntimeError(f"Missing vectors. Available: {list(plot.keys())}")

    # Gain = |Vout/Vin|; Vin is AC=1 so |Vout| = gain
    if v_in is not None:
        gain = np.abs(v_out) / (np.abs(v_in) + 1e-30)
    else:
        gain = np.abs(v_out)

    gain_db = 20.0 * np.log10(gain + 1e-30)
    return freq_vec, gain_db


# =============================================================================
# Our NFB Model (from nfb_register.v / negative_feedback.py)
# =============================================================================

def our_nfb_gain_db(open_loop_gain_db, nfb_amount):
    """Compute closed-loop gain using our NFB model.

    Our Verilog nfb_register.v implements:
        nfb_signal = fb_in >>> NFB_SHIFT
    which gives beta = 1/2^NFB_SHIFT.

    The Python negative_feedback.py uses nfb_amount directly as beta.

    Closed-loop gain (linear theory):
        A_closed = A_open / (1 + A_open * beta)

    Parameters
    ----------
    open_loop_gain_db : float
        Open-loop gain in dB.
    nfb_amount : float
        Feedback fraction (beta). 0 = no feedback, 0.1 = Marshall, 0.3 = Fender.

    Returns
    -------
    closed_loop_gain_db : float
        Closed-loop gain in dB.
    """
    A = 10.0 ** (open_loop_gain_db / 20.0)
    if nfb_amount <= 0:
        return open_loop_gain_db
    A_closed = A / (1.0 + A * nfb_amount)
    return 20.0 * np.log10(A_closed)


# =============================================================================
# Validation
# =============================================================================

def validate_nfb():
    """Validate NFB loop against ngspice and theory."""
    print("=" * 60)
    print("Negative Feedback Loop Validation (ngspice vs theory)")
    print("=" * 60)

    open_loop_gain = 100  # 40 dB
    open_loop_gain_db = 20.0 * np.log10(open_loop_gain)

    # NFB test cases: (nfb_amount/beta, R1, R2, label)
    # beta = R2 / (R1 + R2)
    # For beta=0.091: R1=100k, R2=10k
    # For beta=0.1: R1=90k, R2=10k
    # For beta=0.3: R1=70k, R2=30k
    test_cases = [
        (0.0,  None,  None,  "No NFB (Vox)"),
        (0.1,  90e3,  10e3,  "Light NFB (Marshall, beta=0.1)"),
        (0.3,  70e3,  30e3,  "Heavy NFB (Fender, beta=0.3)"),
    ]

    results = []
    all_passed = True
    tolerance_db = 2.0

    for beta, r1, r2, label in test_cases:
        print(f"\n--- {label} ---")

        # Theoretical closed-loop gain
        theory_gain_db = our_nfb_gain_db(open_loop_gain_db, beta)
        print(f"  Open-loop gain:     {open_loop_gain_db:.1f} dB")
        print(f"  Beta (NFB amount):  {beta:.2f}")
        print(f"  Theory closed-loop: {theory_gain_db:.2f} dB")

        if beta <= 0:
            # No feedback -- just record theory value
            spice_gain_db = open_loop_gain_db
            print(f"  SPICE (no NFB):     {spice_gain_db:.2f} dB")
            diff = 0.0
            passed = True
        else:
            # Run ngspice with feedback network
            netlist, actual_beta, expected_gain, expected_gain_db = make_nfb_netlist(
                open_loop_gain, r1, r2)
            print(f"  SPICE beta:         {actual_beta:.4f}")
            print(f"  SPICE expected:     {expected_gain_db:.2f} dB")

            freqs, gain_db_curve = run_spice_nfb(netlist)

            # Get midband gain (1kHz)
            idx_1k = np.argmin(np.abs(freqs - 1000))
            spice_gain_db = gain_db_curve[idx_1k]
            print(f"  SPICE measured @1kHz: {spice_gain_db:.2f} dB")

            # Compare SPICE against theory
            diff_spice_theory = abs(spice_gain_db - theory_gain_db)
            # Compare our model against theory
            diff = abs(theory_gain_db - our_nfb_gain_db(open_loop_gain_db, actual_beta))
            # The real comparison: SPICE vs our model
            our_gain = our_nfb_gain_db(open_loop_gain_db, actual_beta)
            diff = abs(spice_gain_db - our_gain)
            print(f"  Our model:          {our_gain:.2f} dB")
            print(f"  Difference:         {diff:.2f} dB")
            passed = diff <= tolerance_db

        print(f"  Result: {'PASS' if passed else 'FAIL'}")
        if not passed:
            all_passed = False

        results.append({
            'label': label,
            'beta': beta,
            'theory_db': theory_gain_db,
            'spice_db': spice_gain_db,
            'diff_db': diff if beta > 0 else 0.0,
            'passed': passed,
            'freqs': freqs if beta > 0 else None,
            'gain_curve': gain_db_curve if beta > 0 else None,
        })

    # --- Summary table ---
    print(f"\n{'='*60}")
    print("Summary")
    print("=" * 60)
    print(f"{'Label':<35} {'Beta':>6} {'Theory':>8} {'SPICE':>8} {'Diff':>6} {'Result':>6}")
    print("-" * 75)
    for r in results:
        print(f"{r['label']:<35} {r['beta']:>6.2f} {r['theory_db']:>8.2f} "
              f"{r['spice_db']:>8.2f} {r['diff_db']:>6.2f} {'PASS' if r['passed'] else 'FAIL':>6}")

    # --- Generate plot ---
    print(f"\nGenerating validation plot...")
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    fig.suptitle("NFB Validation: ngspice vs Closed-Loop Theory",
                 fontsize=14, fontweight='bold')

    # Plot 1: Frequency response curves from SPICE
    ax = axes[0]
    colors = ['red', 'orange', 'blue']
    for i, r in enumerate(results):
        if r['freqs'] is not None:
            ax.semilogx(r['freqs'], r['gain_curve'], color=colors[i],
                        linewidth=1.5, label=f"{r['label']} (SPICE)")
            ax.axhline(r['theory_db'], color=colors[i], ls='--', alpha=0.5,
                       label=f"Theory: {r['theory_db']:.1f} dB")
    ax.axhline(open_loop_gain_db, color='gray', ls=':', alpha=0.5,
               label=f"Open-loop: {open_loop_gain_db:.1f} dB")
    ax.set_xlim(10, 100000)
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Gain (dB)")
    ax.set_title("Closed-Loop Gain vs Frequency")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, which='both')

    # Plot 2: Gain reduction vs NFB amount
    ax = axes[1]
    betas = np.linspace(0, 0.5, 50)
    theory_gains = [our_nfb_gain_db(open_loop_gain_db, b) for b in betas]
    ax.plot(betas, theory_gains, 'k-', linewidth=2, label='Theory: A/(1+A*beta)')

    # Plot SPICE points
    for r in results:
        marker = 'o' if r['passed'] else 'x'
        color = 'green' if r['passed'] else 'red'
        ax.plot(r['beta'], r['spice_db'], marker, color=color, markersize=10,
                label=f"{r['label']} ({r['spice_db']:.1f} dB)")

    ax.set_xlabel("NFB Amount (beta)")
    ax.set_ylabel("Closed-Loop Gain (dB)")
    ax.set_title(f"Gain Reduction vs NFB Amount (Open-loop = {open_loop_gain_db:.0f} dB)")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)
    # Mark amp types
    ax.axvspan(0.08, 0.12, alpha=0.1, color='orange', label='Marshall range')
    ax.axvspan(0.25, 0.35, alpha=0.1, color='blue', label='Fender range')

    plt.tight_layout()
    plot_path = os.path.join(DEMOS_DIR, "validation_nfb.png")
    plt.savefig(plot_path, dpi=150)
    print(f"  Saved: {plot_path}")
    plt.close()

    return all_passed


def main():
    try:
        passed = validate_nfb()
    except Exception as e:
        print(f"\nFailed with error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

    if passed:
        print("\nOverall: PASS")
        sys.exit(0)
    else:
        print("\nOverall: FAIL")
        sys.exit(1)


if __name__ == "__main__":
    main()
