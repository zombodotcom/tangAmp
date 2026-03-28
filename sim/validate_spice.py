"""
validate_spice.py
Uses ngspice (via PySpice) to independently validate the WDF triode simulation.
Compares SPICE transient analysis against WDF reference values.
"""

import warnings
warnings.filterwarnings('ignore')

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import sys

# WDF reference values (from wdf_triode_sim_wdf.py, with cathode bypass cap)
WDF_VPLATE_DC = 146.2     # V  (DC unchanged -- cap is open at DC)
WDF_IP_DC     = 0.538e-3  # A (0.538 mA)
WDF_GAIN_DB   = 34.1      # dB (with Ck=22uF bypass)

# Tolerances
DC_TOL_PCT    = 5.0   # percent
GAIN_TOL_DB   = 3.0   # dB

NETLIST = """\
.title 12AX7 Common Cathode Validation
VB supply 0 DC 200
RP supply plate 100k
VIN input 0 DC 0 SIN(0 0.5 440)
CIN input grid 22n
RG grid 0 1MEG
RK cathode 0 1.5k
CK cathode 0 22u
BEd ed 0 V = V(plate,cathode)/600 * ln(1 + exp(600*(0.01 + V(grid,cathode)/sqrt(300 + V(plate,cathode)*V(plate,cathode) + 0.000001))))
BIp plate cathode I = pwr(max(V(ed),0), 1.4) / 1060
.options reltol=0.001 abstol=1n vntol=1u
.ic V(plate)=146 V(cathode)=0.8 V(grid)=0 V(ed)=0.5
.tran 20.833u 141.67m uic
.end"""


def save_netlist_file():
    """Save netlist as .cir file for manual use."""
    with open("spice_triode.cir", "w") as f:
        f.write(NETLIST + "\n")
    print("Saved netlist: spice_triode.cir")


def run_spice_simulation():
    """Run SPICE simulation via PySpice/ngspice shared library.
    Returns (time, v_plate, v_input, v_cathode) arrays."""
    import tempfile
    import os
    from PySpice.Spice.NgSpice.Shared import NgSpiceShared

    ng = NgSpiceShared.new_instance()

    # ngspice 45: load_circuit() is broken, use source via temp file instead
    with tempfile.NamedTemporaryFile(mode='w', suffix='.cir', delete=False) as f:
        f.write(NETLIST + "\n")
        tmp_path = f.name

    try:
        tmp_fwd = tmp_path.replace(os.sep, '/')
        print(f"Loading netlist from: {tmp_fwd}")
        ng.exec_command('source ' + tmp_fwd)
        ng.run()
    finally:
        os.unlink(tmp_path)

    plot_name = ng.last_plot
    print(f"Last plot: {plot_name}")

    # Extract vectors from the plot object
    plot = ng.plot(simulation=None, plot_name=plot_name)
    available_keys = list(plot.keys())
    print(f"Available vectors: {available_keys}")

    def get_vector(plot, candidates):
        """Try multiple key names, return numpy array from Vector._data."""
        for key in candidates:
            if key in plot:
                return np.real(plot[key]._data)
            # Try case-insensitive match
            for pk in plot.keys():
                if pk.lower() == key.lower():
                    return np.real(plot[pk]._data)
        return None

    time_vec = get_vector(plot, ['time'])
    v_plate = get_vector(plot, ['V(plate)', 'v(plate)', 'plate'])
    v_input = get_vector(plot, ['V(input)', 'v(input)', 'input'])
    v_cathode = get_vector(plot, ['V(cathode)', 'v(cathode)', 'cathode'])

    if time_vec is None or v_plate is None:
        raise RuntimeError(
            f"Could not find required vectors. Available: {available_keys}"
        )

    print(f"Extracted {len(time_vec)} data points")
    return time_vec, v_plate, v_input, v_cathode


def analyze_and_compare(time_vec, v_plate, v_input, v_cathode):
    """Analyze SPICE results and compare to WDF reference."""
    dt = time_vec[1] - time_vec[0]
    total_time = time_vec[-1]
    print(f"\nSPICE sim: {len(time_vec)} points, dt={dt*1e6:.2f}us, total={total_time*1000:.1f}ms")

    # DC operating point: average plate voltage over settled region (after 40ms)
    settle_mask = time_vec > 0.04
    vp_dc = np.mean(v_plate[settle_mask])

    # AC analysis: use last 50ms (well-settled with signal)
    ac_mask = time_vec > 0.09
    vp_ac = v_plate[ac_mask] - np.mean(v_plate[ac_mask])
    if v_input is not None:
        vi_ac = v_input[ac_mask] - np.mean(v_input[ac_mask])
    else:
        # Reconstruct input: 0.5 * sin(2*pi*440*t)
        vi_ac = 0.5 * np.sin(2 * np.pi * 440 * time_vec[ac_mask])

    out_rms = np.sqrt(np.mean(vp_ac**2))
    in_rms = np.sqrt(np.mean(vi_ac**2))
    gain_db = 20 * np.log10(out_rms / (in_rms + 1e-12)) if in_rms > 1e-6 else 0.0

    # Plate current estimate: Ip = (VB - Vplate) / RP
    ip_dc = (200.0 - vp_dc) / 100000.0

    print(f"\n=== SPICE Results ===")
    print(f"Vplate DC:  {vp_dc:.2f} V")
    print(f"Ip DC:      {ip_dc*1000:.3f} mA")
    print(f"AC Gain:    {gain_db:.1f} dB")
    print(f"Output RMS: {out_rms:.3f} V")
    print(f"Input RMS:  {in_rms:.3f} V")

    print(f"\n=== WDF Reference ===")
    print(f"Vplate DC:  {WDF_VPLATE_DC:.2f} V")
    print(f"Ip DC:      {WDF_IP_DC*1000:.3f} mA")
    print(f"AC Gain:    {WDF_GAIN_DB:.1f} dB")

    # Comparison
    dc_err_pct = abs(vp_dc - WDF_VPLATE_DC) / WDF_VPLATE_DC * 100
    gain_err_db = abs(gain_db - WDF_GAIN_DB)

    print(f"\n=== Comparison ===")
    print(f"DC error:   {dc_err_pct:.2f}% (tolerance: {DC_TOL_PCT}%)")
    print(f"Gain error: {gain_err_db:.2f} dB (tolerance: {GAIN_TOL_DB} dB)")

    dc_pass = dc_err_pct < DC_TOL_PCT
    gain_pass = gain_err_db < GAIN_TOL_DB

    print(f"DC:   {'PASS' if dc_pass else 'FAIL'}")
    print(f"Gain: {'PASS' if gain_pass else 'FAIL'}")

    overall = dc_pass and gain_pass
    print(f"\nOverall: {'PASS' if overall else 'FAIL'}")

    return overall, vp_dc, gain_db, vp_ac, vi_ac, time_vec[ac_mask]


def generate_plot(time_vec, v_plate, v_input, v_cathode, vp_dc, gain_db):
    """Generate validation_spice.png with waveform overlay."""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle("SPICE vs WDF Validation -- 12AX7 Common Cathode", fontsize=14)

    t_ms = time_vec * 1000

    # Plot 1: Full plate voltage
    ax = axes[0]
    ax.plot(t_ms, v_plate, 'r', linewidth=0.8, label=f'SPICE Vplate (DC={vp_dc:.1f}V)')
    ax.axhline(WDF_VPLATE_DC, color='blue', linestyle='--', alpha=0.7,
               label=f'WDF ref DC={WDF_VPLATE_DC}V')
    ax.set_ylabel("Plate Voltage (V)")
    ax.set_title("Plate Voltage -- Full Simulation")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Plot 2: Zoomed AC region (last few cycles)
    ac_mask = time_vec > 0.13  # last ~11ms, about 5 cycles at 440Hz
    if np.sum(ac_mask) > 10:
        ax = axes[1]
        vp_ac = v_plate[ac_mask] - np.mean(v_plate[ac_mask])
        t_ac = t_ms[ac_mask] - t_ms[ac_mask][0]
        ax.plot(t_ac, vp_ac, 'r', linewidth=1.2, label=f'SPICE output AC ({gain_db:.1f}dB)')
        if v_input is not None:
            vi = v_input[ac_mask]
            scale = np.max(np.abs(vp_ac)) / (np.max(np.abs(vi)) + 1e-12)
            ax.plot(t_ac, vi * scale, 'b', linewidth=0.8, alpha=0.5,
                    label=f'Input (scaled x{scale:.0f})')
        ax.set_ylabel("Voltage (V)")
        ax.set_title(f"AC Output (gain={gain_db:.1f}dB, WDF ref={WDF_GAIN_DB}dB)")
        ax.legend()
        ax.grid(True, alpha=0.3)

    # Plot 3: FFT of output
    ax = axes[2]
    ac_full_mask = time_vec > 0.09
    vp_fft = v_plate[ac_full_mask] - np.mean(v_plate[ac_full_mask])
    N = len(vp_fft)
    if N > 0:
        dt = time_vec[1] - time_vec[0]
        fs = 1.0 / dt
        w = np.hanning(N)
        sp = np.abs(np.fft.rfft(vp_fft * w))
        sp_db = 20 * np.log10(sp / (np.max(sp) + 1e-12) + 1e-12)
        f = np.fft.rfftfreq(N, dt)
        ax.plot(f, sp_db, 'r', linewidth=0.8)
        ax.set_xlim(0, 5000)
        ax.set_ylim(-80, 5)
        for h in range(1, 8):
            ax.axvline(440 * h, color='gray', alpha=0.3, ls='--', lw=0.5)
        ax.set_xlabel("Frequency (Hz)")
        ax.set_ylabel("dB")
        ax.set_title("SPICE Output Spectrum")
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("validation_spice.png", dpi=150)
    print("\nSaved: validation_spice.png")


def main():
    print("=" * 60)
    print("SPICE Validation of WDF 12AX7 Triode Simulation")
    print("=" * 60)

    try:
        time_vec, v_plate, v_input, v_cathode = run_spice_simulation()
    except Exception as e:
        print(f"\nPySpice/ngspice failed: {e}")
        print("Saving netlist for manual simulation...")
        save_netlist_file()
        sys.exit(1)

    overall, vp_dc, gain_db, vp_ac, vi_ac, t_ac = analyze_and_compare(
        time_vec, v_plate, v_input, v_cathode
    )

    generate_plot(time_vec, v_plate, v_input, v_cathode, vp_dc, gain_db)
    save_netlist_file()

    if not overall:
        print("\nValidation FAILED -- see details above")
        sys.exit(1)
    else:
        print("\nValidation PASSED")
        sys.exit(0)


if __name__ == "__main__":
    main()
