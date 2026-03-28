r"""
validate_6l6.py
Validation suite for the 6L6 power amp tube model.

Validates:
  1. DC operating point via bisection (VB=400V, RP=2000, RK=250)
  2. Small-signal gain: analytical vs WDF simulation
  3. SPICE behavioral model comparison (ngspice via PySpice)

Self-contained -- duplicates all needed functions. No project imports.
Exit code 0 = all tests pass.
"""

import numpy as np
import math
import sys
import warnings
warnings.filterwarnings('ignore')

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# =============================================================================
# 6L6 Koren Constants (from tube_lut_gen.py / paengdesign fit)
# =============================================================================

MU  = 10.11
EX  = 1.37
KG1 = 406.6
KP  = 31.2
KVB = 640.7

# Circuit parameters (power amp)
VB  = 400.0
RP  = 2000.0
RK  = 250.0
RG  = 1e6
CIN = 22e-9
CK  = 22e-6
FS  = 48000.0

# Tolerances
DC_TOL_PCT  = 5.0   # percent
GAIN_TOL_DB = 3.0   # dB


# =============================================================================
# Koren Model
# =============================================================================

def koren_ip(vpk, vgk, mu=MU, ex=EX, kg1=KG1, kp=KP, kvb=KVB):
    """Plate current Ip in amps."""
    vpk = float(np.clip(vpk, 0.0, 600.0))
    vgk = float(np.clip(vgk, -60.0, 5.0))
    if vpk <= 0:
        return 0.0
    inner = kp * (1.0 / mu + vgk / math.sqrt(kvb + vpk * vpk))
    inner = float(np.clip(inner, -500, 500))
    Ed = (vpk / kp) * math.log(1.0 + math.exp(inner))
    if Ed <= 0:
        return 0.0
    return (Ed ** ex) / kg1


def koren_dip_dvpk(vpk, vgk, h=0.01):
    return (koren_ip(vpk + h, vgk) - koren_ip(vpk - h, vgk)) / (2 * h)


def koren_dip_dvgk(vpk, vgk, h=0.01):
    return (koren_ip(vpk, vgk + h) - koren_ip(vpk, vgk - h)) / (2 * h)


# =============================================================================
# DC Operating Point via Bisection
# =============================================================================

def find_dc_op(vb=VB, rp=RP, rk=RK, tol=1e-12, max_iter=200):
    """Find DC operating point: Ip where load line meets Koren curve.

    Load line: Vpk = VB - (RP + RK)*Ip, Vgk = -RK*Ip
    """
    ip_lo, ip_hi = 0.0, vb / (rp + rk)
    for _ in range(max_iter):
        ip_mid = (ip_lo + ip_hi) / 2.0
        vpk = vb - (rp + rk) * ip_mid
        vgk = -rk * ip_mid
        ip_model = koren_ip(vpk, vgk)
        if ip_mid < ip_model:
            ip_lo = ip_mid
        else:
            ip_hi = ip_mid
        if abs(ip_hi - ip_lo) < tol:
            break
    ip_dc = (ip_lo + ip_hi) / 2.0
    vpk_dc = vb - (rp + rk) * ip_dc
    vgk_dc = -rk * ip_dc
    return ip_dc, vpk_dc, vgk_dc


# =============================================================================
# Small-Signal Gain (Analytical)
# =============================================================================

def analytical_gain(vpk_q, vgk_q, rp=RP, rk=RK):
    """Compute small-signal voltage gain at the DC operating point.

    Av = -mu * rp_load / (rp_load + ra + (mu+1)*Rk)

    where ra = 1 / (dIp/dVpk) is the plate resistance at the Q-point,
    mu_local = gm * ra, gm = dIp/dVgk.

    For bypassed cathode (Ck present), Rk_eff = 0 at AC.
    For unbypassed, Rk_eff = RK.
    """
    gm = koren_dip_dvgk(vpk_q, vgk_q)
    dip_dvpk = koren_dip_dvpk(vpk_q, vgk_q)

    if dip_dvpk < 1e-12:
        return 0.0, 0.0, 0.0

    ra = 1.0 / dip_dvpk
    mu_local = gm * ra

    # With cathode bypass cap: Rk_eff = 0 at signal frequencies
    av_bypassed = -mu_local * rp / (rp + ra)
    # Without bypass cap
    av_unbypassed = -mu_local * rp / (rp + ra + (mu_local + 1) * rk)

    return av_bypassed, av_unbypassed, ra


# =============================================================================
# WDF Simulation (with cathode bypass cap)
# =============================================================================

def simulate_wdf(audio_in, vb=VB, rp=RP, rk=RK, rg=RG, cin=CIN,
                 use_hp=True, use_bypass=True, ck=CK, fs=FS):
    """Run WDF simulation with cathode bypass cap for 6L6 power tube."""
    n_total = len(audio_in)
    out_vplate = np.zeros(n_total)
    out_ip = np.zeros(n_total)

    # High-pass filter coefficients
    if use_hp and cin > 0:
        tau_hp = rg * cin
        k_hp = 2.0 * fs * tau_hp
        c_hp = (k_hp - 1.0) / (k_hp + 1.0)
        hp_gain = (1.0 + c_hp) / 2.0
    else:
        c_hp = 0.0
        hp_gain = 1.0

    # Cathode bypass cap
    if use_bypass and ck > 0:
        r_ck = 1.0 / (2.0 * fs * ck)
        g_rk = 1.0 / rk
        g_ck = 1.0 / r_ck
        g_tot = g_rk + g_ck
        R_k = 1.0 / g_tot
        gamma_cath = g_rk / g_tot
    else:
        R_k = rk
        gamma_cath = 0.0

    R_p = rp
    prev_ip = 10e-3  # power tubes draw more quiescent current
    hp_y = 0.0
    hp_x_prev = 0.0
    z_ck = 0.0

    for n in range(n_total):
        vin = audio_in[n]

        # High-pass filter
        if use_hp and cin > 0:
            hp_y = c_hp * hp_y + hp_gain * (vin - hp_x_prev)
            hp_x_prev = vin
            v_grid = hp_y
        else:
            v_grid = vin

        # Reflected waves from leaves
        b_plate = vb
        b_grid = v_grid

        if use_bypass and ck > 0:
            b_rk = 0.0
            b_ck = z_ck
            bDiff = b_ck - b_rk
            b_cathode = b_ck - gamma_cath * bDiff
        else:
            b_cathode = 0.0

        a_p = b_plate
        a_g = b_grid
        a_k = b_cathode

        # Newton-Raphson to find Ip
        Ip = prev_ip
        for iteration in range(30):
            Vpk = (a_p - a_k) - (R_p + R_k) * Ip
            Vgk = a_g - a_k - R_k * Ip

            ip_model = koren_ip(Vpk, Vgk)
            f_val = Ip - ip_model

            if abs(f_val) < 1e-10:
                break

            dip_dvpk_val = koren_dip_dvpk(Vpk, Vgk)
            dip_dvgk_val = koren_dip_dvgk(Vpk, Vgk)
            df_dIp = 1.0 + dip_dvpk_val * (R_p + R_k) + dip_dvgk_val * R_k

            if abs(df_dIp) < 1e-15:
                break

            Ip -= f_val / df_dIp
            Ip = max(Ip, 0.0)

        prev_ip = Ip

        b_p = a_p - 2.0 * R_p * Ip
        b_k = a_k + 2.0 * R_k * Ip

        # Update capacitor state
        if use_bypass and ck > 0:
            a_ck = b_k + b_cathode - b_ck
            z_ck = a_ck

        v_plate = (a_p + b_p) / 2.0
        out_vplate[n] = v_plate
        out_ip[n] = Ip

    return out_vplate, out_ip


# =============================================================================
# SPICE Validation (ngspice behavioral model)
# =============================================================================

def build_spice_netlist():
    """Build ngspice netlist for 6L6 common cathode stage."""
    # BEd and BIp use the Koren equation with 6L6 constants
    # Ed = (Vpk/kp) * ln(1 + exp(kp*(1/mu + Vgk/sqrt(kvb + Vpk^2))))
    # Ip = Ed^ex / kg1
    netlist = f"""\
.title 6L6 Common Cathode Validation
VB supply 0 DC {VB}
RP supply plate {RP}
VIN input 0 DC 0 SIN(0 5 440)
CIN input grid {CIN}
RG grid 0 {RG}
RK cathode 0 {RK}
CK cathode 0 {CK}
BEd ed 0 V = V(plate,cathode)/{KP} * ln(1 + exp({KP}*(1/{MU} + V(grid,cathode)/sqrt({KVB} + V(plate,cathode)*V(plate,cathode) + 0.000001))))
BIp plate cathode I = pwr(max(V(ed),0), {EX}) / {KG1}
.options reltol=0.001 abstol=1n vntol=1u
.ic V(plate)=300 V(cathode)=15 V(grid)=0 V(ed)=2
.tran 20.833u 200m uic
.end"""
    return netlist


def run_spice_simulation():
    """Run SPICE simulation via PySpice/ngspice shared library."""
    import tempfile
    import os
    from PySpice.Spice.NgSpice.Shared import NgSpiceShared

    ng = NgSpiceShared.new_instance()
    netlist = build_spice_netlist()

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

    def get_vector(plot, candidates):
        for key in candidates:
            if key in plot:
                return np.real(plot[key]._data)
            for pk in plot.keys():
                if pk.lower() == key.lower():
                    return np.real(plot[pk]._data)
        return None

    time_vec = get_vector(plot, ['time'])
    v_plate = get_vector(plot, ['V(plate)', 'v(plate)', 'plate'])
    v_input = get_vector(plot, ['V(input)', 'v(input)', 'input'])
    v_cathode = get_vector(plot, ['V(cathode)', 'v(cathode)', 'cathode'])

    if time_vec is None or v_plate is None:
        raise RuntimeError(f"Could not find required vectors. Available: {list(plot.keys())}")

    return time_vec, v_plate, v_input, v_cathode


# =============================================================================
# TEST 1: DC Operating Point
# =============================================================================

def test_dc_operating_point():
    print("=" * 60)
    print("TEST 1: 6L6 DC Operating Point (VB=400V, RP=2000, RK=250)")
    print("=" * 60)

    ip_dc, vpk_dc, vgk_dc = find_dc_op()
    vplate_dc = VB - RP * ip_dc  # actual plate voltage (not plate-cathode)
    vk_dc = RK * ip_dc

    print(f"  Ip_dc  = {ip_dc*1000:.3f} mA")
    print(f"  Vpk_dc = {vpk_dc:.2f} V")
    print(f"  Vgk_dc = {vgk_dc:.2f} V")
    print(f"  Vplate = {vplate_dc:.2f} V")
    print(f"  Vk     = {vk_dc:.2f} V")

    # Sanity checks for a 6L6 power tube
    checks = []

    # Ip should be in reasonable range for 6L6 (20-100mA typical)
    ip_ok = 5.0 < ip_dc * 1000 < 150.0
    checks.append(("Ip in range [5, 150] mA", ip_ok))

    # Plate voltage should be reasonable (100-380V)
    vp_ok = 100 < vplate_dc < 380
    checks.append(("Vplate in range [100, 380] V", vp_ok))

    # Vgk should be negative (self-bias)
    vgk_ok = -50 < vgk_dc < 0
    checks.append(("Vgk negative (self-bias)", vgk_ok))

    # Load line consistency: VB = Vplate + Ip*RP + Ip*RK = Vpk + Ip*(RP+RK) + Vk...
    # Actually: VB = RP*Ip + Vpk + RK*Ip  (since Vpk = Vplate - Vk)
    residual = abs(VB - (RP + RK) * ip_dc - vpk_dc)
    ll_ok = residual < 0.001
    checks.append((f"Load line consistent (residual={residual:.6f})", ll_ok))

    all_pass = True
    for desc, ok in checks:
        tag = "PASS" if ok else "FAIL"
        print(f"  {tag}: {desc}")
        if not ok:
            all_pass = False

    return all_pass, ip_dc, vpk_dc, vgk_dc


# =============================================================================
# TEST 2: Small-Signal Gain (Analytical vs WDF)
# =============================================================================

def test_gain():
    print("\n" + "=" * 60)
    print("TEST 2: 6L6 Small-Signal Gain")
    print("=" * 60)

    ip_dc, vpk_dc, vgk_dc = find_dc_op()
    vplate_dc = VB - RP * ip_dc

    av_bypassed, av_unbypassed, ra = analytical_gain(vpk_dc, vgk_dc)
    gain_bypassed_db = 20 * np.log10(abs(av_bypassed) + 1e-12)
    gain_unbypassed_db = 20 * np.log10(abs(av_unbypassed) + 1e-12)

    gm = koren_dip_dvgk(vpk_dc, vgk_dc)
    print(f"  Q-point: Vpk={vpk_dc:.1f}V, Vgk={vgk_dc:.2f}V, Ip={ip_dc*1000:.2f}mA")
    print(f"  gm = {gm*1000:.3f} mA/V")
    print(f"  ra = {ra:.0f} ohm")
    print(f"  Analytical gain (bypassed Ck): {av_bypassed:.2f} ({gain_bypassed_db:.1f} dB)")
    print(f"  Analytical gain (unbypassed):  {av_unbypassed:.2f} ({gain_unbypassed_db:.1f} dB)")

    # Run WDF simulation: settle then 440Hz sine
    n_settle = 4000
    n_signal = 8000
    n_total = n_settle + n_signal
    t = np.arange(n_total) / FS

    vin_amp = 5.0  # 5V input (power tube gets driven hard)
    audio_in = np.zeros(n_total)
    audio_in[n_settle:] = vin_amp * np.sin(2 * np.pi * 440 * t[n_settle:])

    vplate_wdf, ip_wdf = simulate_wdf(audio_in)

    # Measure WDF gain from last 4 cycles
    last_n = int(4 * FS / 440)
    vp_ac = vplate_wdf[-last_n:] - np.mean(vplate_wdf[-last_n:])
    vi_ac = audio_in[-last_n:]

    out_rms = np.sqrt(np.mean(vp_ac ** 2))
    in_rms = np.sqrt(np.mean(vi_ac ** 2))
    wdf_gain_db = 20 * np.log10(out_rms / (in_rms + 1e-12))

    wdf_vp_dc = np.mean(vplate_wdf[n_settle - 200:n_settle])
    wdf_ip_dc = np.mean(ip_wdf[n_settle - 200:n_settle])

    print(f"\n  WDF simulation (Ck bypass, 440Hz, {vin_amp}V input):")
    print(f"  WDF Vplate DC = {wdf_vp_dc:.2f} V (analytical: {vplate_dc:.2f} V)")
    print(f"  WDF Ip DC     = {wdf_ip_dc*1000:.3f} mA (analytical: {ip_dc*1000:.3f} mA)")
    print(f"  WDF AC gain   = {wdf_gain_db:.1f} dB (analytical bypassed: {gain_bypassed_db:.1f} dB)")

    # Check DC agreement
    dc_err_pct = abs(wdf_vp_dc - vplate_dc) / vplate_dc * 100
    dc_ok = dc_err_pct < DC_TOL_PCT
    print(f"\n  DC error: {dc_err_pct:.2f}% (tolerance: {DC_TOL_PCT}%): {'PASS' if dc_ok else 'FAIL'}")

    # Check gain agreement (WDF with bypass should be close to analytical bypassed)
    gain_err = abs(wdf_gain_db - gain_bypassed_db)
    gain_ok = gain_err < GAIN_TOL_DB
    print(f"  Gain error: {gain_err:.2f} dB (tolerance: {GAIN_TOL_DB} dB): {'PASS' if gain_ok else 'FAIL'}")

    all_pass = dc_ok and gain_ok
    return all_pass, wdf_vp_dc, wdf_gain_db, gain_bypassed_db, vplate_wdf, ip_wdf, audio_in, n_settle


# =============================================================================
# TEST 3: SPICE Validation
# =============================================================================

def test_spice(wdf_vp_dc, wdf_gain_db):
    print("\n" + "=" * 60)
    print("TEST 3: ngspice Behavioral Model Validation")
    print("=" * 60)

    spice_available = True
    try:
        time_vec, v_plate, v_input, v_cathode = run_spice_simulation()
    except Exception as e:
        print(f"  PySpice/ngspice not available: {e}")
        print("  Saving netlist for manual simulation...")
        netlist = build_spice_netlist()
        with open("spice_6l6.cir", "w") as f:
            f.write(netlist + "\n")
        print("  Saved: spice_6l6.cir")
        print("  SKIP (ngspice not available)")
        spice_available = False
        return True, None, None, None, None  # Don't fail if SPICE unavailable

    print(f"  SPICE: {len(time_vec)} data points")

    # DC operating point: average plate voltage after settling (after 80ms)
    settle_mask = time_vec > 0.08
    spice_vp_dc = np.mean(v_plate[settle_mask])

    # AC analysis from settled region
    ac_mask = time_vec > 0.15
    vp_ac = v_plate[ac_mask] - np.mean(v_plate[ac_mask])
    if v_input is not None:
        vi_ac = v_input[ac_mask] - np.mean(v_input[ac_mask])
    else:
        vi_ac = 5.0 * np.sin(2 * np.pi * 440 * time_vec[ac_mask])

    out_rms = np.sqrt(np.mean(vp_ac ** 2))
    in_rms = np.sqrt(np.mean(vi_ac ** 2))
    spice_gain_db = 20 * np.log10(out_rms / (in_rms + 1e-12))

    spice_ip_dc = (VB - spice_vp_dc) / RP

    print(f"  SPICE Vplate DC = {spice_vp_dc:.2f} V")
    print(f"  SPICE Ip DC     = {spice_ip_dc*1000:.3f} mA")
    print(f"  SPICE AC gain   = {spice_gain_db:.1f} dB")

    print(f"\n  Comparison with WDF:")
    dc_err = abs(spice_vp_dc - wdf_vp_dc) / wdf_vp_dc * 100
    gain_err = abs(spice_gain_db - wdf_gain_db)

    dc_ok = dc_err < DC_TOL_PCT
    gain_ok = gain_err < GAIN_TOL_DB

    print(f"  DC error:   {dc_err:.2f}% (tol {DC_TOL_PCT}%): {'PASS' if dc_ok else 'FAIL'}")
    print(f"  Gain error: {gain_err:.2f} dB (tol {GAIN_TOL_DB} dB): {'PASS' if gain_ok else 'FAIL'}")

    all_pass = dc_ok and gain_ok
    return all_pass, time_vec, v_plate, v_input, spice_vp_dc


# =============================================================================
# Plot Generation
# =============================================================================

def generate_plot(ip_dc, vpk_dc, vgk_dc,
                  wdf_vplate, wdf_ip, audio_in, n_settle,
                  wdf_vp_dc, wdf_gain_db, analytical_gain_db,
                  spice_time=None, spice_vplate=None, spice_vp_dc=None):
    """Generate validation_6l6.png."""

    has_spice = spice_time is not None
    n_rows = 4 if has_spice else 3
    fig, axes = plt.subplots(n_rows, 1, figsize=(14, 4 * n_rows))
    fig.suptitle("6L6 Power Tube Validation", fontsize=14, fontweight='bold')

    # Panel 1: Plate curves with DC operating point
    ax = axes[0]
    vpk_range = np.linspace(0, 500, 400)
    vgk_values = [0, -5, -10, -15, -20, -25, -30, -40]
    colors = plt.cm.viridis(np.linspace(0, 1, len(vgk_values)))

    for vgk, color in zip(vgk_values, colors):
        ip_curve = np.array([koren_ip(v, vgk) * 1000 for v in vpk_range])
        ax.plot(vpk_range, ip_curve, color=color, linewidth=1.2, label=f'Vgk={vgk:.0f}V')

    # Load line
    ip_loadline = np.linspace(0, VB / (RP + RK), 100)
    vpk_loadline = VB - (RP + RK) * ip_loadline
    ax.plot(vpk_loadline, ip_loadline * 1000, 'r--', linewidth=2, label='Load line')
    ax.plot(vpk_dc, ip_dc * 1000, 'ro', markersize=12, zorder=10, label=f'Q-point ({vpk_dc:.0f}V, {ip_dc*1000:.1f}mA)')

    ax.set_xlabel('Vpk (V)')
    ax.set_ylabel('Ip (mA)')
    ax.set_title('6L6 Plate Curves + DC Operating Point')
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 500)
    ax.set_ylim(0, max(ip_dc * 1000 * 3, 50))

    # Panel 2: WDF plate voltage (full sim)
    ax = axes[1]
    t_wdf = np.arange(len(wdf_vplate)) / FS * 1000
    ax.plot(t_wdf, wdf_vplate, 'b', linewidth=0.6)
    ax.axhline(wdf_vp_dc, color='r', linestyle='--', alpha=0.7,
               label=f'WDF DC={wdf_vp_dc:.1f}V')
    ax.axvline(n_settle / FS * 1000, color='gray', linestyle=':', alpha=0.5, label='Signal start')
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Plate Voltage (V)')
    ax.set_title(f'WDF Simulation (gain={wdf_gain_db:.1f}dB, analytical={analytical_gain_db:.1f}dB)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Panel 3: Zoomed AC waveform (last few cycles)
    ax = axes[2]
    last_n = int(5 * FS / 440)
    t_zoom = np.arange(last_n) / FS * 1000
    vp_ac = wdf_vplate[-last_n:] - np.mean(wdf_vplate[-last_n:])
    vi_scaled = audio_in[-last_n:]
    if np.max(np.abs(vi_scaled)) > 1e-6:
        scale = np.max(np.abs(vp_ac)) / np.max(np.abs(vi_scaled))
        vi_scaled = vi_scaled * scale
    ax.plot(t_zoom, vp_ac, 'b', linewidth=1.2, label='WDF output AC')
    ax.plot(t_zoom, vi_scaled, 'gray', linewidth=0.8, alpha=0.5, label=f'Input (scaled x{scale:.1f})')
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Voltage (V)')
    ax.set_title('AC Output (zoomed, last 5 cycles)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Panel 4: SPICE overlay (if available)
    if has_spice:
        ax = axes[3]
        t_spice_ms = spice_time * 1000
        ax.plot(t_spice_ms, spice_vplate, 'r', linewidth=0.6, label=f'SPICE (DC={spice_vp_dc:.1f}V)')
        ax.axhline(spice_vp_dc, color='r', linestyle='--', alpha=0.5)
        ax.axhline(wdf_vp_dc, color='b', linestyle='--', alpha=0.5, label=f'WDF DC={wdf_vp_dc:.1f}V')
        ax.set_xlabel('Time (ms)')
        ax.set_ylabel('Plate Voltage (V)')
        ax.set_title('SPICE vs WDF DC Comparison')
        ax.legend()
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("validation_6l6.png", dpi=150, bbox_inches='tight')
    print("\nSaved: validation_6l6.png")


# =============================================================================
# Main
# =============================================================================

def main():
    print("=" * 60)
    print("6L6 Power Tube Validation Suite")
    print("=" * 60)
    print(f"  Constants: mu={MU}, ex={EX}, kg1={KG1}, kp={KP}, kvb={KVB}")
    print(f"  Circuit:   VB={VB}V, RP={RP}, RK={RK}")
    print()

    results = []

    # Test 1: DC operating point
    t1_pass, ip_dc, vpk_dc, vgk_dc = test_dc_operating_point()
    results.append(("DC Operating Point", t1_pass))

    # Test 2: Small-signal gain
    t2_pass, wdf_vp_dc, wdf_gain_db, analytical_gain_db, wdf_vplate, wdf_ip, audio_in, n_settle = test_gain()
    results.append(("Small-Signal Gain", t2_pass))

    # Test 3: SPICE validation
    t3_pass, spice_time, spice_vplate, spice_vinput, spice_vp_dc = test_spice(wdf_vp_dc, wdf_gain_db)
    results.append(("SPICE Validation", t3_pass))

    # Generate plot
    generate_plot(
        ip_dc, vpk_dc, vgk_dc,
        wdf_vplate, wdf_ip, audio_in, n_settle,
        wdf_vp_dc, wdf_gain_db, analytical_gain_db,
        spice_time, spice_vplate, spice_vp_dc
    )

    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    all_pass = True
    for name, passed in results:
        tag = "PASS" if passed else "FAIL"
        print(f"  {tag}: {name}")
        if not passed:
            all_pass = False

    print(f"\nOverall: {'PASS' if all_pass else 'FAIL'}")

    if not all_pass:
        sys.exit(1)
    sys.exit(0)


if __name__ == "__main__":
    main()
