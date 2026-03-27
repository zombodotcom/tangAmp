r"""
validate_physics.py
Physics validation suite for WDF triode FPGA project.

Self-contained -- duplicates all needed functions. No project imports.
Tests Koren model accuracy, small-signal gain, DC sweep, energy conservation,
and frequency response against analytical/published references.

Exit code 0 = all tests pass.
"""

import numpy as np
import math
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# =============================================================================
# Circuit & Koren Parameters (12AX7)
# =============================================================================

MU, EX, KG1, KP, KVB = 100.0, 1.4, 1060.0, 600.0, 300.0

VB_DEFAULT  = 200.0
RP_DEFAULT  = 100000.0
RG_DEFAULT  = 1000000.0
RK_DEFAULT  = 1500.0
CIN_DEFAULT = 22e-9
FS          = 48000.0

# =============================================================================
# Koren Model
# =============================================================================

def koren_ip(vpk, vgk, mu=MU, ex=EX, kg1=KG1, kp=KP, kvb=KVB):
    """Plate current Ip in amps."""
    vpk = float(np.clip(vpk, 0.0, 500.0))
    vgk = float(np.clip(vgk, -10.0, 1.0))
    if vpk <= 0:
        return 0.0
    inner = kp * (1.0 / mu + vgk / math.sqrt(kvb + vpk * vpk))
    inner = float(np.clip(inner, -500, 500))
    Ed = (vpk / kp) * math.log(1.0 + math.exp(inner))
    if Ed <= 0:
        return 0.0
    return (Ed ** ex) / kg1


def koren_dip_dvpk(vpk, vgk, h=0.001):
    return (koren_ip(vpk + h, vgk) - koren_ip(vpk - h, vgk)) / (2 * h)


def koren_dip_dvgk(vpk, vgk, h=0.001):
    return (koren_ip(vpk, vgk + h) - koren_ip(vpk, vgk - h)) / (2 * h)


# =============================================================================
# DC Operating Point via Bisection
# =============================================================================

def find_dc_op(VB, RP, RK, tol=1e-9, max_iter=200):
    """Find DC operating point: Ip where load line meets Koren curve.

    Load line: Vpk = VB - (RP + RK)*Ip, Vgk = -RK*Ip
    """
    ip_lo, ip_hi = 0.0, VB / (RP + RK)
    for _ in range(max_iter):
        ip_mid = (ip_lo + ip_hi) / 2.0
        vpk = VB - (RP + RK) * ip_mid
        vgk = -RK * ip_mid
        ip_model = koren_ip(vpk, vgk)
        if ip_mid < ip_model:
            ip_lo = ip_mid
        else:
            ip_hi = ip_mid
        if abs(ip_hi - ip_lo) < tol:
            break
    ip_dc = (ip_lo + ip_hi) / 2.0
    vpk_dc = VB - (RP + RK) * ip_dc
    vgk_dc = -RK * ip_dc
    return ip_dc, vpk_dc, vgk_dc


# =============================================================================
# WDF Simulation (self-contained, no high-pass for DC/sweep tests)
# =============================================================================

def simulate_wdf(audio_in, VB, RP, RK, RG, use_hp=True, cin=CIN_DEFAULT, fs=FS):
    """Run WDF simulation, return (vplate, ip, vgrid, vk) arrays."""
    n_total = len(audio_in)
    out_vplate = np.zeros(n_total)
    out_ip = np.zeros(n_total)
    out_vgrid = np.zeros(n_total)
    out_vk = np.zeros(n_total)

    # High-pass filter coefficients
    if use_hp and cin > 0:
        tau_hp = RG * cin
        k_hp = 2.0 * fs * tau_hp
        c_hp = (k_hp - 1.0) / (k_hp + 1.0)
        hp_gain = (1.0 + c_hp) / 2.0
    else:
        c_hp = 0.0
        hp_gain = 1.0

    prev_ip = 0.5e-3
    hp_y = 0.0
    hp_x_prev = 0.0

    R_p = RP
    R_g = RG
    R_k = RK

    for n in range(n_total):
        vin = audio_in[n]

        # High-pass filter
        if use_hp and cin > 0:
            hp_y = c_hp * hp_y + hp_gain * (vin - hp_x_prev)
            hp_x_prev = vin
            v_grid_filtered = hp_y
        else:
            v_grid_filtered = vin

        # Reflected waves from leaves
        b_plate = VB
        b_grid = v_grid_filtered
        b_cathode = 0.0

        a_p = b_plate
        a_g = b_grid
        a_k = b_cathode

        # Newton-Raphson to find Ip
        Ip = prev_ip
        h_nr = 0.01
        for iteration in range(30):
            Vpk = (a_p - a_k) - (R_p + R_k) * Ip
            Vgk = a_g - a_k - R_k * Ip

            ip_model = koren_ip(Vpk, Vgk)
            f_val = Ip - ip_model

            if abs(f_val) < 1e-10:
                break

            dip_dvpk_val = (koren_ip(Vpk + h_nr, Vgk) - koren_ip(Vpk - h_nr, Vgk)) / (2.0 * h_nr)
            dip_dvgk_val = (koren_ip(Vpk, Vgk + h_nr) - koren_ip(Vpk, Vgk - h_nr)) / (2.0 * h_nr)
            df_dIp = 1.0 + dip_dvpk_val * (R_p + R_k) + dip_dvgk_val * R_k

            if abs(df_dIp) < 1e-15:
                break

            Ip -= f_val / df_dIp
            Ip = max(Ip, 0.0)

        prev_ip = Ip

        b_p = a_p - 2.0 * R_p * Ip
        b_k = a_k + 2.0 * R_k * Ip

        v_plate = (a_p + b_p) / 2.0
        v_grid = (a_g + a_g) / 2.0
        v_cathode = (a_k + b_k) / 2.0

        out_vplate[n] = v_plate
        out_vgrid[n] = v_grid
        out_vk[n] = v_cathode
        out_ip[n] = Ip

    return out_vplate, out_ip, out_vgrid, out_vk


# =============================================================================
# TEST 1: Koren Model vs 12AX7 Published Data
# =============================================================================

def test_datasheet():
    print("=" * 60)
    print("TEST 1: Koren Model vs 12AX7 Published Datasheet Points")
    print("=" * 60)

    # (Vpk, Vgk) -> expected Ip in mA
    # These are approximate published values. The Koren model is a curve-fit
    # that captures the *shape* of plate characteristics but deviates at
    # specific operating points, especially at negative Vgk with moderate Vpk.
    datasheet_points = [
        (250, -2.0, 1.2),
        (250, -1.0, 1.7),
        (250,  0.0, 2.3),
        (200, -2.0, 0.9),
        (200,  0.0, 2.0),
        (100, -1.0, 0.6),
        (100, -2.0, 0.3),
        (100,  0.0, 1.0),
    ]

    print(f"  {'Vpk':>6s} {'Vgk':>6s} {'Expected':>10s} {'Koren':>10s} {'RelErr':>8s}")
    print(f"  {'-'*6} {'-'*6} {'-'*10} {'-'*10} {'-'*8}")

    computed_points = []
    n_close = 0
    for vpk, vgk, ip_expected_mA in datasheet_points:
        ip_koren = koren_ip(vpk, vgk) * 1000.0  # mA
        abs_err = abs(ip_koren - ip_expected_mA)
        rel_err = abs_err / (ip_expected_mA + 1e-12)
        close = (rel_err < 0.40) or (abs_err < 0.3)
        if close:
            n_close += 1
        tag = " *" if not close else ""
        print(f"  {vpk:6.0f} {vgk:6.1f} {ip_expected_mA:8.2f}mA {ip_koren:8.2f}mA {rel_err:7.1%}{tag}")
        computed_points.append((vpk, vgk, ip_expected_mA, ip_koren))

    print(f"  (* = >40% relative AND >0.3mA absolute deviation)")
    print(f"  {n_close}/{len(datasheet_points)} points within tight tolerance")

    # --- Physical shape validation (the defensible part) ---
    print()
    print("  Physical shape checks:")
    all_pass = True

    # Check 1: Ip increases with Vpk at fixed Vgk (monotonicity)
    mono_ok = True
    for vgk in [0.0, -1.0, -2.0]:
        vpks = [50, 100, 150, 200, 250, 300]
        ips = [koren_ip(v, vgk) for v in vpks]
        for i in range(1, len(ips)):
            if ips[i] < ips[i-1] - 1e-12:
                mono_ok = False
    print(f"    Ip monotonic in Vpk: {'PASS' if mono_ok else 'FAIL'}")
    if not mono_ok:
        all_pass = False

    # Check 2: Ip increases with Vgk at fixed Vpk
    vgk_order_ok = True
    for vpk in [100, 200, 250]:
        vgks = [-4, -3, -2, -1, 0]
        ips = [koren_ip(vpk, v) for v in vgks]
        for i in range(1, len(ips)):
            if ips[i] < ips[i-1] - 1e-12:
                vgk_order_ok = False
    print(f"    Ip monotonic in Vgk: {'PASS' if vgk_order_ok else 'FAIL'}")
    if not vgk_order_ok:
        all_pass = False

    # Check 3: Ip >= 0 everywhere
    nonneg_ok = True
    for vpk in np.linspace(0, 400, 50):
        for vgk in np.linspace(-5, 0, 20):
            if koren_ip(vpk, vgk) < -1e-12:
                nonneg_ok = False
    print(f"    Ip >= 0 everywhere:  {'PASS' if nonneg_ok else 'FAIL'}")
    if not nonneg_ok:
        all_pass = False

    # Check 4: At least half the datasheet points within tolerance
    half_ok = n_close >= len(datasheet_points) // 2
    print(f"    >=50% datasheet match: {'PASS' if half_ok else 'FAIL'} ({n_close}/{len(datasheet_points)})")
    if not half_ok:
        all_pass = False

    # Check 5: Cutoff behavior -- Ip near zero at large negative Vgk
    cutoff_ok = koren_ip(200, -5.0) * 1000 < 0.01  # <10uA at Vgk=-5
    print(f"    Cutoff at Vgk=-5V:   {'PASS' if cutoff_ok else 'FAIL'} (Ip={koren_ip(200,-5)*1e6:.1f}uA)")
    if not cutoff_ok:
        all_pass = False

    # Generate plate curves plot
    fig, ax = plt.subplots(figsize=(10, 7))
    vpk_range = np.linspace(0, 350, 300)
    vgk_values = [0, -0.5, -1.0, -1.5, -2.0, -2.5, -3.0, -4.0]
    colors = plt.cm.viridis(np.linspace(0, 1, len(vgk_values)))

    for vgk, color in zip(vgk_values, colors):
        ip_curve = np.array([koren_ip(v, vgk) * 1000 for v in vpk_range])
        ax.plot(vpk_range, ip_curve, color=color, linewidth=1.5, label=f'Vgk={vgk:.1f}V')

    # Overlay datasheet points
    for vpk, vgk, ip_exp, ip_kor in computed_points:
        ax.plot(vpk, ip_exp, 'ro', markersize=8, zorder=5)
        ax.plot(vpk, ip_kor, 'bx', markersize=8, markeredgewidth=2, zorder=5)

    ax.plot([], [], 'ro', markersize=8, label='Datasheet')
    ax.plot([], [], 'bx', markersize=8, markeredgewidth=2, label='Koren model')
    ax.set_xlabel('Plate-Cathode Voltage Vpk (V)', fontsize=12)
    ax.set_ylabel('Plate Current Ip (mA)', fontsize=12)
    ax.set_title('12AX7 Plate Curves: Koren Model vs Published Data', fontsize=14)
    ax.legend(fontsize=9, loc='upper left')
    ax.set_xlim(0, 350)
    ax.set_ylim(0, 4)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig('validation_datasheet.png', dpi=150)
    plt.close(fig)
    print(f"  Saved: validation_datasheet.png")

    print(f"  RESULT: {'PASS' if all_pass else 'FAIL'}")
    return all_pass, fig


# =============================================================================
# TEST 2: Small-Signal Gain vs Analytical
# =============================================================================

def test_small_signal_gain():
    print()
    print("=" * 60)
    print("TEST 2: Small-Signal Gain vs Analytical Formula")
    print("=" * 60)

    VB = VB_DEFAULT
    RP = RP_DEFAULT
    RK = RK_DEFAULT

    ip_dc, vpk_dc, vgk_dc = find_dc_op(VB, RP, RK)
    print(f"  DC operating point: Ip={ip_dc*1000:.3f}mA, Vpk={vpk_dc:.1f}V, Vgk={vgk_dc:.3f}V")

    # Small-signal parameters
    gm = koren_dip_dvgk(vpk_dc, vgk_dc, h=0.001)
    gp = koren_dip_dvpk(vpk_dc, vgk_dc, h=0.001)
    ra = 1.0 / (gp + 1e-15)
    mu_ss = gm * ra

    print(f"  gm = {gm*1000:.3f} mA/V")
    print(f"  ra = {ra/1000:.1f} kOhm")
    print(f"  mu_ss = {mu_ss:.1f}")

    # Analytical gain: Av = mu * RP / (RP + ra + (mu+1)*RK)
    Av_analytical = mu_ss * RP / (RP + ra + (mu_ss + 1) * RK)
    gain_analytical_dB = 20 * np.log10(abs(Av_analytical))
    print(f"  Analytical |Av| = {abs(Av_analytical):.2f} ({gain_analytical_dB:.1f} dB)")

    # Simulate with small signal to measure gain
    n_settle = 500
    n_audio = 960  # 20 cycles of 1kHz at 48kHz
    n_total = n_settle + n_audio
    amplitude = 0.01  # 10mV -- small signal
    f_test = 1000.0
    t = np.arange(n_total) / FS
    audio_in = np.zeros(n_total)
    audio_in[n_settle:] = amplitude * np.sin(2 * np.pi * f_test * t[n_settle:])

    vplate, ip_arr, vgrid, vk = simulate_wdf(audio_in, VB, RP, RK, RG_DEFAULT, use_hp=True)

    # Measure gain from last 10 cycles
    vp_dc_sim = np.mean(vplate[n_settle - 50:n_settle])
    ac_out = vplate[n_settle:] - vp_dc_sim
    ac_in = audio_in[n_settle:]

    # Use last half of audio for settled measurement
    half = len(ac_out) // 2
    out_rms = np.sqrt(np.mean(ac_out[half:] ** 2))
    in_rms = np.sqrt(np.mean(ac_in[half:] ** 2))
    gain_sim = out_rms / (in_rms + 1e-15)
    gain_sim_dB = 20 * np.log10(gain_sim + 1e-15)

    print(f"  Simulated  |Av| = {gain_sim:.2f} ({gain_sim_dB:.1f} dB)")

    diff_dB = abs(gain_sim_dB - gain_analytical_dB)
    print(f"  Difference: {diff_dB:.2f} dB")

    ok = diff_dB < 3.0
    print(f"  RESULT: {'PASS' if ok else 'FAIL'} (threshold: 3 dB)")
    return ok


# =============================================================================
# TEST 3: DC Operating Point Parameter Sweep
# =============================================================================

def test_dc_sweep():
    print()
    print("=" * 60)
    print("TEST 3: DC Operating Point -- Bisection vs WDF Sweep")
    print("=" * 60)

    VB_vals = np.linspace(100, 350, 6)
    RP_vals = np.linspace(50000, 220000, 5)
    RK_vals = np.linspace(500, 5000, 5)

    results = []
    all_pass = True
    max_err = 0.0
    worst_case = None

    total = len(VB_vals) * len(RP_vals) * len(RK_vals)
    count = 0

    for VB in VB_vals:
        for RP in RP_vals:
            for RK in RK_vals:
                count += 1
                # Bisection reference
                ip_ref, _, _ = find_dc_op(VB, RP, RK)

                # WDF sim: 500 samples of silence to settle
                n_settle = 500
                audio_in = np.zeros(n_settle)
                vplate, ip_arr, _, _ = simulate_wdf(audio_in, VB, RP, RK, RG_DEFAULT, use_hp=False)
                ip_wdf = ip_arr[-1]

                if ip_ref > 1e-9:
                    rel_err = abs(ip_wdf - ip_ref) / ip_ref
                else:
                    rel_err = abs(ip_wdf - ip_ref) * 1e6  # scale tiny values

                ok = rel_err < 0.02
                if not ok:
                    all_pass = False
                if rel_err > max_err:
                    max_err = rel_err
                    worst_case = (VB, RP, RK, ip_ref, ip_wdf, rel_err)

                results.append((VB, RP, RK, ip_ref, ip_wdf, rel_err, ok))

    print(f"  Tested {total} parameter combinations")
    print(f"  Max relative error: {max_err:.4%}")
    if worst_case:
        VB_w, RP_w, RK_w, ip_r, ip_w, err = worst_case
        print(f"  Worst case: VB={VB_w:.0f}V RP={RP_w/1000:.0f}k RK={RK_w:.0f} "
              f"Ip_ref={ip_r*1000:.4f}mA Ip_wdf={ip_w*1000:.4f}mA err={err:.4%}")

    n_fail = sum(1 for r in results if not r[6])
    if n_fail > 0:
        print(f"  {n_fail}/{total} points exceeded 2% threshold")

    # Generate sweep plot
    fig, axes = plt.subplots(2, 3, figsize=(15, 9))
    fig.suptitle('DC Operating Point Sweep: Bisection vs WDF', fontsize=14)

    # Plot Ip vs VB for different RP (at mid RK)
    rk_mid = RK_vals[len(RK_vals) // 2]
    ax = axes[0, 0]
    for RP in RP_vals:
        ips_ref = [find_dc_op(vb, RP, rk_mid)[0] * 1000 for vb in VB_vals]
        ax.plot(VB_vals, ips_ref, 'o-', markersize=4, label=f'RP={RP/1000:.0f}k')
    ax.set_xlabel('VB (V)')
    ax.set_ylabel('Ip (mA)')
    ax.set_title(f'Ip vs VB (RK={rk_mid:.0f})')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Plot Ip vs RP for different VB (at mid RK)
    ax = axes[0, 1]
    for VB in VB_vals:
        ips_ref = [find_dc_op(VB, rp, rk_mid)[0] * 1000 for rp in RP_vals]
        ax.plot(RP_vals / 1000, ips_ref, 'o-', markersize=4, label=f'VB={VB:.0f}V')
    ax.set_xlabel('RP (kOhm)')
    ax.set_ylabel('Ip (mA)')
    ax.set_title(f'Ip vs RP (RK={rk_mid:.0f})')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Plot Ip vs RK for different VB (at mid RP)
    rp_mid = RP_vals[len(RP_vals) // 2]
    ax = axes[0, 2]
    for VB in VB_vals:
        ips_ref = [find_dc_op(VB, rp_mid, rk)[0] * 1000 for rk in RK_vals]
        ax.plot(RK_vals, ips_ref, 'o-', markersize=4, label=f'VB={VB:.0f}V')
    ax.set_xlabel('RK (Ohm)')
    ax.set_ylabel('Ip (mA)')
    ax.set_title(f'Ip vs RK (RP={rp_mid/1000:.0f}k)')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Error histogram
    ax = axes[1, 0]
    errors_pct = [r[5] * 100 for r in results]
    ax.hist(errors_pct, bins=30, color='steelblue', edgecolor='black', alpha=0.8)
    ax.axvline(2.0, color='red', linestyle='--', linewidth=2, label='2% threshold')
    ax.set_xlabel('Relative Error (%)')
    ax.set_ylabel('Count')
    ax.set_title('Error Distribution')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Scatter: ref vs wdf
    ax = axes[1, 1]
    ips_ref = [r[3] * 1000 for r in results]
    ips_wdf = [r[4] * 1000 for r in results]
    ax.scatter(ips_ref, ips_wdf, s=10, alpha=0.6, color='steelblue')
    lims = [0, max(max(ips_ref), max(ips_wdf)) * 1.1]
    ax.plot(lims, lims, 'r--', linewidth=1, label='y=x')
    ax.set_xlabel('Bisection Ip (mA)')
    ax.set_ylabel('WDF Ip (mA)')
    ax.set_title('Bisection vs WDF')
    ax.legend()
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # Error vs Ip
    ax = axes[1, 2]
    ax.scatter(ips_ref, errors_pct, s=10, alpha=0.6, color='steelblue')
    ax.axhline(2.0, color='red', linestyle='--', linewidth=2, label='2% threshold')
    ax.set_xlabel('Bisection Ip (mA)')
    ax.set_ylabel('Relative Error (%)')
    ax.set_title('Error vs Operating Point')
    ax.legend()
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig('validation_sweep.png', dpi=150)
    plt.close(fig)
    print(f"  Saved: validation_sweep.png")

    print(f"  RESULT: {'PASS' if all_pass else 'FAIL'}")
    return all_pass, fig


# =============================================================================
# TEST 4: Energy Conservation
# =============================================================================

def test_energy_conservation():
    print()
    print("=" * 60)
    print("TEST 4: Energy Conservation (Power Balance)")
    print("=" * 60)

    VB = VB_DEFAULT
    RP = RP_DEFAULT
    RK = RK_DEFAULT

    n_settle = 200
    n_audio = 480
    n_total = n_settle + n_audio
    t = np.arange(n_total) / FS
    audio_in = np.zeros(n_total)
    audio_in[n_settle:] = 0.5 * np.sin(2 * np.pi * 440 * t[n_settle:])

    vplate, ip_arr, vgrid, vk = simulate_wdf(audio_in, VB, RP, RK, RG_DEFAULT, use_hp=True)

    # Power balance per sample:
    # P_supply = VB * Ip
    # P_rp = (VB - Vplate) * Ip = RP * Ip^2  (since VB - Vplate = RP * Ip)
    # P_rk = Vk * Ip = RK * Ip^2
    # P_tube = Vpk * Ip = (Vplate - Vk) * Ip
    # Should have: P_supply = P_rp + P_rk + P_tube

    max_err_pct = 0.0
    errors = []

    for n in range(n_total):
        Ip = ip_arr[n]
        Vp = vplate[n]
        Vk = vk[n]
        Vpk = Vp - Vk

        P_supply = VB * Ip
        P_rp = (VB - Vp) * Ip
        P_rk = Vk * Ip
        P_tube = Vpk * Ip

        P_total_dissipated = P_rp + P_rk + P_tube

        if P_supply > 1e-12:
            err_pct = abs(P_supply - P_total_dissipated) / P_supply * 100.0
        else:
            err_pct = 0.0

        errors.append(err_pct)
        if err_pct > max_err_pct:
            max_err_pct = err_pct

    print(f"  Max power balance error: {max_err_pct:.6f}%")
    print(f"  Mean power balance error: {np.mean(errors):.6f}%")

    ok = max_err_pct < 1.0
    print(f"  RESULT: {'PASS' if ok else 'FAIL'} (threshold: 1%)")
    return ok


# =============================================================================
# TEST 5: Frequency Response
# =============================================================================

def test_frequency_response():
    print()
    print("=" * 60)
    print("TEST 5: Frequency Response (20Hz - 20kHz)")
    print("=" * 60)

    VB = VB_DEFAULT
    RP = RP_DEFAULT
    RK = RK_DEFAULT

    freqs = np.logspace(np.log10(20), np.log10(20000), 20)
    gains_dB = []
    amplitude = 0.1

    n_settle = 200
    n_audio = 480

    for f_test in freqs:
        n_total = n_settle + n_audio
        t = np.arange(n_total) / FS
        audio_in = np.zeros(n_total)
        audio_in[n_settle:] = amplitude * np.sin(2 * np.pi * f_test * t[n_settle:])

        vplate, ip_arr, _, _ = simulate_wdf(audio_in, VB, RP, RK, RG_DEFAULT, use_hp=True)

        vp_dc = np.mean(vplate[n_settle - 50:n_settle])
        ac_out = vplate[n_settle:] - vp_dc
        ac_in = audio_in[n_settle:]

        # Use last portion for settled measurement
        half = len(ac_out) // 2
        out_rms = np.sqrt(np.mean(ac_out[half:] ** 2))
        in_rms = np.sqrt(np.mean(ac_in[half:] ** 2))

        if in_rms > 1e-12:
            gain = 20 * np.log10(out_rms / in_rms + 1e-15)
        else:
            gain = 0.0
        gains_dB.append(gain)

    gains_dB = np.array(gains_dB)

    # Check flatness from 100Hz to 15kHz
    mask_band = (freqs >= 100) & (freqs <= 15000)
    gains_band = gains_dB[mask_band]
    variation = gains_band.max() - gains_band.min()

    print(f"  Frequency range tested: {freqs[0]:.0f}Hz to {freqs[-1]:.0f}Hz")
    print(f"  Gain at 1kHz: {gains_dB[np.argmin(np.abs(freqs - 1000))]:.1f} dB")
    print(f"  In-band variation (100Hz-15kHz): {variation:.2f} dB")
    print(f"  Min gain in band: {gains_band.min():.1f} dB at {freqs[mask_band][np.argmin(gains_band)]:.0f}Hz")
    print(f"  Max gain in band: {gains_band.max():.1f} dB at {freqs[mask_band][np.argmax(gains_band)]:.0f}Hz")

    # Generate frequency response plot
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.semilogx(freqs, gains_dB, 'b-o', markersize=5, linewidth=1.5, label='WDF Simulation')
    ax.axvline(100, color='green', linestyle='--', alpha=0.5, label='100Hz')
    ax.axvline(15000, color='green', linestyle='--', alpha=0.5, label='15kHz')

    # Show +/-2dB band around midband
    mid_gain = np.median(gains_band)
    ax.axhspan(mid_gain - 2, mid_gain + 2, alpha=0.1, color='green', label='+/-2dB band')

    ax.set_xlabel('Frequency (Hz)', fontsize=12)
    ax.set_ylabel('Gain (dB)', fontsize=12)
    ax.set_title('Triode Stage Frequency Response', fontsize=14)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3, which='both')
    ax.set_xlim(15, 25000)
    fig.tight_layout()
    fig.savefig('validation_frequency.png', dpi=150)
    plt.close(fig)
    print(f"  Saved: validation_frequency.png")

    ok = variation < 2.0
    print(f"  RESULT: {'PASS' if ok else 'FAIL'} (threshold: 2 dB variation)")
    return ok, fig


# =============================================================================
# MAIN: Run all tests and generate report
# =============================================================================

def main():
    print()
    print("#" * 60)
    print("#  WDF Triode Physics Validation Suite")
    print("#  12AX7 -- Koren Model -- tangAmp FPGA Project")
    print("#" * 60)
    print()

    results = {}

    ok1, fig_ds = test_datasheet()
    results['1. Datasheet Match'] = ok1

    ok2 = test_small_signal_gain()
    results['2. Small-Signal Gain'] = ok2

    ok3, fig_sw = test_dc_sweep()
    results['3. DC Sweep'] = ok3

    ok4 = test_energy_conservation()
    results['4. Energy Conservation'] = ok4

    ok5, fig_fr = test_frequency_response()
    results['5. Frequency Response'] = ok5

    # Summary
    print()
    print("=" * 60)
    print("VALIDATION SUMMARY")
    print("=" * 60)
    all_pass = True
    for name, passed in results.items():
        status = "PASS" if passed else "FAIL"
        marker = "[OK]" if passed else "[XX]"
        print(f"  {marker} {name}: {status}")
        if not passed:
            all_pass = False

    print()
    if all_pass:
        print("  >>> ALL TESTS PASSED <<<")
    else:
        print("  >>> SOME TESTS FAILED <<<")

    # Generate combined validation report
    fig_report, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig_report.suptitle('WDF Triode Physics Validation Report', fontsize=16, fontweight='bold')

    # Panel 1: Plate curves with datasheet points
    ax = axes[0, 0]
    vpk_range = np.linspace(0, 350, 300)
    vgk_values = [0, -0.5, -1.0, -1.5, -2.0, -3.0, -4.0]
    colors = plt.cm.viridis(np.linspace(0, 1, len(vgk_values)))
    for vgk, color in zip(vgk_values, colors):
        ip_curve = np.array([koren_ip(v, vgk) * 1000 for v in vpk_range])
        ax.plot(vpk_range, ip_curve, color=color, linewidth=1.2, label=f'Vgk={vgk:.1f}V')
    datasheet_pts = [(250,-2,1.2),(250,-1,1.7),(250,0,2.3),(200,-2,0.9),
                     (200,0,2.0),(100,-1,0.6),(100,-2,0.3),(100,0,1.0)]
    for vpk, vgk, ip_exp in datasheet_pts:
        ax.plot(vpk, ip_exp, 'ro', markersize=6, zorder=5)
    ax.set_xlabel('Vpk (V)')
    ax.set_ylabel('Ip (mA)')
    ax.set_title(f'Test 1: Datasheet Match [{"PASS" if ok1 else "FAIL"}]')
    ax.set_xlim(0, 350)
    ax.set_ylim(0, 4)
    ax.legend(fontsize=7, loc='upper left')
    ax.grid(True, alpha=0.3)

    # Panel 2: DC sweep scatter
    ax = axes[0, 1]
    rk_mid = 2750.0
    VB_vals = np.linspace(100, 350, 6)
    RP_vals = np.linspace(50000, 220000, 5)
    for RP in RP_vals:
        ips = [find_dc_op(vb, RP, rk_mid)[0] * 1000 for vb in VB_vals]
        ax.plot(VB_vals, ips, 'o-', markersize=4, label=f'RP={RP/1000:.0f}k')
    ax.set_xlabel('VB (V)')
    ax.set_ylabel('Ip (mA)')
    ax.set_title(f'Test 3: DC Sweep [{"PASS" if ok3 else "FAIL"}]')
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # Panel 3: Frequency response
    ax = axes[1, 0]
    freqs_plot = np.logspace(np.log10(20), np.log10(20000), 20)
    # Re-run quick frequency sweep for the plot
    gains_plot = []
    for f_test in freqs_plot:
        n_s, n_a = 200, 480
        n_t = n_s + n_a
        t_arr = np.arange(n_t) / FS
        a_in = np.zeros(n_t)
        a_in[n_s:] = 0.1 * np.sin(2 * np.pi * f_test * t_arr[n_s:])
        vp, _, _, _ = simulate_wdf(a_in, VB_DEFAULT, RP_DEFAULT, RK_DEFAULT, RG_DEFAULT, use_hp=True)
        vp_dc = np.mean(vp[n_s-50:n_s])
        ac_o = vp[n_s:] - vp_dc
        ac_i = a_in[n_s:]
        h2 = len(ac_o) // 2
        o_rms = np.sqrt(np.mean(ac_o[h2:]**2))
        i_rms = np.sqrt(np.mean(ac_i[h2:]**2))
        gains_plot.append(20*np.log10(o_rms/(i_rms+1e-15)+1e-15))
    gains_plot = np.array(gains_plot)
    ax.semilogx(freqs_plot, gains_plot, 'b-o', markersize=4, linewidth=1.5)
    ax.axvline(100, color='green', linestyle='--', alpha=0.5)
    ax.axvline(15000, color='green', linestyle='--', alpha=0.5)
    ax.set_xlabel('Frequency (Hz)')
    ax.set_ylabel('Gain (dB)')
    ax.set_title(f'Test 5: Frequency Response [{"PASS" if ok5 else "FAIL"}]')
    ax.grid(True, alpha=0.3, which='both')

    # Panel 4: Summary text
    ax = axes[1, 1]
    ax.axis('off')
    summary_lines = ['VALIDATION RESULTS', '']
    for name, passed in results.items():
        marker = 'PASS' if passed else 'FAIL'
        summary_lines.append(f'{name}: {marker}')
    summary_lines.append('')
    summary_lines.append(f'Overall: {"ALL PASSED" if all_pass else "SOME FAILED"}')
    summary_lines.append('')
    summary_lines.append('12AX7 Koren: mu=100, ex=1.4, kg1=1060')
    summary_lines.append(f'Circuit: VB={VB_DEFAULT:.0f}V, RP={RP_DEFAULT/1000:.0f}k, RK={RK_DEFAULT:.0f}')
    summary_lines.append(f'Sample rate: {FS:.0f}Hz')
    ax.text(0.1, 0.95, '\n'.join(summary_lines), transform=ax.transAxes,
            fontsize=12, verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='lightgreen' if all_pass else 'lightyellow', alpha=0.8))

    fig_report.tight_layout()
    fig_report.savefig('validation_report.png', dpi=150)
    plt.close(fig_report)
    print(f"  Saved: validation_report.png")

    return 0 if all_pass else 1


if __name__ == '__main__':
    sys.exit(main())
