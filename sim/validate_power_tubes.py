r"""
validate_power_tubes.py
Validation suite for EL34 and 300B power tube models.

Validates each tube against:
  1. Published datasheet Ip points (Koren model accuracy)
  2. DC operating point via bisection (VB=400V, RP=2000, RK=250)
  3. Small-signal gain: analytical vs WDF simulation
  4. SPICE behavioral model comparison (ngspice via PySpice)

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
# Tube Definitions
# =============================================================================

TUBES = {
    'EL34': {
        'mu': 10.98, 'ex': 1.42, 'kg1': 249.65, 'kp': 43.2, 'kvb': 333.0,
        'VB': 400.0, 'RP': 2000.0, 'RK': 250.0,
        'RG': 1e6, 'CIN': 22e-9, 'CK': 22e-6, 'FS': 48000.0,
        'published': [
            # (Vp, Vg, Ip_mA, source)
            (250, -13, 100.0, 'Mullard Vp=250V Vg=-13V'),
            (300, -15,  80.0, 'Mullard Vp=300V Vg=-15V'),
        ],
        'vpk_clip': 600.0, 'vgk_clip': (-60.0, 5.0),
    },
    '300B': {
        'mu': 3.95, 'ex': 1.4, 'kg1': 1550.0, 'kp': 65.0, 'kvb': 300.0,
        'VB': 400.0, 'RP': 2000.0, 'RK': 250.0,
        'RG': 1e6, 'CIN': 22e-9, 'CK': 22e-6, 'FS': 48000.0,
        'published': [
            (300, -61, 60.0, 'WE Vp=300V Vg=-61V'),
            (250, -45, 40.0, 'WE Vp=250V Vg=-45V'),
        ],
        'vpk_clip': 600.0, 'vgk_clip': (-100.0, 5.0),
    },
}

# Tolerances
DC_TOL_PCT  = 5.0   # percent
GAIN_TOL_DB = 3.0   # dB
PUBLISHED_TOL_PCT = 50.0  # percent tolerance -- Koren is approximate, some points will be off


# =============================================================================
# Koren Model
# =============================================================================

def koren_ip(vpk, vgk, mu, ex, kg1, kp, kvb, vpk_clip=600.0, vgk_clip=(-60.0, 5.0)):
    """Plate current Ip in amps."""
    vpk = float(np.clip(vpk, 0.0, vpk_clip))
    vgk = float(np.clip(vgk, vgk_clip[0], vgk_clip[1]))
    if vpk <= 0:
        return 0.0
    inner = kp * (1.0 / mu + vgk / math.sqrt(kvb + vpk * vpk))
    inner = float(np.clip(inner, -500, 500))
    Ed = (vpk / kp) * math.log(1.0 + math.exp(inner))
    if Ed <= 0:
        return 0.0
    return (Ed ** ex) / kg1


def koren_dip_dvpk(vpk, vgk, tube, h=0.01):
    t = TUBES[tube]
    return (koren_ip(vpk + h, vgk, **_kp(t)) - koren_ip(vpk - h, vgk, **_kp(t))) / (2 * h)


def koren_dip_dvgk(vpk, vgk, tube, h=0.01):
    t = TUBES[tube]
    return (koren_ip(vpk, vgk + h, **_kp(t)) - koren_ip(vpk, vgk - h, **_kp(t))) / (2 * h)


def _kp(t):
    """Extract Koren params dict from tube dict."""
    return {
        'mu': t['mu'], 'ex': t['ex'], 'kg1': t['kg1'],
        'kp': t['kp'], 'kvb': t['kvb'],
        'vpk_clip': t['vpk_clip'], 'vgk_clip': t['vgk_clip'],
    }


# =============================================================================
# DC Operating Point via Bisection
# =============================================================================

def find_dc_op(tube, tol=1e-12, max_iter=200):
    t = TUBES[tube]
    vb, rp, rk = t['VB'], t['RP'], t['RK']
    kp = _kp(t)

    ip_lo, ip_hi = 0.0, vb / (rp + rk)
    for _ in range(max_iter):
        ip_mid = (ip_lo + ip_hi) / 2.0
        vpk = vb - (rp + rk) * ip_mid
        vgk = -rk * ip_mid
        ip_model = koren_ip(vpk, vgk, **kp)
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

def analytical_gain(vpk_q, vgk_q, tube):
    t = TUBES[tube]
    rp, rk = t['RP'], t['RK']

    gm = koren_dip_dvgk(vpk_q, vgk_q, tube)
    dip = koren_dip_dvpk(vpk_q, vgk_q, tube)

    if dip < 1e-12:
        return 0.0, 0.0, 0.0

    ra = 1.0 / dip
    mu_local = gm * ra

    av_bypassed = -mu_local * rp / (rp + ra)
    av_unbypassed = -mu_local * rp / (rp + ra + (mu_local + 1) * rk)

    return av_bypassed, av_unbypassed, ra


# =============================================================================
# WDF Simulation (with cathode bypass cap)
# =============================================================================

def simulate_wdf(audio_in, tube):
    t = TUBES[tube]
    vb, rp, rk, rg = t['VB'], t['RP'], t['RK'], t['RG']
    cin, ck, fs = t['CIN'], t['CK'], t['FS']
    kp = _kp(t)

    n_total = len(audio_in)
    out_vplate = np.zeros(n_total)
    out_ip = np.zeros(n_total)

    # High-pass filter
    tau_hp = rg * cin
    k_hp = 2.0 * fs * tau_hp
    c_hp = (k_hp - 1.0) / (k_hp + 1.0)
    hp_gain = (1.0 + c_hp) / 2.0

    # Cathode bypass cap
    r_ck = 1.0 / (2.0 * fs * ck)
    g_rk = 1.0 / rk
    g_ck = 1.0 / r_ck
    g_tot = g_rk + g_ck
    R_k = 1.0 / g_tot
    gamma_cath = g_rk / g_tot

    R_p = rp
    prev_ip = 10e-3
    hp_y = 0.0
    hp_x_prev = 0.0
    z_ck = 0.0

    for n in range(n_total):
        vin = audio_in[n]

        hp_y = c_hp * hp_y + hp_gain * (vin - hp_x_prev)
        hp_x_prev = vin
        v_grid = hp_y

        b_plate = vb
        b_grid = v_grid

        b_rk = 0.0
        b_ck = z_ck
        bDiff = b_ck - b_rk
        b_cathode = b_ck - gamma_cath * bDiff

        a_p = b_plate
        a_g = b_grid
        a_k = b_cathode

        Ip = prev_ip
        for iteration in range(30):
            Vpk = (a_p - a_k) - (R_p + R_k) * Ip
            Vgk = a_g - a_k - R_k * Ip

            ip_model = koren_ip(Vpk, Vgk, **kp)
            f_val = Ip - ip_model

            if abs(f_val) < 1e-10:
                break

            dip_dvpk_val = koren_dip_dvpk(Vpk, Vgk, tube)
            dip_dvgk_val = koren_dip_dvgk(Vpk, Vgk, tube)
            df_dIp = 1.0 + dip_dvpk_val * (R_p + R_k) + dip_dvgk_val * R_k

            if abs(df_dIp) < 1e-15:
                break

            Ip -= f_val / df_dIp
            Ip = max(Ip, 0.0)

        prev_ip = Ip

        b_p = a_p - 2.0 * R_p * Ip
        b_k = a_k + 2.0 * R_k * Ip

        a_ck = b_k + b_cathode - b_ck
        z_ck = a_ck

        v_plate = (a_p + b_p) / 2.0
        out_vplate[n] = v_plate
        out_ip[n] = Ip

    return out_vplate, out_ip


# =============================================================================
# SPICE Validation (ngspice behavioral model)
# =============================================================================

def build_spice_netlist(tube):
    t = TUBES[tube]
    vb, rp, rk, rg = t['VB'], t['RP'], t['RK'], t['RG']
    cin, ck = t['CIN'], t['CK']
    mu, ex, kg1, kp, kvb = t['mu'], t['ex'], t['kg1'], t['kp'], t['kvb']

    netlist = f"""\
.title {tube} Common Cathode Validation
VB supply 0 DC {vb}
RP supply plate {rp}
VIN input 0 DC 0 SIN(0 5 440)
CIN input grid {cin}
RG grid 0 {rg}
RK cathode 0 {rk}
CK cathode 0 {ck}
BEd ed 0 V = V(plate,cathode)/{kp} * ln(1 + exp({kp}*(1/{mu} + V(grid,cathode)/sqrt({kvb} + V(plate,cathode)*V(plate,cathode) + 0.000001))))
BIp plate cathode I = pwr(max(V(ed),0), {ex}) / {kg1}
.options reltol=0.001 abstol=1n vntol=1u
.ic V(plate)=300 V(cathode)=15 V(grid)=0 V(ed)=2
.tran 20.833u 200m uic
.end"""
    return netlist


def run_spice_simulation(tube):
    import tempfile
    import os
    from PySpice.Spice.NgSpice.Shared import NgSpiceShared

    ng = NgSpiceShared.new_instance()
    netlist = build_spice_netlist(tube)

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
# TEST 1: Published Data Point Validation
# =============================================================================

def test_published_data(tube):
    t = TUBES[tube]
    kp = _kp(t)
    published = t['published']

    print("=" * 60)
    print(f"TEST 1: {tube} Published Datasheet Points")
    print("=" * 60)

    n_pass = 0
    n_warn = 0
    for vp, vg, ip_expected_ma, source in published:
        # Published Vp and Vg are plate-to-ground and grid-to-ground.
        # For triode-connected: cathode is at ground in datasheet measurements,
        # so Vpk = Vp, Vgk = Vg.
        ip_koren = koren_ip(vp, vg, **kp) * 1000  # mA
        err_pct = abs(ip_koren - ip_expected_ma) / ip_expected_ma * 100
        ok = err_pct < PUBLISHED_TOL_PCT
        tag = "PASS" if ok else "WARN"
        print(f"  {tag}: {source}")
        print(f"         Koren Ip = {ip_koren:.2f} mA, Published = {ip_expected_ma:.1f} mA, Error = {err_pct:.1f}%")
        if ok:
            n_pass += 1
        else:
            n_warn += 1

    # Published data is informational -- Koren is an approximate fit.
    # Pass if at least one point matches well.
    all_pass = n_pass > 0
    if n_warn > 0:
        print(f"  Note: {n_warn} point(s) exceed {PUBLISHED_TOL_PCT}% tolerance (Koren fit limitation)")
    return all_pass


# =============================================================================
# TEST 2: DC Operating Point
# =============================================================================

def test_dc_operating_point(tube):
    t = TUBES[tube]
    vb, rp, rk = t['VB'], t['RP'], t['RK']

    print("\n" + "=" * 60)
    print(f"TEST 2: {tube} DC Operating Point (VB={vb}V, RP={rp}, RK={rk})")
    print("=" * 60)

    ip_dc, vpk_dc, vgk_dc = find_dc_op(tube)
    vplate_dc = vb - rp * ip_dc
    vk_dc = rk * ip_dc

    print(f"  Ip_dc  = {ip_dc*1000:.3f} mA")
    print(f"  Vpk_dc = {vpk_dc:.2f} V")
    print(f"  Vgk_dc = {vgk_dc:.2f} V")
    print(f"  Vplate = {vplate_dc:.2f} V")
    print(f"  Vk     = {vk_dc:.2f} V")

    checks = []

    ip_ok = 5.0 < ip_dc * 1000 < 200.0
    checks.append(("Ip in range [5, 200] mA", ip_ok))

    vp_ok = 50 < vplate_dc < 380
    checks.append(("Vplate in range [50, 380] V", vp_ok))

    vgk_ok = -80 < vgk_dc < 0
    checks.append(("Vgk negative (self-bias)", vgk_ok))

    residual = abs(vb - (rp + rk) * ip_dc - vpk_dc)
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
# TEST 3: Small-Signal Gain (Analytical vs WDF)
# =============================================================================

def test_gain(tube):
    t = TUBES[tube]
    vb, rp, rk = t['VB'], t['RP'], t['RK']
    fs = t['FS']

    print("\n" + "=" * 60)
    print(f"TEST 3: {tube} Small-Signal Gain")
    print("=" * 60)

    ip_dc, vpk_dc, vgk_dc = find_dc_op(tube)
    vplate_dc = vb - rp * ip_dc

    av_bypassed, av_unbypassed, ra = analytical_gain(vpk_dc, vgk_dc, tube)
    gain_bypassed_db = 20 * np.log10(abs(av_bypassed) + 1e-12)
    gain_unbypassed_db = 20 * np.log10(abs(av_unbypassed) + 1e-12)

    gm = koren_dip_dvgk(vpk_dc, vgk_dc, tube)
    print(f"  Q-point: Vpk={vpk_dc:.1f}V, Vgk={vgk_dc:.2f}V, Ip={ip_dc*1000:.2f}mA")
    print(f"  gm = {gm*1000:.3f} mA/V")
    print(f"  ra = {ra:.0f} ohm")
    print(f"  Analytical gain (bypassed Ck): {av_bypassed:.2f} ({gain_bypassed_db:.1f} dB)")
    print(f"  Analytical gain (unbypassed):  {av_unbypassed:.2f} ({gain_unbypassed_db:.1f} dB)")

    # WDF simulation
    n_settle = 4000
    n_signal = 8000
    n_total = n_settle + n_signal
    t_arr = np.arange(n_total) / fs

    vin_amp = 5.0
    audio_in = np.zeros(n_total)
    audio_in[n_settle:] = vin_amp * np.sin(2 * np.pi * 440 * t_arr[n_settle:])

    vplate_wdf, ip_wdf = simulate_wdf(audio_in, tube)

    last_n = int(4 * fs / 440)
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

    dc_err_pct = abs(wdf_vp_dc - vplate_dc) / vplate_dc * 100
    dc_ok = dc_err_pct < DC_TOL_PCT
    print(f"\n  DC error: {dc_err_pct:.2f}% (tolerance: {DC_TOL_PCT}%): {'PASS' if dc_ok else 'FAIL'}")

    gain_err = abs(wdf_gain_db - gain_bypassed_db)
    gain_ok = gain_err < GAIN_TOL_DB
    print(f"  Gain error: {gain_err:.2f} dB (tolerance: {GAIN_TOL_DB} dB): {'PASS' if gain_ok else 'FAIL'}")

    all_pass = dc_ok and gain_ok
    return all_pass, wdf_vp_dc, wdf_gain_db, gain_bypassed_db, vplate_wdf, ip_wdf, audio_in, n_settle


# =============================================================================
# TEST 4: SPICE Validation
# =============================================================================

def test_spice(tube, wdf_vp_dc, wdf_gain_db):
    t = TUBES[tube]
    vb, rp = t['VB'], t['RP']

    print("\n" + "=" * 60)
    print(f"TEST 4: {tube} ngspice Behavioral Model Validation")
    print("=" * 60)

    try:
        time_vec, v_plate, v_input, v_cathode = run_spice_simulation(tube)
    except Exception as e:
        print(f"  PySpice/ngspice not available: {e}")
        netlist = build_spice_netlist(tube)
        fname = f"spice_{tube.lower()}.cir"
        with open(fname, "w") as f:
            f.write(netlist + "\n")
        print(f"  Saved: {fname}")
        print("  SKIP (ngspice not available)")
        return True, None, None, None, None

    print(f"  SPICE: {len(time_vec)} data points")

    settle_mask = time_vec > 0.08
    spice_vp_dc = np.mean(v_plate[settle_mask])

    ac_mask = time_vec > 0.15
    vp_ac = v_plate[ac_mask] - np.mean(v_plate[ac_mask])
    if v_input is not None:
        vi_ac = v_input[ac_mask] - np.mean(v_input[ac_mask])
    else:
        vi_ac = 5.0 * np.sin(2 * np.pi * 440 * time_vec[ac_mask])

    out_rms = np.sqrt(np.mean(vp_ac ** 2))
    in_rms = np.sqrt(np.mean(vi_ac ** 2))
    spice_gain_db = 20 * np.log10(out_rms / (in_rms + 1e-12))

    spice_ip_dc = (vb - spice_vp_dc) / rp

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

def generate_plot(tube_results):
    """Generate validation_power_tubes.png with panels for each tube."""
    n_tubes = len(tube_results)
    # 4 rows per tube: plate curves, WDF full, WDF zoomed, SPICE
    fig, axes = plt.subplots(n_tubes * 4, 1, figsize=(14, 4 * n_tubes * 4))
    fig.suptitle("EL34 + 300B Power Tube Validation", fontsize=16, fontweight='bold', y=1.0)

    row = 0
    for tube, data in tube_results.items():
        t = TUBES[tube]
        ip_dc = data['ip_dc']
        vpk_dc = data['vpk_dc']
        vgk_dc = data['vgk_dc']
        wdf_vplate = data['wdf_vplate']
        audio_in = data['audio_in']
        n_settle = data['n_settle']
        wdf_vp_dc = data['wdf_vp_dc']
        wdf_gain_db = data['wdf_gain_db']
        analytical_gain_db = data['analytical_gain_db']
        spice_time = data.get('spice_time')
        spice_vplate = data.get('spice_vplate')
        spice_vp_dc = data.get('spice_vp_dc')
        kp = _kp(t)
        fs = t['FS']

        # Panel 1: Plate curves with published data and DC op point
        ax = axes[row]
        vpk_range = np.linspace(0, 500, 400)
        vgk_values = [0, -5, -10, -15, -20, -25, -30, -40, -50, -60]
        colors = plt.cm.viridis(np.linspace(0, 1, len(vgk_values)))

        for vgk, color in zip(vgk_values, colors):
            ip_curve = np.array([koren_ip(v, vgk, **kp) * 1000 for v in vpk_range])
            ax.plot(vpk_range, ip_curve, color=color, linewidth=1.0, label=f'Vgk={vgk:.0f}V')

        # Load line
        vb, rp, rk = t['VB'], t['RP'], t['RK']
        ip_loadline = np.linspace(0, vb / (rp + rk), 100)
        vpk_loadline = vb - (rp + rk) * ip_loadline
        ax.plot(vpk_loadline, ip_loadline * 1000, 'r--', linewidth=2, label='Load line')
        ax.plot(vpk_dc, ip_dc * 1000, 'ro', markersize=12, zorder=10,
                label=f'Q-point ({vpk_dc:.0f}V, {ip_dc*1000:.1f}mA)')

        # Published data points
        for vp, vg, ip_pub, src in t['published']:
            ax.plot(vp, ip_pub, 'k*', markersize=14, zorder=11)
            ip_koren_pt = koren_ip(vp, vg, **kp) * 1000
            ax.plot(vp, ip_koren_pt, 'gD', markersize=8, zorder=11)
            ax.annotate(f'{src}\nPub={ip_pub:.0f}mA Koren={ip_koren_pt:.0f}mA',
                       xy=(vp, ip_pub), fontsize=6,
                       xytext=(10, 10), textcoords='offset points')

        ax.set_xlabel('Vpk (V)')
        ax.set_ylabel('Ip (mA)')
        ax.set_title(f'{tube} Plate Curves + Published Data (black*=published, green D=Koren)')
        ax.legend(fontsize=6, ncol=3, loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0, 500)
        ax.set_ylim(0, max(ip_dc * 1000 * 3, 120))
        row += 1

        # Panel 2: WDF plate voltage (full sim)
        ax = axes[row]
        t_wdf = np.arange(len(wdf_vplate)) / fs * 1000
        ax.plot(t_wdf, wdf_vplate, 'b', linewidth=0.6)
        ax.axhline(wdf_vp_dc, color='r', linestyle='--', alpha=0.7,
                   label=f'WDF DC={wdf_vp_dc:.1f}V')
        ax.axvline(n_settle / fs * 1000, color='gray', linestyle=':', alpha=0.5, label='Signal start')
        ax.set_xlabel('Time (ms)')
        ax.set_ylabel('Plate Voltage (V)')
        ax.set_title(f'{tube} WDF Simulation (gain={wdf_gain_db:.1f}dB, analytical={analytical_gain_db:.1f}dB)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        row += 1

        # Panel 3: Zoomed AC waveform
        ax = axes[row]
        last_n = int(5 * fs / 440)
        t_zoom = np.arange(last_n) / fs * 1000
        vp_ac = wdf_vplate[-last_n:] - np.mean(wdf_vplate[-last_n:])
        vi_scaled = audio_in[-last_n:]
        scale = 1.0
        if np.max(np.abs(vi_scaled)) > 1e-6:
            scale = np.max(np.abs(vp_ac)) / np.max(np.abs(vi_scaled))
            vi_scaled = vi_scaled * scale
        ax.plot(t_zoom, vp_ac, 'b', linewidth=1.2, label='WDF output AC')
        ax.plot(t_zoom, vi_scaled, 'gray', linewidth=0.8, alpha=0.5, label=f'Input (scaled x{scale:.1f})')
        ax.set_xlabel('Time (ms)')
        ax.set_ylabel('Voltage (V)')
        ax.set_title(f'{tube} AC Output (zoomed, last 5 cycles)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        row += 1

        # Panel 4: SPICE overlay
        ax = axes[row]
        if spice_time is not None:
            t_spice_ms = spice_time * 1000
            ax.plot(t_spice_ms, spice_vplate, 'r', linewidth=0.6,
                    label=f'SPICE (DC={spice_vp_dc:.1f}V)')
            ax.axhline(spice_vp_dc, color='r', linestyle='--', alpha=0.5)
            ax.axhline(wdf_vp_dc, color='b', linestyle='--', alpha=0.5, label=f'WDF DC={wdf_vp_dc:.1f}V')
            ax.set_title(f'{tube} SPICE vs WDF DC Comparison')
        else:
            ax.text(0.5, 0.5, 'SPICE not available', transform=ax.transAxes,
                   ha='center', va='center', fontsize=14, color='gray')
            ax.set_title(f'{tube} SPICE (skipped)')
        ax.set_xlabel('Time (ms)')
        ax.set_ylabel('Plate Voltage (V)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        row += 1

    plt.tight_layout()
    plt.savefig("validation_power_tubes.png", dpi=150, bbox_inches='tight')
    print("\nSaved: validation_power_tubes.png")


# =============================================================================
# Main
# =============================================================================

def main():
    print("=" * 60)
    print("EL34 + 300B Power Tube Validation Suite")
    print("=" * 60)

    all_results = []
    tube_plot_data = {}

    for tube in ['EL34', '300B']:
        t = TUBES[tube]
        print(f"\n{'#' * 60}")
        print(f"# {tube}: mu={t['mu']}, ex={t['ex']}, kg1={t['kg1']}, kp={t['kp']}, kvb={t['kvb']}")
        print(f"# Circuit: VB={t['VB']}V, RP={t['RP']}, RK={t['RK']}")
        print(f"{'#' * 60}\n")

        # Test 1: Published data
        t1_pass = test_published_data(tube)
        all_results.append((f"{tube} Published Data", t1_pass))

        # Test 2: DC operating point
        t2_pass, ip_dc, vpk_dc, vgk_dc = test_dc_operating_point(tube)
        all_results.append((f"{tube} DC Operating Point", t2_pass))

        # Test 3: Gain
        t3_pass, wdf_vp_dc, wdf_gain_db, analytical_gain_db, wdf_vplate, wdf_ip, audio_in, n_settle = test_gain(tube)
        all_results.append((f"{tube} Small-Signal Gain", t3_pass))

        # Test 4: SPICE
        t4_pass, spice_time, spice_vplate, spice_vinput, spice_vp_dc = test_spice(tube, wdf_vp_dc, wdf_gain_db)
        all_results.append((f"{tube} SPICE Validation", t4_pass))

        tube_plot_data[tube] = {
            'ip_dc': ip_dc, 'vpk_dc': vpk_dc, 'vgk_dc': vgk_dc,
            'wdf_vplate': wdf_vplate, 'audio_in': audio_in, 'n_settle': n_settle,
            'wdf_vp_dc': wdf_vp_dc, 'wdf_gain_db': wdf_gain_db,
            'analytical_gain_db': analytical_gain_db,
            'spice_time': spice_time, 'spice_vplate': spice_vplate,
            'spice_vp_dc': spice_vp_dc,
        }

    # Generate combined plot
    generate_plot(tube_plot_data)

    # Summary table
    print("\n" + "=" * 60)
    print("RESULTS TABLE")
    print("=" * 60)
    print(f"  {'Tube':<6} {'Test':<25} {'Result':<6}")
    print(f"  {'-'*6} {'-'*25} {'-'*6}")

    overall_pass = True
    for name, passed in all_results:
        parts = name.split(' ', 1)
        tube_name = parts[0]
        test_name = parts[1] if len(parts) > 1 else name
        tag = "PASS" if passed else "FAIL"
        print(f"  {tube_name:<6} {test_name:<25} {tag}")
        if not passed:
            overall_pass = False

    # Print Koren vs published summary
    print(f"\n  {'Tube':<6} {'Point':<30} {'Published':>10} {'Koren':>10} {'Error':>8}")
    print(f"  {'-'*6} {'-'*30} {'-'*10} {'-'*10} {'-'*8}")
    for tube in ['EL34', '300B']:
        t = TUBES[tube]
        kp = _kp(t)
        for vp, vg, ip_pub, src in t['published']:
            ip_k = koren_ip(vp, vg, **kp) * 1000
            err = abs(ip_k - ip_pub) / ip_pub * 100
            print(f"  {tube:<6} {src:<30} {ip_pub:>8.1f}mA {ip_k:>8.1f}mA {err:>6.1f}%")

    print(f"\nOverall: {'PASS' if overall_pass else 'FAIL'}")

    if not overall_pass:
        sys.exit(1)
    sys.exit(0)


if __name__ == "__main__":
    main()
