"""
test_fitted_constants.py
Tests whether curve-fitted 12AX7 Koren constants (mu=92.08, ex=1.29,
kg1=1304.71, kp=561.08, kvb=15101.91) can be used in the Verilog WDF
implementation without overflow.

The key concern: kvb=15102 is large. In the Koren equation,
  sqrt(kvb + vpk^2) with kvb=15102, vpk=300V -> sqrt(15102+90000)=324

The tube curve is pre-computed in Python LUTs (float), so kvb doesn't
appear directly in Verilog. However, the LUT values themselves may
have different ranges that cause overflow in the Q16.16 fixed-point
Newton solver, or the IP_SCALE/DIP_SCALE factors may not work for the
new value range.

This script:
1. Generates LUTs with fitted constants
2. Runs the Python WDF sim with both old and fitted constants
3. Compares the Ip/dIp ranges and checks for 16-bit overflow in LUTs
4. Identifies exactly which values overflow and proposes a fix
"""

import numpy as np
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.join(SCRIPT_DIR, "..")
DATA_DIR = os.path.join(ROOT_DIR, "data")

# Add sim dir
sys.path.insert(0, SCRIPT_DIR)

# =============================================================================
# Koren model (same as tube_lut_gen.py)
# =============================================================================

def koren_ip(Vpk, Vgk, mu, ex, kg1, kp, kvb):
    inner = kp * (1.0/mu + Vgk / np.sqrt(kvb + Vpk**2 + 1e-9))
    Ed = (Vpk / kp) * np.log1p(np.exp(np.clip(inner, -500, 500)))
    Ip = (np.maximum(Ed, 0.0) ** ex) / kg1
    return Ip

def koren_dip_dvgk(Vpk, Vgk, mu, ex, kg1, kp, kvb):
    sqrt_term = np.sqrt(kvb + Vpk**2 + 1e-9)
    inner = kp * (1.0/mu + Vgk / sqrt_term)
    inner_c = np.clip(inner, -500, 500)
    Ed = (Vpk / kp) * np.log1p(np.exp(inner_c))
    Ed = np.maximum(Ed, 0.0)
    sig = np.exp(inner_c) / (1.0 + np.exp(inner_c))
    dEd_dVgk = (Vpk / sqrt_term) * sig
    dIp = np.where(Ed > 0, (ex / kg1) * Ed**(ex - 1.0) * dEd_dVgk, 0.0)
    return dIp

def koren_dip_dvpk(Vpk, Vgk, mu, ex, kg1, kp, kvb):
    sqrt_term = np.sqrt(kvb + Vpk**2 + 1e-9)
    inner = kp * (1.0/mu + Vgk / sqrt_term)
    inner_c = np.clip(inner, -500, 500)
    Ed = (Vpk / kp) * np.log1p(np.exp(inner_c))
    Ed = np.maximum(Ed, 0.0)
    sig = np.exp(inner_c) / (1.0 + np.exp(inner_c))
    dinner_dVpk = kp * Vgk * (-Vpk) / (sqrt_term**3)
    dEd_dVpk = (1.0 / kp) * np.log1p(np.exp(inner_c)) + (Vpk / kp) * sig * dinner_dVpk
    dIp = np.where(Ed > 0, (ex / kg1) * Ed**(ex - 1.0) * dEd_dVpk, 0.0)
    return dIp

# =============================================================================
# Constants
# =============================================================================

OLD_12AX7 = dict(mu=100.0, ex=1.4, kg1=1060.0, kp=600.0, kvb=300.0)
FITTED_12AX7 = dict(mu=92.08, ex=1.29, kg1=1304.71, kp=561.08, kvb=15101.91)

# LUT parameters (from tube_lut_gen.py)
LUT_SIZE = 128
IP_SCALE = 1e4
DIP_SCALE = 1e5
DIP_VPK_SCALE = 1e7
VPK_MIN, VPK_MAX = 0.0, 300.0
VGK_MIN, VGK_MAX = -4.0, 0.0

# =============================================================================
# WDF simulation (simplified single-stage, from wdf_triode_sim_wdf.py)
# =============================================================================

def simulate_wdf_stage(audio_in, tube_params, fs=48000.0, settle=2000):
    """Run a simplified WDF triode stage with given tube parameters."""
    VB = 200.0
    RP = 100000.0
    RG = 1000000.0
    RK = 1500.0
    CIN = 22e-9
    CK = 22e-6

    R_CK = 1.0 / (2.0 * fs * CK)
    G_rk = 1.0 / RK
    G_ck = 1.0 / R_CK
    G_total = G_rk + G_ck
    R_CATH = 1.0 / G_total
    GAMMA_CATH = G_rk / G_total

    tau_hp = RG * CIN
    k_hp = 2.0 * fs * tau_hp
    c_hp = (k_hp - 1.0) / (k_hp + 1.0)
    hp_gain = (1.0 + c_hp) / 2.0

    n_total = len(audio_in)
    out_vplate = np.zeros(n_total)
    out_ip = np.zeros(n_total)

    prev_ip = 0.5e-3
    prev_ig = 0.0
    hp_y = 0.0
    hp_x_prev = 0.0
    z_ck = 0.0

    mu = tube_params['mu']
    ex = tube_params['ex']
    kg1 = tube_params['kg1']
    kp = tube_params['kp']
    kvb = tube_params['kvb']

    for n in range(n_total):
        vin = audio_in[n]
        hp_y = c_hp * hp_y + hp_gain * (vin - hp_x_prev)
        hp_x_prev = vin
        v_grid = hp_y

        b_rk = 0.0
        b_ck = z_ck
        bDiff = b_ck - b_rk
        b_cathode = b_ck - GAMMA_CATH * bDiff

        a_p = VB
        a_g = v_grid
        a_k = b_cathode

        Ip = prev_ip
        Ig = prev_ig

        for iteration in range(20):
            Vpk = (a_p - a_k) - (RP + R_CATH) * Ip - R_CATH * Ig
            Vgk = (a_g - a_k) - RG * Ig - R_CATH * (Ip + Ig)

            Vpk_c = np.clip(Vpk, 0.0, 500.0)
            Vgk_c = np.clip(Vgk, -10.0, 1.0)

            ip_model = float(koren_ip(Vpk_c, Vgk_c, mu, ex, kg1, kp, kvb))
            f1 = Ip - ip_model

            Vgk_pos = max(0.0, Vgk)
            ig_model = 0.002 * Vgk_pos**1.5 if Vgk_pos > 0 else 0.0
            f2 = Ig - ig_model

            if abs(f1) < 1e-10 and abs(f2) < 1e-10:
                break

            h = 0.01
            dip_dvpk = float((koren_ip(Vpk_c+h, Vgk_c, mu, ex, kg1, kp, kvb) -
                              koren_ip(Vpk_c-h, Vgk_c, mu, ex, kg1, kp, kvb)) / (2*h))
            dip_dvgk = float((koren_ip(Vpk_c, Vgk_c+h, mu, ex, kg1, kp, kvb) -
                              koren_ip(Vpk_c, Vgk_c-h, mu, ex, kg1, kp, kvb)) / (2*h))
            dig_dvgk = 0.003 * Vgk_pos**0.5 if Vgk_pos > 0 else 0.0

            J11 = 1.0 + dip_dvpk * (RP + R_CATH) + dip_dvgk * R_CATH
            J12 = dip_dvpk * R_CATH + dip_dvgk * (RG + R_CATH)
            J21 = dig_dvgk * R_CATH
            J22 = 1.0 + dig_dvgk * (RG + R_CATH)

            det = J11 * J22 - J12 * J21
            if abs(det) < 1e-30:
                break

            dIp = (J22 * f1 - J12 * f2) / det
            dIg = (J11 * f2 - J21 * f1) / det

            Ip = max(Ip - dIp, 0.0)
            Ig = max(Ig - dIg, 0.0)

        prev_ip = Ip
        prev_ig = Ig

        b_p = a_p - 2.0 * RP * Ip
        b_k = a_k + 2.0 * R_CATH * (Ip + Ig)
        a_ck = b_k + b_cathode - b_ck
        z_ck = a_ck

        v_plate = (a_p + b_p) / 2.0
        out_vplate[n] = v_plate
        out_ip[n] = Ip

    return out_vplate, out_ip


# =============================================================================
# LUT Analysis
# =============================================================================

def analyze_lut_range(tube_params, label):
    """Analyze LUT value ranges and check for 16-bit overflow."""
    p = tube_params
    Vpk_axis = np.linspace(VPK_MIN, VPK_MAX, LUT_SIZE)
    Vgk_axis = np.linspace(VGK_MIN, VGK_MAX, LUT_SIZE)
    Vpk_grid, Vgk_grid = np.meshgrid(Vpk_axis, Vgk_axis, indexing='ij')

    ip_grid = koren_ip(Vpk_grid, Vgk_grid, **p)
    dip_vgk_grid = koren_dip_dvgk(Vpk_grid, Vgk_grid, **p)
    dip_vpk_grid = koren_dip_dvpk(Vpk_grid, Vgk_grid, **p)

    ip_scaled = ip_grid * IP_SCALE
    dip_vgk_scaled = dip_vgk_grid * DIP_SCALE
    dip_vpk_scaled = dip_vpk_grid * DIP_VPK_SCALE

    print(f"\n  === {label} LUT Analysis ===")
    print(f"  Ip range:       {ip_grid.min()*1000:.4f} to {ip_grid.max()*1000:.4f} mA")
    print(f"  dIp/dVgk range: {dip_vgk_grid.min():.6f} to {dip_vgk_grid.max():.6f} A/V")
    print(f"  dIp/dVpk range: {dip_vpk_grid.min():.8f} to {dip_vpk_grid.max():.8f} A/V")

    print(f"\n  Scaled values (must fit in int16 [-32768, 32767]):")
    overflow_found = False

    for name, scaled in [("Ip", ip_scaled), ("dIp/dVgk", dip_vgk_scaled), ("dIp/dVpk", dip_vpk_scaled)]:
        smin, smax = scaled.min(), scaled.max()
        fits = (smin >= -32768) and (smax <= 32767)
        status = "OK" if fits else "OVERFLOW"
        if not fits:
            overflow_found = True
            n_overflow = np.sum((scaled < -32768) | (scaled > 32767))
            pct = 100.0 * n_overflow / scaled.size
        else:
            n_overflow = 0
            pct = 0.0
        print(f"    {name:>12}: [{smin:>10.2f}, {smax:>10.2f}] -> {status}"
              f"  ({n_overflow} overflow entries, {pct:.1f}%)")

    return overflow_found, {
        'ip': ip_grid, 'dip_vgk': dip_vgk_grid, 'dip_vpk': dip_vpk_grid,
        'ip_scaled': ip_scaled, 'dip_vgk_scaled': dip_vgk_scaled,
        'dip_vpk_scaled': dip_vpk_scaled,
    }


def check_q16_16_intermediates(tube_params, label):
    """Check whether Q16.16 intermediate computations overflow in the Verilog Newton solver.

    The Verilog converts LUT raw int16 values back to Q16.16:
      ip_model    = (ip_raw << 16) / IP_SCALE
      dip_vgk_val = (dip_vgk_raw << 16) / DIP_SCALE
      dip_vpk_val = (dip_vpk_raw << 16) / DIP_VPK_SCALE

    Then computes:
      J11 = 1 + dIp/dVpk * (Rp + Rk)       in Q16.16
      J12 = dIp/dVgk * Rg                   in Q16.16
      ...
      det = J11*J22 - J12*J21               64-bit intermediate
      dIp = (J22*f1 - J12*f2) / det         64-bit intermediate

    Key question: do any 64-bit intermediate products overflow?
    """
    print(f"\n  === {label} Q16.16 Intermediate Analysis ===")

    # Worst-case LUT raw values (clamped to 16-bit)
    p = tube_params
    Vpk_axis = np.linspace(VPK_MIN, VPK_MAX, LUT_SIZE)
    Vgk_axis = np.linspace(VGK_MIN, VGK_MAX, LUT_SIZE)
    Vpk_grid, Vgk_grid = np.meshgrid(Vpk_axis, Vgk_axis, indexing='ij')

    ip_grid = koren_ip(Vpk_grid, Vgk_grid, **p)
    dip_vgk_grid = koren_dip_dvgk(Vpk_grid, Vgk_grid, **p)
    dip_vpk_grid = koren_dip_dvpk(Vpk_grid, Vgk_grid, **p)

    # Clamp to 16-bit range (as the LUT generator does)
    ip_raw_max = min(ip_grid.max() * IP_SCALE, 32767)
    dip_vgk_raw_max = min(dip_vgk_grid.max() * DIP_SCALE, 32767)
    dip_vpk_raw_max = min(dip_vpk_grid.max() * DIP_VPK_SCALE, 32767)

    RP_INT = 100000
    RG_INT = 1000000
    FP_FRAC = 16

    # Conversion: ip_model = (ip_raw << 16) / IP_SCALE
    ip_model_max = (ip_raw_max * 65536) / IP_SCALE
    dip_vgk_model_max = (dip_vgk_raw_max * 65536) / DIP_SCALE
    dip_vpk_model_max = (dip_vpk_raw_max * 65536) / DIP_VPK_SCALE

    print(f"  Max ip_model (Q16.16):    {ip_model_max:.0f}  ({ip_model_max/65536:.6f} A)")
    print(f"  Max dip_vgk (Q16.16):     {dip_vgk_model_max:.0f}  ({dip_vgk_model_max/65536:.6f} A/V)")
    print(f"  Max dip_vpk (Q16.16):     {dip_vpk_model_max:.0f}  ({dip_vpk_model_max/65536:.8f} A/V)")

    # J11 = ONE_FP + (dip_vpk_raw * RPK_INT << FP_FRAC) / DIP_VPK_SCALE
    # The intermediate: dip_vpk_raw * RPK_INT << 16
    j11_intermediate = dip_vpk_raw_max * RP_INT * 65536
    print(f"\n  J11 intermediate (dip_vpk_raw * RP * 2^16): {j11_intermediate:.0e}")
    print(f"  Fits in int64? {abs(j11_intermediate) < 2**63}")

    # J12 = (dip_vgk_raw * RG_INT << FP_FRAC) / DIP_SCALE
    j12_intermediate = dip_vgk_raw_max * RG_INT * 65536
    print(f"  J12 intermediate (dip_vgk_raw * RG * 2^16): {j12_intermediate:.0e}")
    print(f"  Fits in int64? {abs(j12_intermediate) < 2**63}")

    # f_val max: ip_est - ip_model, worst case ~ip_model max
    f_val_max = ip_model_max  # Q16.16

    # det = J11 * J22 >> FP_FRAC, J11 and J22 are Q16.16
    # Worst case J11, J22 ~ 65536 (=1.0) + some correction
    # det product: J11*J22 in Q32.32 -> shift to Q16.16
    # This is fine as long as J11, J22 fit in 32 bits

    # dIp_num = (J22 * f1 - J12 * f2) >> FP_FRAC
    # Then: step = (dIp_num << FP_FRAC) / det
    # The left shift: dIp_num << 16. If dIp_num is ~32 bits, this overflows 64 bits
    dip_num_worst = 2**31 * 65536  # worst case: full 32-bit * 65536
    print(f"\n  Worst-case dIp_num << 16: {dip_num_worst:.0e}")
    print(f"  Fits in int64? {abs(dip_num_worst) < 2**63}")

    overflow_risk = abs(j11_intermediate) >= 2**63 or abs(j12_intermediate) >= 2**63
    return overflow_risk


# =============================================================================
# Main
# =============================================================================

def main():
    print("=" * 60)
    print("Testing Fitted 12AX7 Constants in Q16.16 Fixed Point")
    print("=" * 60)

    print(f"\nOld constants:    mu={OLD_12AX7['mu']}, ex={OLD_12AX7['ex']}, "
          f"kg1={OLD_12AX7['kg1']}, kp={OLD_12AX7['kp']}, kvb={OLD_12AX7['kvb']}")
    print(f"Fitted constants: mu={FITTED_12AX7['mu']}, ex={FITTED_12AX7['ex']}, "
          f"kg1={FITTED_12AX7['kg1']}, kp={FITTED_12AX7['kp']}, kvb={FITTED_12AX7['kvb']}")

    # --- Task 1: Analyze LUT ranges ---
    print("\n" + "=" * 60)
    print("Task 1: LUT Range Analysis")
    print("=" * 60)

    old_overflow, old_data = analyze_lut_range(OLD_12AX7, "Old 12AX7")
    fit_overflow, fit_data = analyze_lut_range(FITTED_12AX7, "Fitted 12AX7")

    # --- Task 2: Check Q16.16 intermediates ---
    print("\n" + "=" * 60)
    print("Task 2: Q16.16 Intermediate Overflow Check")
    print("=" * 60)

    old_q16_risk = check_q16_16_intermediates(OLD_12AX7, "Old 12AX7")
    fit_q16_risk = check_q16_16_intermediates(FITTED_12AX7, "Fitted 12AX7")

    # --- Task 3: Run Python WDF sim with both ---
    print("\n" + "=" * 60)
    print("Task 3: Python WDF Simulation Comparison")
    print("=" * 60)

    fs = 48000.0
    duration = 0.1  # 100ms enough to see gain
    settle = 2000
    n_audio = int(duration * fs)
    n_total = settle + n_audio
    t = np.arange(n_total) / fs

    # Test signal: 440Hz sine, 0.5V peak (typical guitar level)
    audio_in = np.zeros(n_total)
    audio_in[settle:] = 0.5 * np.sin(2 * np.pi * 440 * t[settle:])

    print("\n  Running WDF with old constants...")
    vp_old, ip_old = simulate_wdf_stage(audio_in, OLD_12AX7, fs=fs, settle=settle)

    print("  Running WDF with fitted constants...")
    vp_fit, ip_fit = simulate_wdf_stage(audio_in, FITTED_12AX7, fs=fs, settle=settle)

    # Analyze results
    audio_slice = slice(settle, n_total)

    vp_old_dc = np.mean(vp_old[audio_slice])
    vp_fit_dc = np.mean(vp_fit[audio_slice])
    vp_old_ac = vp_old[audio_slice] - vp_old_dc
    vp_fit_ac = vp_fit[audio_slice] - vp_fit_dc

    old_rms = np.sqrt(np.mean(vp_old_ac**2))
    fit_rms = np.sqrt(np.mean(vp_fit_ac**2))
    in_rms = np.sqrt(np.mean(audio_in[audio_slice]**2))

    old_gain_db = 20 * np.log10(old_rms / (in_rms + 1e-12))
    fit_gain_db = 20 * np.log10(fit_rms / (in_rms + 1e-12))

    ip_old_dc = np.mean(ip_old[audio_slice])
    ip_fit_dc = np.mean(ip_fit[audio_slice])

    print(f"\n  Old constants:")
    print(f"    Vplate DC:  {vp_old_dc:.2f} V")
    print(f"    Ip DC:      {ip_old_dc*1000:.3f} mA")
    print(f"    AC gain:    {old_gain_db:.1f} dB")
    print(f"    Output RMS: {old_rms:.3f} V")

    print(f"\n  Fitted constants:")
    print(f"    Vplate DC:  {vp_fit_dc:.2f} V")
    print(f"    Ip DC:      {ip_fit_dc*1000:.3f} mA")
    print(f"    AC gain:    {fit_gain_db:.1f} dB")
    print(f"    Output RMS: {fit_rms:.3f} V")

    # Check for NaN/Inf (Newton divergence)
    old_has_nan = np.any(np.isnan(vp_old)) or np.any(np.isinf(vp_old))
    fit_has_nan = np.any(np.isnan(vp_fit)) or np.any(np.isinf(vp_fit))

    print(f"\n  Old NaN/Inf: {old_has_nan}")
    print(f"  Fitted NaN/Inf: {fit_has_nan}")

    # Check if Ip stayed reasonable
    ip_fit_max = np.max(ip_fit[audio_slice])
    ip_fit_min = np.min(ip_fit[audio_slice])
    print(f"\n  Fitted Ip range: {ip_fit_min*1000:.4f} to {ip_fit_max*1000:.4f} mA")

    # --- Task 4: Propose fix if needed ---
    print("\n" + "=" * 60)
    print("Task 4: Diagnosis and Recommendations")
    print("=" * 60)

    issues = []

    if fit_overflow:
        issues.append("LUT values overflow int16 with current scale factors")
        # Find which values overflow
        for name, scaled in [("Ip", fit_data['ip_scaled']),
                              ("dIp/dVgk", fit_data['dip_vgk_scaled']),
                              ("dIp/dVpk", fit_data['dip_vpk_scaled'])]:
            n_over = np.sum((scaled < -32768) | (scaled > 32767))
            if n_over > 0:
                max_val = np.max(np.abs(scaled))
                needed_scale = 32767 / (max_val / {
                    "Ip": IP_SCALE, "dIp/dVgk": DIP_SCALE, "dIp/dVpk": DIP_VPK_SCALE
                }[name])
                current_scale = {"Ip": IP_SCALE, "dIp/dVgk": DIP_SCALE, "dIp/dVpk": DIP_VPK_SCALE}[name]
                safe_scale = int(32767 / (max_val / current_scale))
                print(f"\n  OVERFLOW in {name}: max |scaled| = {max_val:.1f}, "
                      f"needs scale <= {safe_scale}")
                issues.append(f"  Fix: reduce {name} scale from {int(current_scale)} to {safe_scale}")

    if fit_q16_risk:
        issues.append("Q16.16 intermediate calculations risk 64-bit overflow")

    if fit_has_nan:
        issues.append("Python WDF Newton solver diverges with fitted constants")

    if not issues:
        print("\n  No overflow issues found!")
        print("  The fitted constants work correctly with current scale factors.")
        print(f"  LUT values fit in int16, Q16.16 intermediates fit in int64.")
        print(f"\n  The kvb=15102 value only affects Python LUT generation (float).")
        print(f"  The Verilog runtime reads pre-computed hex files and never")
        print(f"  computes sqrt(kvb + vpk^2) directly.")
    else:
        print(f"\n  Found {len(issues)} issue(s):")
        for issue in issues:
            print(f"    - {issue}")

        # Compute recommended scale factors
        print("\n  Recommended fix:")
        print("  Update IP_SCALE, DIP_SCALE, DIP_VPK_SCALE in tube_lut_gen.py")
        print("  and match them in wdf_triode_wdf.v parameters.")

        # Try to find safe scales
        ip_max = fit_data['ip'].max()
        dip_vgk_max = fit_data['dip_vgk'].max()
        dip_vpk_max = fit_data['dip_vpk'].max()

        safe_ip_scale = int(32000 / ip_max) if ip_max > 0 else int(IP_SCALE)
        safe_dip_scale = int(32000 / dip_vgk_max) if dip_vgk_max > 0 else int(DIP_SCALE)
        safe_dip_vpk_scale = int(32000 / dip_vpk_max) if dip_vpk_max > 0 else int(DIP_VPK_SCALE)

        print(f"\n  Safe scale factors for fitted 12AX7:")
        print(f"    IP_SCALE:      {safe_ip_scale} (was {int(IP_SCALE)})")
        print(f"    DIP_SCALE:     {safe_dip_scale} (was {int(DIP_SCALE)})")
        print(f"    DIP_VPK_SCALE: {safe_dip_vpk_scale} (was {int(DIP_VPK_SCALE)})")

    # --- Overall result ---
    print("\n" + "=" * 60)
    all_pass = (not fit_overflow) and (not fit_q16_risk) and (not fit_has_nan)
    if all_pass:
        print("Overall: PASS -- Fitted constants are safe for Verilog use")
    else:
        print("Overall: ISSUES FOUND -- See recommendations above")
    print("=" * 60)

    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
