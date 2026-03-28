"""
quick_test.py -- Fast smoke test (~1s). Python-only, no Verilog sim.
Tests that the WDF triode produces correct gain and operating point.
"""

import numpy as np
import time
import sys
import math

# ============================================================================
# Circuit Parameters (must match wdf_triode_sim_wdf.py)
# ============================================================================

VB  = 200.0
RP  = 100000.0
RG  = 1000000.0
RK  = 1500.0
CIN = 22e-9
CK  = 22e-6
FS  = 48000.0

# Cathode bypass cap WDF parameters
R_CK = 1.0 / (2.0 * FS * CK)
G_rk = 1.0 / RK
G_ck = 1.0 / R_CK
G_total = G_rk + G_ck
R_CATH = 1.0 / G_total
GAMMA_CATH = G_rk / G_total

# Koren 12AX7
MU, EX, KG1, KP, KVB = 100.0, 1.4, 1060.0, 600.0, 300.0

# Coupling cap high-pass
tau_hp = RG * CIN
k_hp = 2.0 * FS * tau_hp
c_hp = (k_hp - 1.0) / (k_hp + 1.0)
hp_gain = (1.0 + c_hp) / 2.0


def koren_ip(vpk, vgk):
    """Plate current Ip in amps."""
    vpk = max(0.0, min(vpk, 500.0))
    vgk = max(-10.0, min(vgk, 1.0))
    if vpk <= 0:
        return 0.0
    inner = KP * (1.0 / MU + vgk / math.sqrt(KVB + vpk * vpk))
    inner = max(-500.0, min(inner, 500.0))
    Ed = (vpk / KP) * math.log(1.0 + math.exp(inner))
    if Ed <= 0:
        return 0.0
    return (Ed ** EX) / KG1


def simulate_wdf(n_total, audio_in):
    """Run WDF triode sim with cathode bypass cap, return (vplate, ip) arrays."""
    out_vplate = np.zeros(n_total)
    out_ip = np.zeros(n_total)

    prev_ip = 0.5e-3
    hp_y = 0.0
    hp_x_prev = 0.0
    h_nr = 0.01
    z_ck = 0.0  # capacitor state for Ck

    R_p = RP
    R_k = R_CATH

    for n in range(n_total):
        vin = audio_in[n]

        # High-pass filter
        hp_y = c_hp * hp_y + hp_gain * (vin - hp_x_prev)
        hp_x_prev = vin
        v_grid_filtered = hp_y

        # Cathode: parallel adaptor Rk || Ck (upward pass)
        b_rk = 0.0
        b_ck = z_ck
        bDiff = b_ck - b_rk
        b_cathode = b_ck - GAMMA_CATH * bDiff

        a_p = VB
        a_g = v_grid_filtered
        a_k = b_cathode

        # Newton-Raphson
        Ip = prev_ip
        for _ in range(20):
            Vpk = (a_p - a_k) - (R_p + R_k) * Ip
            Vgk = a_g - a_k - R_k * Ip

            ip_model = koren_ip(Vpk, Vgk)
            f_val = Ip - ip_model

            if abs(f_val) < 1e-10:
                break

            dip_dvpk = (koren_ip(Vpk + h_nr, Vgk) - koren_ip(Vpk - h_nr, Vgk)) / (2.0 * h_nr)
            dip_dvgk = (koren_ip(Vpk, Vgk + h_nr) - koren_ip(Vpk, Vgk - h_nr)) / (2.0 * h_nr)
            df_dIp = 1.0 + dip_dvpk * (R_p + R_k) + dip_dvgk * R_k

            if abs(df_dIp) < 1e-15:
                break

            Ip -= f_val / df_dIp
            Ip = max(Ip, 0.0)

        prev_ip = Ip

        b_p = a_p - 2.0 * R_p * Ip
        b_k = a_k + 2.0 * R_k * Ip
        v_plate = (a_p + b_p) / 2.0

        # Downward pass: update capacitor state
        a_ck = b_k + b_cathode - b_ck
        z_ck = a_ck

        out_vplate[n] = v_plate
        out_ip[n] = Ip

    return out_vplate, out_ip


def main():
    t0 = time.perf_counter()
    all_ok = True

    # Parameters (2000 settle samples needed for bypass cap to charge)
    n_settle = 2000
    n_audio = 480
    n_total = n_settle + n_audio

    t = np.arange(n_total) / FS
    audio_in = np.zeros(n_total)
    audio_in[n_settle:] = 0.5 * np.sin(2 * np.pi * 440 * t[n_settle:])

    # Run simulation
    vplate, ip_out = simulate_wdf(n_total, audio_in)

    # DC operating point (last 50 samples of settling)
    dc_slice = slice(n_settle - 50, n_settle)
    vp_dc = float(vplate[dc_slice].mean())
    ip_dc = float(ip_out[dc_slice].mean())

    t_dc = time.perf_counter() - t0
    dc_ok = (140.0 <= vp_dc <= 155.0) and (0.4e-3 <= ip_dc <= 0.7e-3)
    status = "OK" if dc_ok else "FAIL"
    if not dc_ok:
        all_ok = False
    print(f"[{t_dc:.1f}s] DC: Vplate={vp_dc:.1f}V Ip={ip_dc*1000:.3f}mA  {status}")

    # AC gain
    audio_out = vplate[n_settle:] - vp_dc
    inp = audio_in[n_settle:]
    in_rms = float(np.sqrt(np.mean(inp**2)))
    out_rms = float(np.sqrt(np.mean(audio_out**2)))
    gain_db = 20 * np.log10(out_rms / (in_rms + 1e-12)) if in_rms > 0 else 0
    out_min = float(audio_out.min())
    out_max = float(audio_out.max())

    t_ac = time.perf_counter() - t0
    ac_ok = (30.0 <= gain_db <= 38.0)
    range_ok = (15.0 <= max(abs(out_min), abs(out_max)) <= 35.0)
    status = "OK" if (ac_ok and range_ok) else "FAIL"
    if not (ac_ok and range_ok):
        all_ok = False
    print(f"[{t_ac:.1f}s] AC: Gain={gain_db:.1f}dB Range=[{out_min:.1f}, {out_max:.1f}]V  {status}")

    total = time.perf_counter() - t0
    if all_ok:
        print(f"PASS ({total:.1f}s total)")
        return 0
    else:
        print(f"FAIL ({total:.1f}s total)")
        return 1


if __name__ == "__main__":
    sys.exit(main())
