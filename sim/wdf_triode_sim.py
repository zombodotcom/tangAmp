"""
wdf_triode_sim.py
Floating-point simulation of a single common-cathode 12AX7 triode stage.

Approach:
  - Coupling cap (Cin+Rg): bilinear-transform IIR high-pass filter
  - Tube: Koren equation solved iteratively each sample
  - Load line: V_plate = VB - Rp * Ip
  - Cathode bias: V_cathode = Rk * Ip (self-bias)

This avoids the WDF 3-terminal problem by treating the tube as a nonlinear
function mapping (Vpk, Vgk) -> Ip, and computing the operating point each
sample using Newton-Raphson iteration.
"""

import numpy as np
import math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

try:
    import numba
    HAS_NUMBA = True
except ImportError:
    HAS_NUMBA = False

# ═══════════════════════════════════════════════════════════════════════════════
# Circuit Parameters
# ═══════════════════════════════════════════════════════════════════════════════

VB  = 200.0      # B+ supply (V)
RP  = 100000.0   # Plate resistor (ohms)
RG  = 1000000.0  # Grid resistor (ohms)
RK  = 1500.0     # Cathode resistor (ohms)
CIN = 22e-9      # Input coupling cap (F) — 22nF
CK  = 22e-6      # Cathode bypass cap (F) — 22uF
FS  = 48000.0    # Sample rate (Hz)

# Cathode bypass cap WDF parameters
R_CK = 1.0 / (2.0 * FS * CK)  # ~0.4735 ohm
G_rk = 1.0 / RK
G_ck = 1.0 / R_CK
G_total = G_rk + G_ck
R_CATH = 1.0 / G_total          # port resistance seen by root
GAMMA_CATH = G_rk / G_total     # scattering coefficient

# ═══════════════════════════════════════════════════════════════════════════════
# Koren 12AX7 Triode Model
# ═══════════════════════════════════════════════════════════════════════════════

MU, EX, KG1, KP, KVB = 100.0, 1.4, 1060.0, 600.0, 300.0

def koren_ip(vpk, vgk):
    """Plate current Ip in amps. Clamped for safety."""
    vpk = float(np.clip(vpk, 0.0, 500.0))
    vgk = float(np.clip(vgk, -10.0, 1.0))
    if vpk <= 0:
        return 0.0
    inner = KP * (1.0/MU + vgk / np.sqrt(KVB + vpk**2))
    inner = np.clip(inner, -500, 500)
    Ed = (vpk / KP) * np.log1p(np.exp(float(inner)))
    if Ed <= 0:
        return 0.0
    return (Ed ** EX) / KG1

def koren_dip(vpk, vgk, h=0.01):
    """Numerical derivative dIp/dVpk for Newton solver."""
    return (koren_ip(vpk + h, vgk) - koren_ip(vpk - h, vgk)) / (2*h)

# ═══════════════════════════════════════════════════════════════════════════════
# Coupling Cap: Bilinear-Transform High-Pass Filter
# H(s) = s*tau / (s*tau + 1) where tau = Rg * Cin
# Bilinear: c = (2*Fs*tau - 1) / (2*Fs*tau + 1)
# y[n] = c*y[n-1] + (1+c)/2 * (x[n] - x[n-1])
# ═══════════════════════════════════════════════════════════════════════════════

tau = RG * CIN  # 0.022 s
k = 2.0 * FS * tau  # 2112
c_hp = (k - 1.0) / (k + 1.0)  # 0.999054
hp_gain = (1.0 + c_hp) / 2.0   # ≈ 1.0

print(f"=== Coupling Cap High-Pass ===")
print(f"tau = {tau*1000:.1f} ms, f_c = {1/(2*np.pi*tau):.1f} Hz, c = {c_hp:.6f}")

# ═══════════════════════════════════════════════════════════════════════════════
# Numba JIT-compiled simulation loop
# ═══════════════════════════════════════════════════════════════════════════════

if HAS_NUMBA:
    @numba.jit(nopython=True)
    def _koren_ip_jit(vpk, vgk, MU, EX, KG1, KP, KVB):
        vpk = min(max(vpk, 0.0), 500.0)
        vgk = min(max(vgk, -10.0), 1.0)
        if vpk <= 0.0:
            return 0.0
        inner = KP * (1.0 / MU + vgk / math.sqrt(KVB + vpk * vpk))
        inner = min(max(inner, -500.0), 500.0)
        Ed = (vpk / KP) * math.log(1.0 + math.exp(inner))
        if Ed <= 0.0:
            return 0.0
        return (Ed ** EX) / KG1

    @numba.jit(nopython=True)
    def _simulate_nr_jit(audio_in, n_total, VB, RP, RG, RK, R_k_port,
                         gamma_cath, c_hp, hp_gain, ip_dc, MU, EX, KG1, KP, KVB):
        out_vp = np.zeros(n_total)
        out_vg = np.zeros(n_total)
        out_ip = np.zeros(n_total)

        hp_y = 0.0
        hp_x_prev = 0.0
        ip_prev = ip_dc
        z_ck = 0.0  # capacitor state for Ck

        for n in range(n_total):
            vin = audio_in[n]

            # Coupling cap high-pass
            hp_y = c_hp * hp_y + hp_gain * (vin - hp_x_prev)
            hp_x_prev = vin
            v_grid = hp_y

            # Cathode: parallel adaptor Rk || Ck (upward pass)
            b_rk = 0.0
            b_ck = z_ck
            bDiff = b_ck - b_rk
            b_cathode = b_ck - gamma_cath * bDiff

            # WDF root: Newton-Raphson
            a_p = VB
            a_g = v_grid
            a_k = b_cathode
            R_p = RP
            R_k = R_k_port

            ip = ip_prev
            for _ in range(20):
                vpk = (a_p - a_k) - (R_p + R_k) * ip
                vgk = a_g - a_k - R_k * ip

                ip_model = _koren_ip_jit(vpk, vgk, MU, EX, KG1, KP, KVB)
                f = ip - ip_model

                if abs(f) < 1e-10:
                    break

                h = 0.01
                dip_dvpk = (_koren_ip_jit(vpk + h, vgk, MU, EX, KG1, KP, KVB) -
                            _koren_ip_jit(vpk - h, vgk, MU, EX, KG1, KP, KVB)) / (2.0 * h)
                dip_dvgk = (_koren_ip_jit(vpk, vgk + h, MU, EX, KG1, KP, KVB) -
                            _koren_ip_jit(vpk, vgk - h, MU, EX, KG1, KP, KVB)) / (2.0 * h)
                fp = 1.0 + dip_dvpk * (R_p + R_k) + dip_dvgk * R_k

                if abs(fp) < 1e-15:
                    break

                ip -= f / fp
                ip = max(ip, 0.0)

            ip_prev = ip

            # Grid current: when Vgk > 0, grid draws current
            vgk_final = a_g - a_k - R_k * ip
            if vgk_final > 0:
                ig = 0.002 * vgk_final ** 1.5
            else:
                ig = 0.0
            ip_total = ip + ig

            b_p = a_p - 2.0 * R_p * ip_total
            b_k = a_k + 2.0 * R_k * ip_total
            vp = (a_p + b_p) / 2.0

            # Downward pass: update capacitor state
            a_ck = b_k + b_cathode - b_ck
            z_ck = a_ck

            out_vp[n] = vp
            out_vg[n] = v_grid
            out_ip[n] = ip

        return out_vp, out_vg, out_ip

# ═══════════════════════════════════════════════════════════════════════════════
# Find DC Operating Point
# ═══════════════════════════════════════════════════════════════════════════════

def find_dc_op():
    """Bisection: find Ip where Ip = koren_ip(VB-(RP+RK)*Ip, -RK*Ip)"""
    lo, hi = 0.0, VB / (RP + RK)
    for _ in range(100):
        mid = (lo + hi) / 2
        vpk = VB - (RP + RK) * mid
        vgk = -RK * mid
        r = mid - koren_ip(vpk, vgk)
        if r < 0: lo = mid
        else: hi = mid
    ip = mid
    vk, vp = RK*ip, VB - RP*ip
    print(f"\n=== DC Operating Point ===")
    print(f"Ip={ip*1000:.3f}mA  Vp={vp:.1f}V  Vk={vk:.2f}V  Vgk={-vk:.3f}V  Vpk={vp-vk:.1f}V")
    return ip, vp, vk

ip_dc, vp_dc, vk_dc = find_dc_op()

# ═══════════════════════════════════════════════════════════════════════════════
# Simulation
# ═══════════════════════════════════════════════════════════════════════════════

def simulate(audio_in, n_samples):
    if HAS_NUMBA:
        print("  [Using Numba JIT]")
        out_vp, out_vg, out_ip = _simulate_nr_jit(
            audio_in, n_samples, VB, RP, RG, RK, R_CATH, GAMMA_CATH,
            c_hp, hp_gain, ip_dc, MU, EX, KG1, KP, KVB)
        out = out_vp - vp_dc
        return out, out_vp, out_vg, out_ip

    # State
    hp_y = 0.0      # high-pass filter output (grid voltage)
    hp_x_prev = 0.0 # previous input to filter
    ip_prev = ip_dc # previous plate current (for initial guess)
    z_ck = 0.0      # capacitor state for Ck

    # Output arrays
    out = np.zeros(n_samples)
    out_vp = np.zeros(n_samples)
    out_vg = np.zeros(n_samples)
    out_ip = np.zeros(n_samples)

    for n in range(n_samples):
        vin = audio_in[n] if n < len(audio_in) else 0.0

        # Coupling cap high-pass: Vin -> V_grid
        hp_y = c_hp * hp_y + hp_gain * (vin - hp_x_prev)
        hp_x_prev = vin
        v_grid = hp_y

        # Cathode: parallel adaptor Rk || Ck (upward pass)
        b_rk = 0.0
        b_ck = z_ck
        bDiff = b_ck - b_rk
        b_cathode = b_ck - GAMMA_CATH * bDiff

        # WDF root: Newton-Raphson
        a_p = VB
        a_g = v_grid
        a_k = b_cathode
        R_p = RP
        R_k = R_CATH

        ip = ip_prev
        for _ in range(20):
            vpk = (a_p - a_k) - (R_p + R_k) * ip
            vgk = a_g - a_k - R_k * ip

            ip_model = koren_ip(vpk, vgk)
            f = ip - ip_model

            if abs(f) < 1e-10:
                break

            h = 0.01
            dip_dvpk = (koren_ip(vpk + h, vgk) - koren_ip(vpk - h, vgk)) / (2.0 * h)
            dip_dvgk = (koren_ip(vpk, vgk + h) - koren_ip(vpk, vgk - h)) / (2.0 * h)
            fp = 1.0 + dip_dvpk * (R_p + R_k) + dip_dvgk * R_k

            if abs(fp) < 1e-15:
                break

            ip -= f / fp
            ip = max(ip, 0.0)

        ip_prev = ip

        b_p = a_p - 2.0 * R_p * ip
        b_k = a_k + 2.0 * R_k * ip
        vp = (a_p + b_p) / 2.0

        # Downward pass: update capacitor state
        a_ck = b_k + b_cathode - b_ck
        z_ck = a_ck

        out_vp[n] = vp
        out_vg[n] = v_grid
        out_ip[n] = ip
        out[n] = vp - vp_dc

    return out, out_vp, out_vg, out_ip

# ═══════════════════════════════════════════════════════════════════════════════
# Run
# ═══════════════════════════════════════════════════════════════════════════════

n_settle = 2000
n_audio  = 4800
n_total  = n_settle + n_audio

t = np.arange(n_total) / FS
audio_in = np.zeros(n_total)
audio_in[n_settle:] = 0.5 * np.sin(2 * np.pi * 440 * t[n_settle:])

print(f"\nSimulating {n_total} samples...")
audio_out, vplate, vgrid, ip_out = simulate(audio_in, n_total)

# ═══════════════════════════════════════════════════════════════════════════════
# Analysis
# ═══════════════════════════════════════════════════════════════════════════════

print(f"\n=== DC Settling (last 100 before audio) ===")
sl = slice(n_settle-100, n_settle)
vp_dc_measured = vplate[sl].mean()
print(f"V_plate: {vp_dc_measured:.2f}V (stable={vplate[sl].std():.4f}V)")
print(f"V_grid:  {vgrid[sl].mean():.6f}V")
print(f"Ip:      {ip_out[sl].mean()*1000:.3f}mA")

# Re-compute AC output using measured DC (more accurate with bypass cap)
audio_out = vplate - vp_dc_measured
ap = audio_out[n_settle:]
inp = audio_in[n_settle:]
in_rms = np.sqrt(np.mean(inp**2))
out_rms = np.sqrt(np.mean(ap**2))
gain_db = 20*np.log10(out_rms / in_rms + 1e-12) if in_rms > 0 else 0

print(f"\n=== Audio ===")
print(f"Input:  {inp.min():.3f}V to {inp.max():.3f}V  rms={in_rms:.3f}V")
print(f"Output: {ap.min():.2f}V to {ap.max():.2f}V  rms={out_rms:.2f}V")
print(f"Gain:   {gain_db:.1f} dB")

# ═══════════════════════════════════════════════════════════════════════════════
# Plots
# ═══════════════════════════════════════════════════════════════════════════════

t_ms = t * 1000

fig, axes = plt.subplots(4, 1, figsize=(14, 12))
fig.suptitle("12AX7 Triode Stage — Koren Model + Load Line", fontsize=14)

ax = axes[0]
ax.plot(t_ms[:n_settle], vplate[:n_settle], 'r', linewidth=0.8)
ax.axhline(vp_dc, color='gray', linestyle='--', alpha=0.5, label=f'DC={vp_dc:.0f}V')
ax.set_ylabel("Plate Voltage (V)")
ax.set_title("DC Settling")
ax.legend(); ax.grid(True, alpha=0.3)

t_a = t_ms[n_settle:] - t_ms[n_settle]
mask = t_a < 10

ax = axes[1]
ax.plot(t_a[mask], inp[mask], 'b', linewidth=1.5, label='Input')
ax.set_ylabel("V"); ax.set_title("Input — 440Hz, 0.5V"); ax.legend(); ax.grid(True, alpha=0.3)

ax = axes[2]
ax.plot(t_a[mask], ap[mask], 'r', linewidth=1.5, label='Output (AC)')
ax.set_ylabel("V"); ax.set_title(f"Output ({gain_db:.1f} dB gain)"); ax.legend(); ax.grid(True, alpha=0.3)

ax = axes[3]
in_n = inp[mask] / (np.max(np.abs(inp[mask])) + 1e-12)
out_n = ap[mask] / (np.max(np.abs(ap[mask])) + 1e-12)
ax.plot(t_a[mask], in_n, 'b', alpha=0.6, label='Input (norm)')
ax.plot(t_a[mask], out_n, 'r', alpha=0.6, label='Output (norm)')
ax.set_xlabel("Time (ms)"); ax.set_ylabel("Norm"); ax.set_title("Overlay — asymmetric clipping = tube character")
ax.legend(); ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("wdf_sim_waveforms.png", dpi=150)
print("\nSaved: wdf_sim_waveforms.png")

# FFT
fig2, axes2 = plt.subplots(1, 2, figsize=(14, 5))
fig2.suptitle("Harmonic Content", fontsize=13)

def plot_fft(ax, sig, title="", color='b'):
    N = len(sig); w = np.hanning(N)
    sp = np.abs(np.fft.rfft(sig * w))
    sp_db = 20*np.log10(sp/(max(sp)+1e-12)+1e-12)
    f = np.fft.rfftfreq(N, 1/FS)
    ax.plot(f, sp_db, color=color, linewidth=0.8)
    ax.set_xlim(0, 5000); ax.set_ylim(-80, 5)
    ax.set_xlabel("Hz"); ax.set_ylabel("dB"); ax.set_title(title)
    ax.grid(True, alpha=0.3)
    for h in range(1, 8): ax.axvline(440*h, color='gray', alpha=0.3, ls='--', lw=0.5)

plot_fft(axes2[0], inp, "Input Spectrum", 'blue')
plot_fft(axes2[1], ap, "Output Spectrum (harmonics = tube saturation)", 'red')
plt.tight_layout()
plt.savefig("wdf_sim_harmonics.png", dpi=150)
print("Saved: wdf_sim_harmonics.png")

# Write output file
with open("wdf_sim_output.txt", "w") as f:
    for n in range(n_total):
        f.write(f"{n} {int(audio_in[n]*65536)} {int(audio_out[n]*65536)}\n")
print("Saved: wdf_sim_output.txt\nDone.")
