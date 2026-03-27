r"""
wdf_triode_sim_wdf.py
Wave Digital Filter simulation of a single common-cathode 12AX7 triode stage.

Uses WDF wave variables (incident/reflected wave pairs) with a nonlinear
root solver. This serves as the ground truth for the Verilog WDF implementation.

WDF Tree:
            [TRIODE 3-PORT ROOT]
           /        |          \
    plate_leaf  grid_leaf  cathode_leaf
    (Rp+VB)     (Rg+HP)      (Rk)

Each port is a single WDF leaf element connected to the triode root.
The root solver uses Newton-Raphson to find plate current Ip.

Plate leaf: Thevenin source modeling RP connected to VB.
  R0 = RP, b = VB (constant reflected wave).
  This gives v_plate = VB - RP*Ip (correct load line).

Grid leaf: Resistor RG driven by external IIR high-pass (Cin+Rg coupling).
  R0 = RG, b = v_grid_filtered each sample.
  With Ig=0, v_grid = b_grid = v_filtered.

Cathode leaf: Resistor RK to ground.
  R0 = RK, b = 0. v_cathode = RK*Ip (self-bias).
  (When Ck bypass cap is added later, use Parallel(Rk,Ck) adaptor here.)
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# =============================================================================
# Circuit Parameters
# =============================================================================

VB  = 200.0       # B+ supply (V)
RP  = 100000.0    # Plate resistor (ohms)
RG  = 1000000.0   # Grid resistor (ohms)
RK  = 1500.0      # Cathode resistor (ohms)
CIN = 22e-9       # Input coupling cap (F) -- 22nF
CK  = 22e-6       # Cathode bypass cap (F) -- 22uF (not used in this stage)
FS  = 48000.0     # Sample rate (Hz)

# =============================================================================
# Koren 12AX7 Triode Model
# =============================================================================

MU, EX, KG1, KP, KVB = 100.0, 1.4, 1060.0, 600.0, 300.0

def koren_ip(vpk, vgk):
    """Plate current Ip in amps."""
    vpk = float(np.clip(vpk, 0.0, 500.0))
    vgk = float(np.clip(vgk, -10.0, 1.0))
    if vpk <= 0:
        return 0.0
    inner = KP * (1.0 / MU + vgk / np.sqrt(KVB + vpk * vpk))
    inner = np.clip(inner, -500, 500)
    Ed = (vpk / KP) * np.log1p(np.exp(float(inner)))
    if Ed <= 0:
        return 0.0
    return (Ed ** EX) / KG1

def koren_dip_dvpk(vpk, vgk, h=0.01):
    """Numerical dIp/dVpk."""
    return (koren_ip(vpk + h, vgk) - koren_ip(vpk - h, vgk)) / (2 * h)

def koren_dip_dvgk(vpk, vgk, h=0.01):
    """Numerical dIp/dVgk."""
    return (koren_ip(vpk, vgk + h) - koren_ip(vpk, vgk - h)) / (2 * h)

# =============================================================================
# Input Coupling: Bilinear-Transform High-Pass Filter (Cin + Rg)
# =============================================================================

tau_hp = RG * CIN   # 0.022 s
k_hp = 2.0 * FS * tau_hp
c_hp = (k_hp - 1.0) / (k_hp + 1.0)
hp_gain = (1.0 + c_hp) / 2.0

print(f"=== Coupling Cap High-Pass ===")
print(f"tau = {tau_hp*1000:.1f} ms, f_c = {1/(2*np.pi*tau_hp):.1f} Hz, c = {c_hp:.6f}")

# =============================================================================
# WDF Elements
# =============================================================================

class Resistor:
    def __init__(self, R):
        self.R0 = R
        self.a = 0.0
        self.b = 0.0

    def reflect(self):
        self.b = 0.0

class Capacitor:
    def __init__(self, C, fs):
        self.R0 = 1.0 / (2.0 * fs * C)
        self.a = 0.0
        self.b = 0.0
        self.z = 0.0

    def reflect(self):
        self.b = self.z

    def update_state(self):
        self.z = self.a

# =============================================================================
# Build WDF Tree
# =============================================================================

# Plate leaf: Thevenin source (VB through RP)
# R0 = RP, b = VB (constant -- no state feedback)
# v_plate = (a + VB)/2, i = (a - VB)/(2*RP)
# => v = VB + RP*i_in = VB - RP*Ip (since Ip flows out of source)
R_plate = RP

# Grid leaf: Rg with external high-pass filter
# R0 = RG, b = v_grid_filtered (set each sample)
R_grid = RG

# Cathode: single resistor Rk (no bypass cap, matching reference sim)
# When Ck is added later, use Parallel(Rk, Ck) adaptor here.
rk_elem = Resistor(RK)
R_cathode = rk_elem.R0

print(f"\n=== WDF Port Resistances ===")
print(f"Plate R0:   {R_plate:.1f} ohm  (RP, Thevenin source)")
print(f"Grid R0:    {R_grid:.1f} ohm  (RG)")
print(f"Cathode R0: {R_cathode:.1f} ohm  (Rk)")

# =============================================================================
# Simulation
# =============================================================================

n_settle = 2000
n_audio  = 4800
n_total  = n_settle + n_audio

t = np.arange(n_total) / FS
audio_in = np.zeros(n_total)
audio_in[n_settle:] = 0.5 * np.sin(2 * np.pi * 440 * t[n_settle:])

out_vplate = np.zeros(n_total)
out_vgrid  = np.zeros(n_total)
out_vk     = np.zeros(n_total)
out_ip     = np.zeros(n_total)

print(f"\nSimulating {n_total} samples...")

prev_ip = 0.5e-3

# High-pass filter state
hp_y = 0.0
hp_x_prev = 0.0

for n in range(n_total):
    vin = audio_in[n]

    # === Input coupling: IIR high-pass filter ===
    hp_y = c_hp * hp_y + hp_gain * (vin - hp_x_prev)
    hp_x_prev = vin
    v_grid_filtered = hp_y

    # =================================================================
    # UPWARD PASS: compute reflected waves from leaves
    # =================================================================

    # Plate leaf: b = VB (Thevenin source, constant reflected wave)
    b_plate = VB

    # Grid leaf: b = v_grid_filtered
    b_grid = v_grid_filtered

    # Cathode: single resistor leaf
    rk_elem.reflect()    # b = 0
    b_cathode = rk_elem.b

    # =================================================================
    # ROOT: Solve triode nonlinearity
    # =================================================================
    # Incident waves to root from each subtree:
    a_p = b_plate      # plate (single leaf, direct)
    a_g = b_grid       # grid (single leaf, direct)
    a_k = b_cathode    # cathode (parallel adaptor, direct polarity)

    R_p = R_plate
    R_g = R_grid
    R_k = R_cathode

    # Newton-Raphson: Ip = koren(Vpk, Vgk)
    # v_plate = (a_p + b_p)/2 where b_p = a_p - 2*R_p*Ip
    #   => v_plate = a_p - R_p*Ip
    # v_cathode = (a_k + b_k)/2 where b_k = a_k + 2*R_k*Ip
    #   => v_cathode = a_k + R_k*Ip
    # v_grid = a_g (since Ig=0, b_g = a_g)
    # Vpk = v_plate - v_cathode = (a_p - a_k) - (R_p + R_k)*Ip
    # Vgk = v_grid - v_cathode = a_g - a_k - R_k*Ip

    Ip = prev_ip
    for iteration in range(20):
        Vpk = (a_p - a_k) - (R_p + R_k) * Ip
        Vgk = a_g - a_k - R_k * Ip

        ip_model = koren_ip(Vpk, Vgk)
        f_val = Ip - ip_model

        if abs(f_val) < 1e-10:
            break

        dip_dvpk = koren_dip_dvpk(Vpk, Vgk)
        dip_dvgk = koren_dip_dvgk(Vpk, Vgk)
        df_dIp = 1.0 + dip_dvpk * (R_p + R_k) + dip_dvgk * R_k

        if abs(df_dIp) < 1e-15:
            break

        Ip -= f_val / df_dIp
        Ip = max(Ip, 0.0)

    prev_ip = Ip

    # Reflected waves from triode back to subtrees
    b_p = a_p - 2.0 * R_p * Ip   # plate: Ip flows in, voltage drops
    b_g = a_g                      # grid: no current
    b_k = a_k + 2.0 * R_k * Ip   # cathode: Ip flows out

    # Extract tube node voltages
    v_plate   = (a_p + b_p) / 2.0   # = a_p - R_p*Ip = VB - RP*Ip
    v_grid    = (a_g + b_g) / 2.0   # = a_g = v_grid_filtered
    v_cathode = (a_k + b_k) / 2.0   # = a_k + R_k*Ip

    out_vplate[n] = v_plate
    out_vgrid[n]  = v_grid
    out_vk[n]     = v_cathode
    out_ip[n]     = Ip

    # =================================================================
    # DOWNWARD PASS: push reflected waves to leaves
    # =================================================================

    # Plate leaf: no state to update (b = VB is constant)
    # The plate leaf doesn't use its incident wave for anything.

    # Grid leaf: no state to update

    # Cathode: single resistor, set incident wave (not used, but for completeness)
    rk_elem.a = b_k

    if n < 5 or (n % 500 == 0):
        print(f"  [{n:5d}] Vp={v_plate:.2f}V  Vg={v_grid:.4f}V  "
              f"Vk={v_cathode:.3f}V  Ip={Ip*1000:.4f}mA  vin={vin:.4f}")

# =============================================================================
# Analysis
# =============================================================================

print(f"\n=== DC Settling (last 100 before audio) ===")
sl = slice(n_settle - 100, n_settle)
vp_dc = out_vplate[sl].mean()
print(f"V_plate: {vp_dc:.2f}V (std={out_vplate[sl].std():.4f}V)")
print(f"V_grid:  {out_vgrid[sl].mean():.6f}V")
print(f"V_cathode: {out_vk[sl].mean():.4f}V")
print(f"Ip:      {out_ip[sl].mean()*1000:.3f}mA")

audio_out = out_vplate[n_settle:] - vp_dc
inp = audio_in[n_settle:]
in_rms = np.sqrt(np.mean(inp**2))
out_rms = np.sqrt(np.mean(audio_out**2))
gain_db = 20 * np.log10(out_rms / (in_rms + 1e-12)) if in_rms > 0 else 0

print(f"\n=== Audio ===")
print(f"Input:  {inp.min():.3f}V to {inp.max():.3f}V  rms={in_rms:.3f}V")
print(f"Output: {audio_out.min():.2f}V to {audio_out.max():.2f}V  rms={out_rms:.2f}V")
print(f"Gain:   {gain_db:.1f} dB")

# =============================================================================
# Plots
# =============================================================================

t_ms = t * 1000

fig, axes = plt.subplots(4, 1, figsize=(14, 12))
fig.suptitle("12AX7 Triode Stage -- WDF Simulation", fontsize=14)

ax = axes[0]
ax.plot(t_ms[:n_settle], out_vplate[:n_settle], 'r', linewidth=0.8)
ax.axhline(vp_dc, color='gray', linestyle='--', alpha=0.5, label=f'DC={vp_dc:.0f}V')
ax.set_ylabel("Plate Voltage (V)")
ax.set_title("DC Settling")
ax.legend()
ax.grid(True, alpha=0.3)

t_a = t_ms[n_settle:] - t_ms[n_settle]
mask = t_a < 10

ax = axes[1]
ax.plot(t_a[mask], inp[mask], 'b', linewidth=1.5, label='Input')
ax.set_ylabel("V")
ax.set_title("Input -- 440Hz, 0.5V")
ax.legend()
ax.grid(True, alpha=0.3)

ax = axes[2]
ax.plot(t_a[mask], audio_out[mask], 'r', linewidth=1.5, label='Output (AC)')
ax.set_ylabel("V")
ax.set_title(f"Output ({gain_db:.1f} dB gain)")
ax.legend()
ax.grid(True, alpha=0.3)

ax = axes[3]
in_n = inp[mask] / (np.max(np.abs(inp[mask])) + 1e-12)
out_n = audio_out[mask] / (np.max(np.abs(audio_out[mask])) + 1e-12)
ax.plot(t_a[mask], in_n, 'b', alpha=0.6, label='Input (norm)')
ax.plot(t_a[mask], out_n, 'r', alpha=0.6, label='Output (norm)')
ax.set_xlabel("Time (ms)")
ax.set_ylabel("Norm")
ax.set_title("Overlay -- asymmetric clipping = tube character")
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("wdf_sim_wdf_waveforms.png", dpi=150)
print("\nSaved: wdf_sim_wdf_waveforms.png")

# FFT
fig2, axes2 = plt.subplots(1, 2, figsize=(14, 5))
fig2.suptitle("Harmonic Content -- WDF", fontsize=13)

def plot_fft(ax, sig, title="", color='b'):
    N = len(sig)
    w = np.hanning(N)
    sp = np.abs(np.fft.rfft(sig * w))
    sp_db = 20 * np.log10(sp / (max(sp) + 1e-12) + 1e-12)
    f = np.fft.rfftfreq(N, 1 / FS)
    ax.plot(f, sp_db, color=color, linewidth=0.8)
    ax.set_xlim(0, 5000)
    ax.set_ylim(-80, 5)
    ax.set_xlabel("Hz")
    ax.set_ylabel("dB")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    for h in range(1, 8):
        ax.axvline(440 * h, color='gray', alpha=0.3, ls='--', lw=0.5)

plot_fft(axes2[0], inp, "Input Spectrum", 'blue')
plot_fft(axes2[1], audio_out, "Output Spectrum (harmonics = tube saturation)", 'red')
plt.tight_layout()
plt.savefig("wdf_sim_wdf_harmonics.png", dpi=150)
print("Saved: wdf_sim_wdf_harmonics.png")

# Write output file
with open("wdf_sim_wdf_output.txt", "w") as f:
    full_out = out_vplate - vp_dc
    for n in range(n_total):
        f.write(f"{n} {int(audio_in[n]*65536)} {int(full_out[n]*65536)}\n")
print("Saved: wdf_sim_wdf_output.txt")
print("Done.")
