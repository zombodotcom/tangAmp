"""
12AX7 Triode LUT Generator
Generates lookup tables for the Koren tube model for use in FPGA WDF implementation.

Outputs:
  - ip_lut.hex       : plate current Ip(Vpk, Vgk) as 16-bit fixed point
  - dip_dvgk_lut.hex : derivative dIp/dVgk(Vpk, Vgk) as 16-bit fixed point
  - lut_params.v     : Verilog parameters for LUT addressing
  - tube_plot.png    : visualization of the tube curves
"""

import numpy as np
import matplotlib.pyplot as plt
import os

# ─── Tube Parameters ──────────────────────────────────────────────────────────
# Common preamp and power tubes — swap these to change amp character
# Old constants (pre-fit, for reference):
#   "12AX7": dict(mu=100.0, ex=1.4,  kg1=1060.0, kp=600.0, kvb=300.0),
#   "12AU7": dict(mu=21.5,  ex=1.35, kg1=1180.0, kp=84.0,  kvb=300.0),
#   "6SL7":  dict(mu=70.0,  ex=1.4,  kg1=1060.0, kp=600.0, kvb=300.0),
# New constants fitted to RCA datasheet data (fit_tube_model.py) and paengdesign/miko fits:
TUBES = {
    # Preamp tubes
    "12AX7": dict(mu=92.08,  ex=1.29, kg1=1304.71, kp=561.08, kvb=15101.91),  # fitted to RCA data
    "12AU7": dict(mu=27.48,  ex=1.03, kg1=466.13,  kp=135.10, kvb=24224.55),  # fitted to RCA data
    "6SL7":  dict(mu=90.41,  ex=1.25, kg1=597.32,  kp=511.97, kvb=6747.79),   # paengdesign miko fit
    # Power tubes (triode-connected)
    "EL34":  dict(mu=10.98,  ex=1.42, kg1=249.65,  kp=43.2,   kvb=333.0),     # paengdesign
    "6L6":   dict(mu=10.11,  ex=1.37, kg1=406.6,   kp=31.2,   kvb=640.7),     # paengdesign
    "300B":  dict(mu=3.95,   ex=1.4,  kg1=1550.0,  kp=65.0,   kvb=300.0),     # paengdesign
}

TUBE = "12AX7"
p = TUBES[TUBE]

# ─── Operating Range ──────────────────────────────────────────────────────────
# Typical common-cathode preamp stage operating points
VPK_MIN  =   0.0   # V  plate-cathode min
VPK_MAX  = 300.0   # V  plate-cathode max (B+ supply)
VGK_MIN  =  -4.0   # V  grid-cathode min (cutoff region)
VGK_MAX  =   0.0   # V  grid-cathode max (onset of grid current)

LUT_SIZE = 256     # entries per axis — 256x256 = 64K entries total

# ─── Fixed Point Format ───────────────────────────────────────────────────────
# Q1.15 format: 1 sign bit, 15 fractional bits
# Scale factor maps physical amps to integer range
IP_SCALE       = 1e4   # 1 unit = 0.1mA  (max ~20mA covers most triodes)
DIP_SCALE      = 1e5   # derivative scale (dIp/dVgk)
DIP_VPK_SCALE  = 1e5   # derivative scale (dIp/dVpk)

# ─── Koren Triode Model ───────────────────────────────────────────────────────
def koren_ip(Vpk, Vgk, mu, ex, kg1, kp, kvb):
    """
    Koren triode plate current model.
    Returns Ip in amps.
    """
    # Argument to log — clamp to avoid log(0)
    inner = kp * (1.0/mu + Vgk / np.sqrt(kvb + Vpk**2 + 1e-9))
    Ed = (Vpk / kp) * np.log1p(np.exp(np.clip(inner, -500, 500)))
    Ip = (np.maximum(Ed, 0.0) ** ex) / kg1
    return Ip

def koren_dip_dvgk(Vpk, Vgk, mu, ex, kg1, kp, kvb):
    """
    Analytical derivative dIp/dVgk of the Koren model.
    Used for Newton-Raphson step in WDF nonlinear solver.
    """
    sqrt_term = np.sqrt(kvb + Vpk**2 + 1e-9)
    inner = kp * (1.0/mu + Vgk / sqrt_term)
    inner_c = np.clip(inner, -500, 500)
    
    Ed = (Vpk / kp) * np.log1p(np.exp(inner_c))
    Ed = np.maximum(Ed, 0.0)
    
    # sigmoid-like term from log1p(exp(x)) derivative
    sig = np.exp(inner_c) / (1.0 + np.exp(inner_c))
    dEd_dVgk = (Vpk / sqrt_term) * sig
    
    # chain rule through power
    dIp = np.where(Ed > 0,
                   (ex / kg1) * Ed**(ex - 1.0) * dEd_dVgk,
                   0.0)
    return dIp

def koren_dip_dvpk(Vpk, Vgk, mu, ex, kg1, kp, kvb):
    """
    Analytical derivative dIp/dVpk of the Koren model.
    Needed for Newton-Raphson Jacobian in WDF nonlinear root solver.
    """
    sqrt_term = np.sqrt(kvb + Vpk**2 + 1e-9)
    inner = kp * (1.0/mu + Vgk / sqrt_term)
    inner_c = np.clip(inner, -500, 500)

    Ed = (Vpk / kp) * np.log1p(np.exp(inner_c))
    Ed = np.maximum(Ed, 0.0)

    # sigmoid-like term from log1p(exp(x)) derivative
    sig = np.exp(inner_c) / (1.0 + np.exp(inner_c))

    # d(inner)/dVpk = kp * Vgk * (-Vpk) / (kvb + Vpk^2)^(3/2)
    dinner_dVpk = kp * Vgk * (-Vpk) / (sqrt_term**3)

    # dEd/dVpk via product rule: (1/kp)*log1p(exp(inner)) + (Vpk/kp)*sig*d(inner)/dVpk
    dEd_dVpk = (1.0 / kp) * np.log1p(np.exp(inner_c)) + (Vpk / kp) * sig * dinner_dVpk

    # chain rule through power
    dIp = np.where(Ed > 0,
                   (ex / kg1) * Ed**(ex - 1.0) * dEd_dVpk,
                   0.0)
    return dIp

# ─── Generate LUT Grids ───────────────────────────────────────────────────────
print(f"Generating LUTs for {TUBE}...")
print(f"  Grid: {LUT_SIZE}x{LUT_SIZE} ({LUT_SIZE*LUT_SIZE} entries)")
print(f"  Vpk range:  {VPK_MIN:.0f}V to {VPK_MAX:.0f}V")
print(f"  Vgk range:  {VGK_MIN:.0f}V to {VGK_MAX:.0f}V")

Vpk_axis = np.linspace(VPK_MIN, VPK_MAX, LUT_SIZE)
Vgk_axis = np.linspace(VGK_MIN, VGK_MAX, LUT_SIZE)
Vpk_grid, Vgk_grid = np.meshgrid(Vpk_axis, Vgk_axis, indexing='ij')

ip_grid       = koren_ip(Vpk_grid, Vgk_grid, **p)
dip_grid      = koren_dip_dvgk(Vpk_grid, Vgk_grid, **p)
dip_vpk_grid  = koren_dip_dvpk(Vpk_grid, Vgk_grid, **p)

print(f"  Ip range:       {ip_grid.min()*1000:.3f}mA to {ip_grid.max()*1000:.3f}mA")
print(f"  dIp/dVgk range: {dip_grid.min():.4f} to {dip_grid.max():.4f} A/V")
print(f"  dIp/dVpk range: {dip_vpk_grid.min():.6f} to {dip_vpk_grid.max():.6f} A/V")

# ─── Quantize to Fixed Point ──────────────────────────────────────────────────
def to_fixed16(arr, scale):
    """Scale float array to 16-bit signed integers, clamped."""
    scaled = arr * scale
    clamped = np.clip(scaled, -32768, 32767)
    return clamped.astype(np.int16)

ip_fixed      = to_fixed16(ip_grid,  IP_SCALE)
dip_fixed     = to_fixed16(dip_grid, DIP_SCALE)
dip_vpk_fixed = to_fixed16(dip_vpk_grid, DIP_VPK_SCALE)

# ─── Write Hex Files ($readmemh format) ───────────────────────────────────────
def write_hex(filename, data_2d):
    """Write 2D array as flat hex file for Verilog $readmemh."""
    flat = data_2d.flatten().astype(np.uint16)  # reinterpret as unsigned
    with open(filename, 'w') as f:
        for i, val in enumerate(flat):
            f.write(f"{val:04X}\n")
    print(f"  Written: {filename} ({len(flat)} entries)")

write_hex("ip_lut.hex",       ip_fixed)
write_hex("dip_dvgk_lut.hex", dip_fixed)
write_hex("dip_dvpk_lut.hex", dip_vpk_fixed)

# ─── Write Verilog Parameters ─────────────────────────────────────────────────
verilog_params = f"""\
// Auto-generated by tube_lut_gen.py
// Tube: {TUBE}
// Do not edit manually

// LUT dimensions
parameter LUT_SIZE    = {LUT_SIZE};
parameter LUT_BITS    = {int(np.log2(LUT_SIZE))};  // {int(np.log2(LUT_SIZE))} bits to address {LUT_SIZE} entries

// Physical ranges (scaled to fixed point for address calculation)
// Vpk: {VPK_MIN}V to {VPK_MAX}V
// Vgk: {VGK_MIN}V to {VGK_MAX}V
parameter VPK_MIN_MV  = {int(VPK_MIN*1000)};   // millivolts
parameter VPK_MAX_MV  = {int(VPK_MAX*1000)};
parameter VGK_MIN_MV  = {int(VGK_MIN*1000)};
parameter VGK_MAX_MV  = {int(VGK_MAX*1000)};

// Scale factors (multiply float by these to get LUT integer values)
parameter IP_SCALE    = {int(IP_SCALE)};   // Ip * {int(IP_SCALE)} = 16-bit int
parameter DIP_SCALE   = {int(DIP_SCALE)};  // dIp/dVgk * {int(DIP_SCALE)} = 16-bit int
parameter DIP_VPK_SCALE = {int(DIP_VPK_SCALE)};  // dIp/dVpk * {int(DIP_VPK_SCALE)} = 16-bit int

// Tube constants (for reference / soft parameter override)
parameter real MU  = {p['mu']};
parameter real EX  = {p['ex']};
parameter real KG1 = {p['kg1']};
parameter real KP  = {p['kp']};
parameter real KVB = {p['kvb']};
"""

with open("lut_params.v", 'w') as f:
    f.write(verilog_params)
print("  Written: lut_params.v")

# ─── Visualize ────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(14, 5))
fig.suptitle(f"{TUBE} Triode Model — Koren Equation", fontsize=13)

# Plate curves (Ip vs Vpk for several Vgk values)
ax = axes[0]
vgk_plot_vals = np.linspace(VGK_MIN, VGK_MAX, 6)
for vgk in vgk_plot_vals:
    ip_curve = koren_ip(Vpk_axis, vgk, **p) * 1000  # to mA
    ax.plot(Vpk_axis, ip_curve, label=f"Vgk={vgk:.1f}V")
ax.set_xlabel("Plate Voltage Vpk (V)")
ax.set_ylabel("Plate Current Ip (mA)")
ax.set_title("Plate Curves")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)
ax.set_xlim(0, VPK_MAX)
ax.set_ylim(0)

# Transfer curve (Ip vs Vgk at fixed Vpk)
ax = axes[1]
vpk_plot_vals = [50, 100, 150, 200, 250]
for vpk in vpk_plot_vals:
    ip_curve = koren_ip(vpk, Vgk_axis, **p) * 1000
    ax.plot(Vgk_axis, ip_curve, label=f"Vpk={vpk}V")
ax.set_xlabel("Grid Voltage Vgk (V)")
ax.set_ylabel("Plate Current Ip (mA)")
ax.set_title("Transfer Curves (your guitar signal is Vgk)")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("tube_plot.png", dpi=150, bbox_inches='tight')
print("  Written: tube_plot.png")

# ─── Summary ──────────────────────────────────────────────────────────────────
print(f"\nDone. Files generated:")
print(f"  ip_lut.hex        — load with $readmemh in Verilog BRAM")
print(f"  dip_dvgk_lut.hex  — derivative dIp/dVgk LUT for Newton step")
print(f"  dip_dvpk_lut.hex  — derivative dIp/dVpk LUT for Jacobian")
print(f"  lut_params.v      — include in your Verilog top module")
print(f"  tube_plot.png     — visual check of the model")
print(f"\nTo use a different tube, change TUBE = '12AX7' to '12AU7' or '6SL7'")
print(f"To change resolution, change LUT_SIZE (must be power of 2)")

# WDF constants for Verilog hardcoding
R_cin = 1.0/(2*48000*22e-9)
R_ck = 1.0/(2*48000*22e-6)
R_plate = 100000 + 1
R_grid = R_cin + 1000000
R_cath = 1.0/(1.0/1500 + 1.0/R_ck)
print(f"\n=== WDF Constants (for Verilog) ===")
print(f"R_CIN = {R_cin:.4f}")
print(f"R_CK = {R_ck:.4f}")
print(f"R_PLATE = {R_plate:.4f}")
print(f"R_GRID = {R_grid:.4f}")
print(f"R_CATHODE = {R_cath:.6f}")
print(f"GAMMA_PLATE = {int(100000/R_plate * 65536)} // Q16.16")
print(f"GAMMA_GRID = {int(R_cin/R_grid * 65536)} // Q16.16")
print(f"GAMMA_CATHODE = {int((1.0/1500)/(1.0/1500+1.0/R_ck) * 65536)} // Q16.16")
print(f"TWO_VB_FP = {int(400 * 65536)} // 2*VB in Q16.16")
print(f"R_PK_INT = {int(R_plate + R_cath)}")
print(f"TWO_R_PLATE = {int(2*R_plate)}")
print(f"TWO_R_CATHODE_SCALED = {int(2*R_cath*65536)} // Q16.16")
