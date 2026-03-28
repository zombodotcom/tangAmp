"""
12AX7 Triode LUT Generator
Generates lookup tables for the Koren tube model for use in FPGA WDF implementation.

Outputs (in data/):
  - ip_lut.hex       : plate current Ip(Vpk, Vgk) as 16-bit fixed point
  - dip_dvgk_lut.hex : derivative dIp/dVgk(Vpk, Vgk) as 16-bit fixed point
  - lut_params.v     : Verilog parameters for LUT addressing
  - tube_plot.png    : visualization of the tube curves
"""

import numpy as np
import matplotlib.pyplot as plt
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(SCRIPT_DIR, "..", "data")
DEMOS_DIR = os.path.join(SCRIPT_DIR, "..", "demos")
os.makedirs(DATA_DIR, exist_ok=True)

# ─── Tube Parameters ──────────────────────────────────────────────────────────
# Common preamp and power tubes — swap these to change amp character
# Old constants (pre-fit, for reference):
#   "12AX7": dict(mu=100.0, ex=1.4,  kg1=1060.0, kp=600.0, kvb=300.0),
#   "12AU7": dict(mu=21.5,  ex=1.35, kg1=1180.0, kp=84.0,  kvb=300.0),
#   "6SL7":  dict(mu=70.0,  ex=1.4,  kg1=1060.0, kp=600.0, kvb=300.0),
# New constants fitted to RCA datasheet data (fit_tube_model.py) and paengdesign/miko fits:
TUBES = {
    # Preamp tubes
    "12AX7": dict(mu=92.08, ex=1.29, kg1=1304.71, kp=561.08, kvb=15101.91),  # fitted to RCA datasheet (fit_tube_model.py)
    "12AU7": dict(mu=27.48,  ex=1.03, kg1=466.13,  kp=135.10, kvb=24224.55),  # fitted to RCA data
    "6SL7":  dict(mu=90.41,  ex=1.25, kg1=597.32,  kp=511.97, kvb=6747.79),   # paengdesign miko fit
    # Power tubes (triode-connected)
    "EL34":  dict(mu=10.98,  ex=1.42, kg1=249.65,  kp=43.2,   kvb=333.0),     # paengdesign
    "6L6":   dict(mu=10.11,  ex=1.37, kg1=406.6,   kp=31.2,   kvb=640.7),     # paengdesign
    "300B":  dict(mu=3.95,   ex=1.4,  kg1=1550.0,  kp=65.0,   kvb=300.0),     # paengdesign
    "EL84":  dict(mu=18.39,  ex=1.32, kg1=374.24,  kp=85.89,  kvb=3000.0),    # triode-connected, fitted to Mullard datasheet
    "6V6":   dict(mu=10.33,  ex=1.27, kg1=555.36,  kp=93.89,  kvb=4513.17),   # triode-connected, fitted to RCA datasheet
}

# ─── Tube Configurations ─────────────────────────────────────────────────────
# Each config specifies: tube name, operating ranges, output file suffix
TUBE_CONFIGS = {
    "12AX7": {
        "params": TUBES["12AX7"],
        "vpk_min": 0.0, "vpk_max": 300.0,
        "vgk_min": -4.0, "vgk_max": 0.0,
        "suffix": "",  # default files (ip_lut.hex etc.)
    },
    "6L6": {
        "params": TUBES["6L6"],
        "vpk_min": 0.0, "vpk_max": 500.0,   # wider: VB=400V power amp
        "vgk_min": -50.0, "vgk_max": 0.0,    # 6L6 biased more negative
        "suffix": "_6l6",  # ip_lut_6l6.hex etc.
    },
    "EL84": {
        "params": TUBES["EL84"],
        "vpk_min": 0.0, "vpk_max": 400.0,   # Vox AC30: B+=330V
        "vgk_min": -20.0, "vgk_max": 0.0,    # EL84 triode bias range
        "suffix": "_el84",
    },
    "6V6": {
        "params": TUBES["6V6"],
        "vpk_min": 0.0, "vpk_max": 500.0,   # Fender Deluxe: B+=420V
        "vgk_min": -30.0, "vgk_max": 0.0,    # 6V6 triode bias range
        "suffix": "_6v6",
    },
}

LUT_SIZE = 128     # entries per axis — 128x128 = 16K entries per LUT
                   # Halved from 256 to fit both 12AX7 + 6L6 in BSRAM

# ─── Fixed Point Format ───────────────────────────────────────────────────────
# Q1.15 format: 1 sign bit, 15 fractional bits
# Scale factor maps physical amps to integer range
IP_SCALE       = 1e4   # 1 unit = 0.1mA  (max ~20mA covers most triodes)
DIP_SCALE      = 1e5   # derivative scale (dIp/dVgk)
DIP_VPK_SCALE  = 1e7   # derivative scale (dIp/dVpk) — needs high scale, dIp/dVpk is ~1e-5

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

# ─── Quantize to Fixed Point ──────────────────────────────────────────────────
def to_fixed16(arr, scale):
    """Scale float array to 16-bit signed integers, clamped."""
    scaled = arr * scale
    clamped = np.clip(scaled, -32768, 32767)
    return clamped.astype(np.int16)

# ─── Write Hex Files ($readmemh format) ───────────────────────────────────────
def write_hex(filename, data_2d):
    """Write 2D array as flat hex file for Verilog $readmemh."""
    flat = data_2d.flatten().astype(np.uint16)  # reinterpret as unsigned
    with open(filename, 'w') as f:
        for i, val in enumerate(flat):
            f.write(f"{val:04X}\n")
    print(f"  Written: {filename} ({len(flat)} entries)")

# ─── Generate LUTs for Each Tube ─────────────────────────────────────────────
all_plot_data = {}

for tube_name, cfg in TUBE_CONFIGS.items():
    p = cfg["params"]
    VPK_MIN = cfg["vpk_min"]
    VPK_MAX = cfg["vpk_max"]
    VGK_MIN = cfg["vgk_min"]
    VGK_MAX = cfg["vgk_max"]
    suffix = cfg["suffix"]

    print(f"\nGenerating LUTs for {tube_name}...")
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

    ip_fixed      = to_fixed16(ip_grid,  IP_SCALE)
    dip_fixed     = to_fixed16(dip_grid, DIP_SCALE)
    dip_vpk_fixed = to_fixed16(dip_vpk_grid, DIP_VPK_SCALE)

    write_hex(os.path.join(DATA_DIR, f"ip_lut{suffix}.hex"),       ip_fixed)
    write_hex(os.path.join(DATA_DIR, f"dip_dvgk_lut{suffix}.hex"), dip_fixed)
    write_hex(os.path.join(DATA_DIR, f"dip_dvpk_lut{suffix}.hex"), dip_vpk_fixed)

    all_plot_data[tube_name] = {
        "p": p, "Vpk_axis": Vpk_axis, "Vgk_axis": Vgk_axis,
        "VPK_MIN": VPK_MIN, "VPK_MAX": VPK_MAX,
        "VGK_MIN": VGK_MIN, "VGK_MAX": VGK_MAX,
    }

# ─── Write Verilog Parameters (12AX7 preamp defaults) ────────────────────────
cfg_12ax7 = TUBE_CONFIGS["12AX7"]
p_12ax7 = cfg_12ax7["params"]
cfg_6l6 = TUBE_CONFIGS["6L6"]
p_6l6 = cfg_6l6["params"]

verilog_params = f"""\
// Auto-generated by tube_lut_gen.py
// Preamp: 12AX7, Power amp: 6L6
// LUT size: {LUT_SIZE}x{LUT_SIZE} (both tubes)
// Do not edit manually

// LUT dimensions
parameter LUT_SIZE    = {LUT_SIZE};
parameter LUT_BITS    = {int(np.log2(LUT_SIZE))};  // {int(np.log2(LUT_SIZE))} bits to address {LUT_SIZE} entries

// 12AX7 preamp ranges
parameter VPK_MIN_MV  = {int(cfg_12ax7["vpk_min"]*1000)};   // millivolts
parameter VPK_MAX_MV  = {int(cfg_12ax7["vpk_max"]*1000)};
parameter VGK_MIN_MV  = {int(cfg_12ax7["vgk_min"]*1000)};
parameter VGK_MAX_MV  = {int(cfg_12ax7["vgk_max"]*1000)};

// 6L6 power amp ranges
parameter VPK_MIN_MV_6L6  = {int(cfg_6l6["vpk_min"]*1000)};
parameter VPK_MAX_MV_6L6  = {int(cfg_6l6["vpk_max"]*1000)};
parameter VGK_MIN_MV_6L6  = {int(cfg_6l6["vgk_min"]*1000)};
parameter VGK_MAX_MV_6L6  = {int(cfg_6l6["vgk_max"]*1000)};

// Scale factors (multiply float by these to get LUT integer values)
parameter IP_SCALE    = {int(IP_SCALE)};   // Ip * {int(IP_SCALE)} = 16-bit int
parameter DIP_SCALE   = {int(DIP_SCALE)};  // dIp/dVgk * {int(DIP_SCALE)} = 16-bit int
parameter DIP_VPK_SCALE = {int(DIP_VPK_SCALE)};  // dIp/dVpk * {int(DIP_VPK_SCALE)} = 16-bit int

// 12AX7 tube constants (for reference)
parameter real MU_12AX7  = {p_12ax7['mu']};
parameter real EX_12AX7  = {p_12ax7['ex']};
parameter real KG1_12AX7 = {p_12ax7['kg1']};
parameter real KP_12AX7  = {p_12ax7['kp']};
parameter real KVB_12AX7 = {p_12ax7['kvb']};

// 6L6 tube constants (for reference)
parameter real MU_6L6  = {p_6l6['mu']};
parameter real EX_6L6  = {p_6l6['ex']};
parameter real KG1_6L6 = {p_6l6['kg1']};
parameter real KP_6L6  = {p_6l6['kp']};
parameter real KVB_6L6 = {p_6l6['kvb']};
"""

lut_params_path = os.path.join(DATA_DIR, "lut_params.v")
with open(lut_params_path, 'w') as f:
    f.write(verilog_params)
print(f"\n  Written: {lut_params_path}")

# ─── Visualize ────────────────────────────────────────────────────────────────
n_tubes = len(all_plot_data)
fig, axes = plt.subplots(n_tubes, 2, figsize=(14, 5*n_tubes))
fig.suptitle("Tube LUTs: " + " + ".join(all_plot_data.keys()), fontsize=13)

for row, tube_name in enumerate(all_plot_data.keys()):
    pd = all_plot_data[tube_name]
    p = pd["p"]

    # Plate curves
    ax = axes[row, 0]
    vgk_plot_vals = np.linspace(pd["VGK_MIN"], pd["VGK_MAX"], 6)
    for vgk in vgk_plot_vals:
        ip_curve = koren_ip(pd["Vpk_axis"], vgk, **p) * 1000
        ax.plot(pd["Vpk_axis"], ip_curve, label=f"Vgk={vgk:.1f}V")
    ax.set_xlabel("Plate Voltage Vpk (V)")
    ax.set_ylabel("Plate Current Ip (mA)")
    ax.set_title(f"{tube_name} Plate Curves")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, pd["VPK_MAX"])
    ax.set_ylim(0)

    # Transfer curves
    ax = axes[row, 1]
    vpk_plot_vals = np.linspace(pd["VPK_MAX"]*0.2, pd["VPK_MAX"]*0.9, 5)
    for vpk in vpk_plot_vals:
        ip_curve = koren_ip(vpk, pd["Vgk_axis"], **p) * 1000
        ax.plot(pd["Vgk_axis"], ip_curve, label=f"Vpk={vpk:.0f}V")
    ax.set_xlabel("Grid Voltage Vgk (V)")
    ax.set_ylabel("Plate Current Ip (mA)")
    ax.set_title(f"{tube_name} Transfer Curves")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

plt.tight_layout()
tube_plot_path = os.path.join(DEMOS_DIR, "tube_plot.png")
plt.savefig(tube_plot_path, dpi=150, bbox_inches='tight')
print(f"  Written: {tube_plot_path}")

# ─── Grid Current LUTs (for 2x2 Newton-Raphson solver) ──────────────────────
# Langmuir-Child law: Ig = 0.0002 * Vgk^1.5 when Vgk > 0
# Derivative: dIg/dVgk = 0.003 * Vgk^0.5
IG_LUT_SIZE = 64
IG_VGK_MAX = 2.0  # volts

Vgk_ig_axis = np.linspace(0.0, IG_VGK_MAX, IG_LUT_SIZE)

ig_values = 0.0002 * Vgk_ig_axis**1.5
dig_values = 0.003 * Vgk_ig_axis**0.5

# Q16.16 fixed point: multiply by 65536, store as unsigned 16-bit hex
ig_fp = (ig_values * 65536).astype(np.uint16)
dig_fp = (dig_values * 65536).astype(np.uint16)

ig_lut_path = os.path.join(DATA_DIR, "ig_lut.hex")
with open(ig_lut_path, 'w') as f:
    for val in ig_fp:
        f.write(f"{val:04X}\n")
print(f"  Written: {ig_lut_path} ({IG_LUT_SIZE} entries)")

dig_lut_path = os.path.join(DATA_DIR, "dig_lut.hex")
with open(dig_lut_path, 'w') as f:
    for val in dig_fp:
        f.write(f"{val:04X}\n")
print(f"  Written: {dig_lut_path} ({IG_LUT_SIZE} entries)")

IG_VGK_MAX_FP = int(IG_VGK_MAX * 65536)
IG_VGK_STEP_FP = int((IG_VGK_MAX / (IG_LUT_SIZE - 1)) * 65536)
print(f"\n  Grid current LUT parameters (for Verilog):")
print(f"    IG_LUT_SIZE   = {IG_LUT_SIZE}")
print(f"    IG_VGK_MAX_FP = {IG_VGK_MAX_FP}  // {IG_VGK_MAX}V in Q16.16")
print(f"    IG_VGK_STEP_FP = {IG_VGK_STEP_FP}  // {IG_VGK_MAX}/{IG_LUT_SIZE-1} * 65536")

# ─── Summary ──────────────────────────────────────────────────────────────────
print(f"\nDone. Files generated for {LUT_SIZE}x{LUT_SIZE} LUTs:")
for tube_name, cfg in TUBE_CONFIGS.items():
    suffix = cfg["suffix"] if cfg["suffix"] else " (default)"
    print(f"  ip_lut{cfg['suffix']}.hex / dip_dvgk_lut{cfg['suffix']}.hex / dip_dvpk_lut{cfg['suffix']}.hex  ({tube_name})")
print(f"  lut_params.v   — Verilog parameters")
print(f"  tube_plot.png  — visual check")

# WDF constants for Verilog hardcoding
R_cin = 1.0/(2*48000*22e-9)
R_ck = 1.0/(2*48000*22e-6)
R_plate = 100000 + 1
R_grid = R_cin + 1000000
R_cath = 1.0/(1.0/1500 + 1.0/R_ck)
print(f"\n=== WDF Constants (12AX7 preamp, for Verilog) ===")
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

# 6L6 power amp WDF constants
RP_6L6 = 2000
RK_6L6 = 250
R_ck_6l6 = 1.0/(2*48000*22e-6)  # same cap value
R_cath_6l6 = 1.0/(1.0/RK_6L6 + 1.0/R_ck_6l6)
print(f"\n=== WDF Constants (6L6 power amp, for Verilog) ===")
print(f"RP_6L6 = {RP_6L6}")
print(f"RK_6L6 = {RK_6L6}")
print(f"R_CATHODE_6L6 = {R_cath_6l6:.6f}")
print(f"GAMMA_CATHODE_6L6 = {int((1.0/RK_6L6)/(1.0/RK_6L6+1.0/R_ck_6l6) * 65536)} // Q16.16")
print(f"TWO_R_CATHODE_6L6 = {int(2*R_cath_6l6*65536)} // Q16.16")
print(f"RPK_6L6 = {int(RP_6L6 + R_cath_6l6)}")
