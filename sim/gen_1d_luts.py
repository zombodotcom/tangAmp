"""
gen_1d_luts.py -- Generate 1D LUTs for hybrid Koren triode computation on FPGA

Instead of a 128x128 2D LUT for Ip(Vpk, Vgk) = 32KB per table, this decomposes
the Koren equation into steps that each use a small 1D LUT:

  Step 1: sqrt_val = sqrt(kvb + vpk^2)      -> 1D LUT indexed by Vpk (64 entries)
  Step 2: inner = kp * (1/mu + vgk/sqrt_val) -> multiply + add, no LUT
  Step 3: softplus = log(1 + exp(inner))     -> 1D LUT indexed by inner (64 entries)
  Step 4: Ed = (vpk/kp) * softplus           -> multiply, no LUT
  Step 5: Ip = Ed^ex / kg1                   -> 1D LUT for x^ex (64 entries)

Total memory: 3 x 128 bytes = 384 bytes (vs 32KB for 2D LUT).
Frees ~99% of BSRAM used by tube LUTs.

Outputs (in data/):
  - sqrt_lut.hex     : sqrt(kvb + vpk^2) as Q16.16
  - softplus_lut.hex : log(1+exp(x)) as Q16.16
  - power_lut.hex    : x^1.29 as Q16.16
"""

import numpy as np
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(SCRIPT_DIR, "..", "data")
os.makedirs(DATA_DIR, exist_ok=True)

# -- 12AX7 tube constants (fitted to RCA datasheet) --
MU    = 92.08
EX    = 1.29
KG1   = 1304.71
KP    = 561.08
KVB   = 15101.91

# -- LUT parameters --
LUT_SIZE = 64          # entries per 1D LUT
FP_FRAC  = 16          # Q16.16 fractional bits
ONE_FP   = 1 << FP_FRAC  # 65536

# -- Operating ranges --
VPK_MIN = 0.0
VPK_MAX = 300.0
VGK_MIN = -4.0
VGK_MAX = 0.0

# Softplus input range: for 12AX7 operating range, inner typically spans ~[-10, +10]
# but log(1+exp(x)) only needs a LUT for -6 to +6; outside that it's trivial
SOFTPLUS_MIN = -6.0
SOFTPLUS_MAX = 6.0

# Power LUT: Ed range for 12AX7 -- Ed = (Vpk/kp) * softplus
# At operating point (Vpk~135V, Vgk~-0.9V): Ed ≈ 0.03-0.15
# Max Ed occurs at Vpk=300V, Vgk=0: Ed ≈ 0.55
ED_MIN = 0.0
ED_MAX = 0.6  # generous headroom

# ============================================================================
# Generate sqrt LUT: sqrt(kvb + vpk^2)
# ============================================================================
vpk_axis = np.linspace(VPK_MIN, VPK_MAX, LUT_SIZE)
sqrt_vals = np.sqrt(KVB + vpk_axis**2)

# Store as Q16.16 unsigned (values are always positive, range ~123 to ~335)
# Max value ~335, fits in 32-bit Q16.16
sqrt_fp = np.round(sqrt_vals * ONE_FP).astype(np.int64)
# Clamp to 32-bit unsigned
sqrt_fp = np.clip(sqrt_fp, 0, 0xFFFFFFFF).astype(np.uint32)

sqrt_path = os.path.join(DATA_DIR, "sqrt_lut.hex")
with open(sqrt_path, 'w') as f:
    for val in sqrt_fp:
        f.write(f"{val:08X}\n")
print(f"  Written: {sqrt_path} ({LUT_SIZE} entries, Q16.16)")
print(f"    Vpk range: {VPK_MIN:.0f} to {VPK_MAX:.0f} V")
print(f"    sqrt range: {sqrt_vals[0]:.2f} to {sqrt_vals[-1]:.2f}")

# ============================================================================
# Generate inv_sqrt LUT: 1/sqrt(kvb + vpk^2)
# ============================================================================
# This eliminates the 32-bit division in koren_direct.v (saves ~700 LUTs)
# vgk / sqrt_val -> vgk * inv_sqrt_val (one DSP multiply instead of divider)
inv_sqrt_vals = 1.0 / sqrt_vals

# Store as Q16.16 (values range from ~0.003 to ~0.008)
inv_sqrt_fp = np.round(inv_sqrt_vals * ONE_FP).astype(np.int64)
inv_sqrt_fp = np.clip(inv_sqrt_fp, 0, 0xFFFFFFFF).astype(np.uint32)

inv_sqrt_path = os.path.join(DATA_DIR, "inv_sqrt_lut.hex")
with open(inv_sqrt_path, 'w') as f:
    for val in inv_sqrt_fp:
        f.write(f"{val:08X}\n")
print(f"  Written: {inv_sqrt_path} ({LUT_SIZE} entries, Q16.16)")
print(f"    inv_sqrt range: {inv_sqrt_vals[0]:.6f} to {inv_sqrt_vals[-1]:.6f}")

# ============================================================================
# Generate softplus LUT: log(1 + exp(x))
# ============================================================================
softplus_axis = np.linspace(SOFTPLUS_MIN, SOFTPLUS_MAX, LUT_SIZE)
softplus_vals = np.log1p(np.exp(softplus_axis))

# Store as Q16.16 (values range from ~0.002 to ~6.002)
softplus_fp = np.round(softplus_vals * ONE_FP).astype(np.int64)
softplus_fp = np.clip(softplus_fp, 0, 0xFFFFFFFF).astype(np.uint32)

softplus_path = os.path.join(DATA_DIR, "softplus_lut.hex")
with open(softplus_path, 'w') as f:
    for val in softplus_fp:
        f.write(f"{val:08X}\n")
print(f"  Written: {softplus_path} ({LUT_SIZE} entries, Q16.16)")
print(f"    Inner range: {SOFTPLUS_MIN:.1f} to {SOFTPLUS_MAX:.1f}")
print(f"    Softplus range: {softplus_vals[0]:.6f} to {softplus_vals[-1]:.6f}")

# ============================================================================
# Generate power LUT: x^1.29
# ============================================================================
# Ed is always >= 0 (clamped in the Koren equation)
ed_axis = np.linspace(ED_MIN, ED_MAX, LUT_SIZE)
# Avoid 0^1.29 = 0 (which is fine, just ensure no NaN)
power_vals = np.where(ed_axis > 0, ed_axis ** EX, 0.0)

# Store as Q16.16 (values range from 0 to ~0.55^1.29 ≈ 0.46)
power_fp = np.round(power_vals * ONE_FP).astype(np.int64)
power_fp = np.clip(power_fp, 0, 0xFFFFFFFF).astype(np.uint32)

power_path = os.path.join(DATA_DIR, "power_lut.hex")
with open(power_path, 'w') as f:
    for val in power_fp:
        f.write(f"{val:08X}\n")
print(f"  Written: {power_path} ({LUT_SIZE} entries, Q16.16)")
print(f"    Ed range: {ED_MIN:.3f} to {ED_MAX:.3f}")
print(f"    Power range: {power_vals[0]:.6f} to {power_vals[-1]:.6f}")

# ============================================================================
# Generate Verilog parameters include file
# ============================================================================
params = f"""\
// Auto-generated by gen_1d_luts.py -- 1D LUT parameters for koren_direct.v
// Do not edit manually

// 1D LUT dimensions
localparam KLUT_SIZE = {LUT_SIZE};
localparam KLUT_BITS = {int(np.log2(LUT_SIZE))};  // {int(np.log2(LUT_SIZE))} bits to address {LUT_SIZE} entries

// 12AX7 Koren constants in Q16.16
localparam signed [31:0] KOREN_MU_FP     = 32'sd{int(round(MU * ONE_FP))};    // {MU}
localparam signed [31:0] KOREN_KP_FP     = 32'sd{int(round(KP * ONE_FP))};    // {KP}
localparam signed [31:0] KOREN_KG1_FP    = 32'sd{int(round(KG1 * ONE_FP))};   // {KG1}
localparam signed [31:0] KOREN_INV_MU_FP = 32'sd{int(round((1.0/MU) * ONE_FP))}; // 1/{MU}
localparam signed [31:0] KOREN_INV_KP_FP = 32'sd{int(round((1.0/KP) * ONE_FP))}; // 1/{KP}

// Sqrt LUT: index = (Vpk - VPK_MIN) / VPK_STEP
localparam signed [31:0] SQRT_VPK_MIN_FP  = 32'sd{int(round(VPK_MIN * ONE_FP))};
localparam signed [31:0] SQRT_VPK_STEP_FP = 32'sd{int(round((VPK_MAX - VPK_MIN) / (LUT_SIZE - 1) * ONE_FP))};

// Softplus LUT: index = (inner - SOFTPLUS_MIN) / SOFTPLUS_STEP
localparam signed [31:0] SOFTPLUS_MIN_FP  = 32'sd{int(round(SOFTPLUS_MIN * ONE_FP))};
localparam signed [31:0] SOFTPLUS_MAX_FP  = 32'sd{int(round(SOFTPLUS_MAX * ONE_FP))};
localparam signed [31:0] SOFTPLUS_STEP_FP = 32'sd{int(round((SOFTPLUS_MAX - SOFTPLUS_MIN) / (LUT_SIZE - 1) * ONE_FP))};

// Power LUT: index = (Ed - ED_MIN) / ED_STEP
localparam signed [31:0] POWER_ED_MIN_FP  = 32'sd{int(round(ED_MIN * ONE_FP))};
localparam signed [31:0] POWER_ED_MAX_FP  = 32'sd{int(round(ED_MAX * ONE_FP))};
localparam signed [31:0] POWER_ED_STEP_FP = 32'sd{int(round((ED_MAX - ED_MIN) / (LUT_SIZE - 1) * ONE_FP))};
"""

params_path = os.path.join(DATA_DIR, "koren_1d_params.v")
with open(params_path, 'w') as f:
    f.write(params)
print(f"\n  Written: {params_path}")

# ============================================================================
# Summary
# ============================================================================
total_bytes = 4 * LUT_SIZE * 4  # 4 LUTs, 64 entries, 32-bit each
print(f"\nTotal 1D LUT memory: {total_bytes} bytes ({total_bytes/1024:.1f} KB)")
print(f"  vs 2D LUT memory: {128*128*2} bytes ({128*128*2/1024:.1f} KB) per table")
print(f"  Savings: {(1 - total_bytes/(128*128*2*3))*100:.1f}%")
