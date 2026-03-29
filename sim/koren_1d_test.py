"""
koren_1d_test.py -- Accuracy test for 1D-LUT-based Koren triode computation

Compares the hybrid 1D-LUT approach (with linear interpolation) against the
exact floating-point Koren equation across the full operating range.

Sweep: Vpk 0-300V, Vgk -4V to 0V
Target: <2% mean error at the operating point (Vpk~135V, Vgk~-0.9V)
"""

import numpy as np
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(SCRIPT_DIR, "..", "data")

# -- 12AX7 tube constants (fitted to RCA datasheet) --
MU    = 92.08
EX    = 1.29
KG1   = 1304.71
KP    = 561.08
KVB   = 15101.91

# -- LUT parameters (must match gen_1d_luts.py) --
LUT_SIZE = 64
FP_FRAC  = 16
ONE_FP   = 1 << FP_FRAC

VPK_MIN = 0.0
VPK_MAX = 300.0
SOFTPLUS_MIN = -6.0
SOFTPLUS_MAX = 6.0
ED_MIN = 0.0
ED_MAX = 0.6


# ============================================================================
# Exact Koren equation (floating point reference)
# ============================================================================
def koren_exact(vpk, vgk):
    """Exact Koren plate current in amps."""
    sqrt_val = np.sqrt(KVB + vpk**2 + 1e-9)
    inner = KP * (1.0/MU + vgk / sqrt_val)
    ed = (vpk / KP) * np.log1p(np.exp(np.clip(inner, -500, 500)))
    ip = (np.maximum(ed, 0.0) ** EX) / KG1
    return ip


# ============================================================================
# Build 1D LUT arrays (floating point, matching the hex files)
# ============================================================================
vpk_axis = np.linspace(VPK_MIN, VPK_MAX, LUT_SIZE)
sqrt_lut = np.sqrt(KVB + vpk_axis**2)

softplus_axis = np.linspace(SOFTPLUS_MIN, SOFTPLUS_MAX, LUT_SIZE)
softplus_lut = np.log1p(np.exp(softplus_axis))

ed_axis = np.linspace(ED_MIN, ED_MAX, LUT_SIZE)
power_lut = np.where(ed_axis > 0, ed_axis ** EX, 0.0)


def lut_interp(lut, x_axis, x):
    """Linear interpolation in a 1D LUT, with clamping at boundaries."""
    x = np.clip(x, x_axis[0], x_axis[-1])
    # Fractional index
    frac_idx = (x - x_axis[0]) / (x_axis[-1] - x_axis[0]) * (len(lut) - 1)
    idx_lo = np.floor(frac_idx).astype(int)
    idx_hi = np.minimum(idx_lo + 1, len(lut) - 1)
    frac = frac_idx - idx_lo
    return lut[idx_lo] * (1.0 - frac) + lut[idx_hi] * frac


def koren_1d_lut(vpk, vgk):
    """Koren plate current using 1D LUT decomposition with linear interpolation."""
    # Step 1: sqrt(kvb + vpk^2) from LUT
    sqrt_val = lut_interp(sqrt_lut, vpk_axis, vpk)

    # Step 2: inner = kp * (1/mu + vgk / sqrt_val)
    inner = KP * (1.0/MU + vgk / sqrt_val)

    # Step 3: softplus = log(1 + exp(inner)) from LUT
    # Piecewise: below SOFTPLUS_MIN -> ~0, above SOFTPLUS_MAX -> ~inner
    softplus = np.where(
        inner < SOFTPLUS_MIN,
        np.exp(inner),  # tiny, nearly 0
        np.where(
            inner > SOFTPLUS_MAX,
            inner,  # ≈ inner for large values
            lut_interp(softplus_lut, softplus_axis, inner)
        )
    )

    # Step 4: Ed = (vpk / kp) * softplus
    ed = (vpk / KP) * softplus
    ed = np.maximum(ed, 0.0)

    # Step 5: Ip = Ed^ex / kg1 from power LUT
    ip = np.where(
        ed <= 0,
        0.0,
        np.where(
            ed > ED_MAX,
            # Extrapolate: (ED_MAX^ex / kg1) * (ed/ED_MAX)^ex
            (ED_MAX ** EX / KG1) * (ed / ED_MAX) ** EX,
            lut_interp(power_lut, ed_axis, ed) / KG1
        )
    )
    return ip


# ============================================================================
# Also simulate the fixed-point (Q16.16 quantized) version
# ============================================================================
def quantize_fp(val, frac_bits=16):
    """Quantize a float to Q16.16 fixed point and back."""
    return np.round(val * (1 << frac_bits)) / (1 << frac_bits)

# Build quantized LUTs (matching hex file precision)
sqrt_lut_q = quantize_fp(sqrt_lut)
softplus_lut_q = quantize_fp(softplus_lut)
power_lut_q = quantize_fp(power_lut)

def koren_1d_lut_quantized(vpk, vgk):
    """Same as koren_1d_lut but using quantized LUT values."""
    sqrt_val = lut_interp(sqrt_lut_q, vpk_axis, vpk)
    inner = KP * (1.0/MU + vgk / sqrt_val)
    softplus = np.where(
        inner < SOFTPLUS_MIN,
        np.exp(inner),
        np.where(
            inner > SOFTPLUS_MAX,
            inner,
            lut_interp(softplus_lut_q, softplus_axis, inner)
        )
    )
    ed = (vpk / KP) * softplus
    ed = np.maximum(ed, 0.0)
    ip = np.where(
        ed <= 0, 0.0,
        np.where(
            ed > ED_MAX,
            (ED_MAX ** EX / KG1) * (ed / ED_MAX) ** EX,
            lut_interp(power_lut_q, ed_axis, ed) / KG1
        )
    )
    return ip


# ============================================================================
# Run accuracy sweep
# ============================================================================
print("=" * 70)
print("Koren 1D LUT Accuracy Test (12AX7)")
print("=" * 70)

N_VPK = 200
N_VGK = 100
vpk_sweep = np.linspace(1.0, 300.0, N_VPK)  # avoid Vpk=0 (Ip=0 trivially)
vgk_sweep = np.linspace(-4.0, 0.0, N_VGK)

Vpk_grid, Vgk_grid = np.meshgrid(vpk_sweep, vgk_sweep, indexing='ij')

ip_exact = koren_exact(Vpk_grid, Vgk_grid)
ip_1d = koren_1d_lut(Vpk_grid, Vgk_grid)
ip_1d_q = koren_1d_lut_quantized(Vpk_grid, Vgk_grid)

# Only compute relative error where Ip is significant (>1uA)
mask = ip_exact > 1e-6  # 1 uA threshold

if mask.sum() > 0:
    rel_err = np.abs(ip_1d[mask] - ip_exact[mask]) / ip_exact[mask] * 100
    rel_err_q = np.abs(ip_1d_q[mask] - ip_exact[mask]) / ip_exact[mask] * 100

    print(f"\n--- Float LUT (ideal interpolation) ---")
    print(f"  Points tested: {mask.sum()} / {Vpk_grid.size}")
    print(f"  Max relative error:  {rel_err.max():.4f}%")
    print(f"  Mean relative error: {rel_err.mean():.4f}%")
    print(f"  Median error:        {np.median(rel_err):.4f}%")
    print(f"  95th percentile:     {np.percentile(rel_err, 95):.4f}%")

    print(f"\n--- Quantized LUT (Q16.16 fixed-point) ---")
    print(f"  Max relative error:  {rel_err_q.max():.4f}%")
    print(f"  Mean relative error: {rel_err_q.mean():.4f}%")
    print(f"  Median error:        {np.median(rel_err_q):.4f}%")
    print(f"  95th percentile:     {np.percentile(rel_err_q, 95):.4f}%")

# -- Operating point accuracy --
print(f"\n--- Operating point (Vpk=135V, Vgk=-0.9V) ---")
vpk_op, vgk_op = 135.0, -0.9
ip_ex = koren_exact(vpk_op, vgk_op)
ip_lut_val = koren_1d_lut(np.array([vpk_op]), np.array([vgk_op]))[0]
ip_lut_q_val = koren_1d_lut_quantized(np.array([vpk_op]), np.array([vgk_op]))[0]
print(f"  Exact:     {ip_ex*1000:.6f} mA")
print(f"  1D LUT:    {ip_lut_val*1000:.6f} mA  (error: {abs(ip_lut_val-ip_ex)/ip_ex*100:.4f}%)")
print(f"  1D LUT Q:  {ip_lut_q_val*1000:.6f} mA  (error: {abs(ip_lut_q_val-ip_ex)/ip_ex*100:.4f}%)")

# -- Second operating point: hot bias --
print(f"\n--- Operating point (Vpk=100V, Vgk=-0.5V) ---")
vpk_op2, vgk_op2 = 100.0, -0.5
ip_ex2 = koren_exact(vpk_op2, vgk_op2)
ip_lut2 = koren_1d_lut(np.array([vpk_op2]), np.array([vgk_op2]))[0]
print(f"  Exact:     {ip_ex2*1000:.6f} mA")
print(f"  1D LUT:    {ip_lut2*1000:.6f} mA  (error: {abs(ip_lut2-ip_ex2)/ip_ex2*100:.4f}%)")

# -- Cutoff region --
print(f"\n--- Near cutoff (Vpk=200V, Vgk=-3.5V) ---")
vpk_op3, vgk_op3 = 200.0, -3.5
ip_ex3 = koren_exact(vpk_op3, vgk_op3)
ip_lut3 = koren_1d_lut(np.array([vpk_op3]), np.array([vgk_op3]))[0]
if ip_ex3 > 1e-9:
    print(f"  Exact:     {ip_ex3*1e6:.3f} uA")
    print(f"  1D LUT:    {ip_lut3*1e6:.3f} uA  (error: {abs(ip_lut3-ip_ex3)/ip_ex3*100:.4f}%)")
else:
    print(f"  Exact:     {ip_ex3*1e6:.3f} uA (near zero)")
    print(f"  1D LUT:    {ip_lut3*1e6:.3f} uA")

# -- Memory comparison --
print(f"\n--- Memory comparison ---")
mem_2d = 128 * 128 * 2 * 3  # 3 tables (ip, dip_dvgk, dip_dvpk) x 128x128 x 16-bit
mem_1d = 3 * LUT_SIZE * 4   # 3 tables x 64 entries x 32-bit
print(f"  2D LUT (current):  {mem_2d:,} bytes ({mem_2d/1024:.1f} KB) -- 3 tables of 128x128x16bit")
print(f"  1D LUT (proposed): {mem_1d:,} bytes ({mem_1d/1024:.1f} KB) -- 3 tables of {LUT_SIZE}x32bit")
print(f"  Reduction:         {(1-mem_1d/mem_2d)*100:.1f}%")
print(f"  BSRAM freed:       ~{mem_2d//1024} KB (fits in distributed SSRAM instead)")

# -- Pass/fail --
target_error = 2.0
if mask.sum() > 0 and rel_err.mean() < target_error:
    print(f"\n  PASS: Mean error {rel_err.mean():.4f}% < {target_error}% target")
else:
    print(f"\n  FAIL: Mean error {rel_err.mean():.4f}% >= {target_error}% target")
