#!/usr/bin/env python3
"""
Koren triode equation: piecewise polynomial approximation comparison.

Compares three approaches to replacing the 2D LUT (128KB BRAM):
  A) 2D Chebyshev polynomial fit (degree 4,6,8,10)
  B) Piecewise 1D LUT decomposition (3 small LUTs, 384 bytes)
  C) Direct CORDIC-like shift-add computation (no LUT)

Saves comparison plot to demos/koren_approx_comparison.png
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os
import sys
import time

# Fix Windows console encoding
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.join(SCRIPT_DIR, "..")
DEMOS_DIR = os.path.join(ROOT_DIR, "demos")
os.makedirs(DEMOS_DIR, exist_ok=True)

# ─── 12AX7 Koren constants (fitted to RCA datasheet) ────────────────────────
MU, EX, KG1, KP, KVB = 92.08, 1.29, 1304.71, 561.08, 15101.91

# ─── Operating ranges ────────────────────────────────────────────────────────
VPK_MIN, VPK_MAX = 0.0, 300.0
VGK_MIN, VGK_MAX = -4.0, 0.0

# Focus region for weighted error
VPK_FOCUS = (80.0, 200.0)
VGK_FOCUS = (-2.0, 0.0)


# ═══════════════════════════════════════════════════════════════════════════════
# Reference: exact Koren equation
# ═══════════════════════════════════════════════════════════════════════════════

def koren_ip_exact(vpk, vgk):
    """Exact Koren plate current (vectorized). Returns Ip in amps."""
    vpk = np.clip(vpk, 0.0, 500.0)
    vgk = np.clip(vgk, -10.0, 1.0)
    inner = KP * (1.0 / MU + vgk / np.sqrt(KVB + vpk**2 + 1e-9))
    Ed = (vpk / KP) * np.log1p(np.exp(np.clip(inner, -500, 500)))
    Ip = (np.maximum(Ed, 0.0) ** EX) / KG1
    return Ip


def make_grid(n=256):
    """Create evaluation grid."""
    vpk = np.linspace(VPK_MIN, VPK_MAX, n)
    vgk = np.linspace(VGK_MIN, VGK_MAX, n)
    Vpk, Vgk = np.meshgrid(vpk, vgk, indexing='ij')
    return vpk, vgk, Vpk, Vgk


def compute_errors(Ip_exact, Ip_approx, Vpk, Vgk):
    """Compute error stats over full range and focus region."""
    abs_err = np.abs(Ip_exact - Ip_approx)
    # Relative error where Ip > 0.01 mA (avoid div-by-zero in cutoff)
    mask_nonzero = Ip_exact > 1e-5
    rel_err = np.zeros_like(abs_err)
    rel_err[mask_nonzero] = abs_err[mask_nonzero] / Ip_exact[mask_nonzero] * 100.0

    # Focus region mask
    focus = ((Vpk >= VPK_FOCUS[0]) & (Vpk <= VPK_FOCUS[1]) &
             (Vgk >= VGK_FOCUS[0]) & (Vgk <= VGK_FOCUS[1]))
    focus_nonzero = focus & mask_nonzero

    results = {
        'max_abs_mA': np.max(abs_err) * 1000,
        'mean_abs_mA': np.mean(abs_err) * 1000,
        'max_rel_pct': np.max(rel_err[mask_nonzero]) if mask_nonzero.any() else 0,
        'mean_rel_pct': np.mean(rel_err[mask_nonzero]) if mask_nonzero.any() else 0,
        'focus_max_rel_pct': np.max(rel_err[focus_nonzero]) if focus_nonzero.any() else 0,
        'focus_mean_rel_pct': np.mean(rel_err[focus_nonzero]) if focus_nonzero.any() else 0,
    }
    return results


# ═══════════════════════════════════════════════════════════════════════════════
# Approach A: 2D Chebyshev polynomial fit
# ═══════════════════════════════════════════════════════════════════════════════

def fit_chebyshev_2d(vpk_1d, vgk_1d, Ip_grid, deg):
    """
    Fit Ip(vpk, vgk) as a 2D Chebyshev polynomial.
    Uses least-squares fit on the flattened grid.
    Returns coefficients and evaluation function.
    """
    from numpy.polynomial import chebyshev as C

    # Map to [-1, 1]
    vpk_norm = 2.0 * (vpk_1d - VPK_MIN) / (VPK_MAX - VPK_MIN) - 1.0
    vgk_norm = 2.0 * (vgk_1d - VGK_MIN) / (VGK_MAX - VGK_MIN) - 1.0

    Vn, Gn = np.meshgrid(vpk_norm, vgk_norm, indexing='ij')

    # Build Vandermonde-like matrix for 2D Chebyshev
    n_terms = (deg + 1) ** 2
    A = np.zeros((Vn.size, n_terms))
    col = 0
    for i in range(deg + 1):
        Ti_v = C.chebval(Vn.ravel(), np.eye(deg + 1)[i])
        for j in range(deg + 1):
            Tj_g = C.chebval(Gn.ravel(), np.eye(deg + 1)[j])
            A[:, col] = Ti_v * Tj_g
            col += 1

    # Least-squares fit
    coeffs, _, _, _ = np.linalg.lstsq(A, Ip_grid.ravel(), rcond=None)

    def eval_cheb(vpk_2d, vgk_2d):
        vn = 2.0 * (vpk_2d - VPK_MIN) / (VPK_MAX - VPK_MIN) - 1.0
        gn = 2.0 * (vgk_2d - VGK_MIN) / (VGK_MAX - VGK_MIN) - 1.0
        result = np.zeros_like(vn, dtype=float)
        col = 0
        for i in range(deg + 1):
            Ti = C.chebval(vn, np.eye(deg + 1)[i])
            for j in range(deg + 1):
                Tj = C.chebval(gn, np.eye(deg + 1)[j])
                result += coeffs[col] * Ti * Tj
                col += 1
        return np.maximum(result, 0.0)

    return coeffs, eval_cheb


# ═══════════════════════════════════════════════════════════════════════════════
# Approach B: Piecewise 1D LUT decomposition
# ═══════════════════════════════════════════════════════════════════════════════

def build_1d_luts(n_entries=64):
    """
    Break Koren into 3 one-dimensional LUTs with linear interpolation:
      1. sqrt_lut: sqrt(KVB + vpk^2), indexed by vpk [0, 300]
      2. softplus_lut: log1p(exp(x)), indexed by x [-8, 8]
      3. power_lut: x^EX, indexed by x [0, Ed_max]
    """
    # LUT 1: sqrt(KVB + vpk^2)
    vpk_lut = np.linspace(VPK_MIN, VPK_MAX, n_entries)
    sqrt_vals = np.sqrt(KVB + vpk_lut**2)

    # LUT 2: softplus = log1p(exp(x))
    # Determine range of 'inner' from operating region
    # inner = KP * (1/MU + vgk / sqrt_val)
    # For vpk=0..300, vgk=-4..0: inner ranges about -20 to +8
    sp_min, sp_max = -8.0, 8.0
    sp_x = np.linspace(sp_min, sp_max, n_entries)
    sp_vals = np.log1p(np.exp(np.clip(sp_x, -500, 500)))

    # LUT 3: x^EX (power function)
    # Ed = (vpk/KP) * softplus; max Ed ~ (300/561)*8 ~ 4.3
    ed_max = 5.0
    ed_x = np.linspace(0.0, ed_max, n_entries)
    pow_vals = ed_x ** EX

    luts = {
        'sqrt': (vpk_lut, sqrt_vals, VPK_MIN, VPK_MAX),
        'softplus': (sp_x, sp_vals, sp_min, sp_max),
        'power': (ed_x, pow_vals, 0.0, ed_max),
    }
    return luts


def lut_interp(x, lut_x, lut_y, x_min, x_max):
    """Piecewise linear interpolation with clamping."""
    n = len(lut_x)
    # Normalize to [0, n-1]
    t = (x - x_min) / (x_max - x_min) * (n - 1)
    t = np.clip(t, 0, n - 2)
    idx = np.floor(t).astype(int)
    frac = t - idx
    return lut_y[idx] * (1.0 - frac) + lut_y[np.minimum(idx + 1, n - 1)] * frac


def eval_1d_lut_koren(Vpk, Vgk, luts):
    """Evaluate Koren using 3 small 1D LUTs."""
    vpk = np.clip(Vpk, 0.0, 300.0)
    vgk = np.clip(Vgk, -4.0, 0.0)

    # Step 1: sqrt(KVB + vpk^2) from LUT
    sqrt_val = lut_interp(vpk, *luts['sqrt'])

    # Step 2: inner = KP * (1/MU + vgk / sqrt_val) — direct arithmetic
    inner = KP * (1.0 / MU + vgk / sqrt_val)

    # Step 3: softplus = log1p(exp(inner)) from LUT
    softplus = lut_interp(inner, *luts['softplus'])

    # Step 4: Ed = (vpk / KP) * softplus — multiply
    Ed = (vpk / KP) * softplus
    Ed = np.maximum(Ed, 0.0)

    # Step 5: power = Ed^EX from LUT
    power = lut_interp(Ed, *luts['power'])

    # Step 6: Ip = power / KG1 — multiply
    Ip = power / KG1
    return np.maximum(Ip, 0.0)


# ═══════════════════════════════════════════════════════════════════════════════
# Approach C: CORDIC-like direct computation (fixed-point emulation)
# ═══════════════════════════════════════════════════════════════════════════════

def fixed_sqrt_newton(x, n_iter=6):
    """Newton's method sqrt in fixed point (emulated in float)."""
    if np.isscalar(x):
        if x <= 0:
            return 0.0
        # Initial guess: x/2 or better
        r = x / 2.0
        for _ in range(n_iter):
            r = 0.5 * (r + x / r)
        return r
    result = np.zeros_like(x, dtype=float)
    mask = x > 0
    r = x[mask] / 2.0
    for _ in range(n_iter):
        r = 0.5 * (r + x[mask] / r)
    result[mask] = r
    return result


def fixed_exp_approx(x, n_terms=8):
    """
    Polynomial approximation of exp(x) for |x| < 8.
    Uses truncated Taylor series — maps to shift+add in hardware.
    """
    x = np.clip(x, -8.0, 8.0)
    result = np.ones_like(x, dtype=float)
    term = np.ones_like(x, dtype=float)
    for k in range(1, n_terms + 1):
        term = term * x / k
        result = result + term
    return np.maximum(result, 0.0)


def fixed_log1p_exp(x, n_terms=8):
    """
    Compute log1p(exp(x)) using piecewise approach:
      x >> 0: ~ x
      x << 0: ~ exp(x)
      |x| < 4: polynomial approximation
    """
    result = np.zeros_like(x, dtype=float)

    # Region 1: x > 4 => log1p(exp(x)) ~ x
    hi = x > 4.0
    result[hi] = x[hi]

    # Region 2: x < -4 => log1p(exp(x)) ~ exp(x)
    lo = x < -4.0
    result[lo] = fixed_exp_approx(x[lo], n_terms)

    # Region 3: |x| <= 4 => direct computation
    mid = ~hi & ~lo
    result[mid] = np.log1p(fixed_exp_approx(x[mid], n_terms))

    return result


def fixed_power(x, ex, n_terms=8):
    """
    Compute x^ex using exp(ex * ln(x)).
    For hardware: ln via series, exp via series.
    """
    result = np.zeros_like(x, dtype=float)
    mask = x > 1e-10

    # ln(x) via Newton: ln(x) = 2 * atanh((x-1)/(x+1))
    # Use range reduction: x = m * 2^k, ln(x) = k*ln2 + ln(m)
    # Simpler: use log Taylor series around 1
    # For x in [0, 5], scale to [0.5, 2] range
    xm = x[mask]
    ln_x = np.log(xm)  # In HW this would be CORDIC or LUT+interp
    result[mask] = fixed_exp_approx(ex * ln_x, n_terms)

    return result


def eval_cordic_koren(Vpk, Vgk):
    """Evaluate Koren using CORDIC-like iterative algorithms."""
    vpk = np.clip(Vpk, 0.0, 300.0)
    vgk = np.clip(Vgk, -4.0, 0.0)

    # sqrt via Newton (6 iterations)
    sqrt_val = fixed_sqrt_newton(KVB + vpk**2, n_iter=6)

    # inner = KP * (1/MU + vgk/sqrt_val)
    inner = KP * (1.0 / MU + vgk / sqrt_val)

    # softplus = log1p(exp(inner))
    softplus = fixed_log1p_exp(inner, n_terms=8)

    # Ed = (vpk / KP) * softplus
    Ed = (vpk / KP) * softplus
    Ed = np.maximum(Ed, 0.0)

    # power = Ed^EX
    power = fixed_power(Ed, EX, n_terms=8)

    # Ip = power / KG1
    Ip = power / KG1
    return np.maximum(Ip, 0.0)


# ═══════════════════════════════════════════════════════════════════════════════
# Main comparison
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    print("=" * 70)
    print("Koren Triode Approximation Comparison (12AX7)")
    print("=" * 70)

    # Generate reference grid
    N = 256
    vpk_1d, vgk_1d, Vpk, Vgk = make_grid(N)
    Ip_exact = koren_ip_exact(Vpk, Vgk)

    print(f"\nReference grid: {N}x{N} = {N*N} points")
    print(f"Vpk: [{VPK_MIN}, {VPK_MAX}] V, Vgk: [{VGK_MIN}, {VGK_MAX}] V")
    print(f"Max Ip: {np.max(Ip_exact)*1000:.3f} mA")
    print()

    all_results = []

    # ─── Approach A: Chebyshev 2D ────────────────────────────────────────────
    print("─── Approach A: 2D Chebyshev Polynomial ───")
    cheb_results = {}
    for deg in [4, 6, 8, 10]:
        t0 = time.time()
        coeffs, eval_fn = fit_chebyshev_2d(vpk_1d, vgk_1d, Ip_exact, deg)
        Ip_cheb = eval_fn(Vpk, Vgk)
        dt = time.time() - t0

        errs = compute_errors(Ip_exact, Ip_cheb, Vpk, Vgk)
        n_coeffs = (deg + 1) ** 2
        n_mults = n_coeffs * 4  # rough: each term needs ~4 multiplies (Chebyshev recurrence + coeff)
        mem_bytes = n_coeffs * 4  # 32-bit coefficients

        cheb_results[deg] = {'Ip': Ip_cheb, 'errs': errs, 'eval_fn': eval_fn}
        label = f"Cheb-{deg}"
        all_results.append({
            'name': label,
            'errs': errs,
            'n_mults': n_mults,
            'mem_bytes': mem_bytes,
            'Ip': Ip_cheb,
        })

        print(f"  Degree {deg}: {n_coeffs} coeffs, {mem_bytes} bytes, ~{n_mults} muls")
        print(f"    Max err: {errs['max_abs_mA']:.4f} mA ({errs['max_rel_pct']:.2f}%)")
        print(f"    Mean err: {errs['mean_abs_mA']:.4f} mA ({errs['mean_rel_pct']:.2f}%)")
        print(f"    Focus region: max {errs['focus_max_rel_pct']:.2f}%, mean {errs['focus_mean_rel_pct']:.2f}%")
        print(f"    Time: {dt:.2f}s")

    # ─── Approach B: 1D LUT decomposition ────────────────────────────────────
    print("\n─── Approach B: Piecewise 1D LUT Decomposition ───")
    for n_entries in [32, 64, 128, 256]:
        luts = build_1d_luts(n_entries)
        t0 = time.time()
        Ip_lut = eval_1d_lut_koren(Vpk, Vgk, luts)
        dt = time.time() - t0

        errs = compute_errors(Ip_exact, Ip_lut, Vpk, Vgk)
        mem_bytes = 3 * n_entries * 2  # 3 LUTs, 16-bit each
        n_mults = 6  # 3 interp (each 2 muls) + 2 arithmetic muls + 1 div

        label = f"1D-LUT-{n_entries}"
        all_results.append({
            'name': label,
            'errs': errs,
            'n_mults': n_mults,
            'mem_bytes': mem_bytes,
            'Ip': Ip_lut,
        })

        print(f"  {n_entries} entries/LUT: {mem_bytes} bytes, ~{n_mults} muls")
        print(f"    Max err: {errs['max_abs_mA']:.4f} mA ({errs['max_rel_pct']:.2f}%)")
        print(f"    Mean err: {errs['mean_abs_mA']:.4f} mA ({errs['mean_rel_pct']:.2f}%)")
        print(f"    Focus region: max {errs['focus_max_rel_pct']:.2f}%, mean {errs['focus_mean_rel_pct']:.2f}%")
        print(f"    Time: {dt:.3f}s")

    # ─── Approach C: CORDIC-like direct computation ──────────────────────────
    print("\n─── Approach C: CORDIC-like Direct Computation ───")
    t0 = time.time()
    Ip_cordic = eval_cordic_koren(Vpk, Vgk)
    dt = time.time() - t0

    errs = compute_errors(Ip_exact, Ip_cordic, Vpk, Vgk)
    # Newton sqrt: 6 iters * 2 ops = 12; exp: 8 terms * 2 = 16; log: 8 * 2 = 16; etc
    n_mults = 50  # rough estimate of multiply+divide operations
    mem_bytes = 0  # no LUT memory

    all_results.append({
        'name': 'CORDIC',
        'errs': errs,
        'n_mults': n_mults,
        'mem_bytes': mem_bytes,
        'Ip': Ip_cordic,
    })

    print(f"  No LUT, ~{n_mults} muls, {mem_bytes} bytes")
    print(f"    Max err: {errs['max_abs_mA']:.4f} mA ({errs['max_rel_pct']:.2f}%)")
    print(f"    Mean err: {errs['mean_abs_mA']:.4f} mA ({errs['mean_rel_pct']:.2f}%)")
    print(f"    Focus region: max {errs['focus_max_rel_pct']:.2f}%, mean {errs['focus_mean_rel_pct']:.2f}%")
    print(f"    Time: {dt:.3f}s")

    # ─── Reference: current 2D LUT (128x128) ────────────────────────────────
    print("\n─── Reference: Current 2D LUT (128x128, bilinear interp) ───")
    # Simulate what the FPGA does: 128x128 grid with bilinear interpolation
    n_2d = 128
    vpk_2d = np.linspace(VPK_MIN, VPK_MAX, n_2d)
    vgk_2d = np.linspace(VGK_MIN, VGK_MAX, n_2d)
    V2d, G2d = np.meshgrid(vpk_2d, vgk_2d, indexing='ij')
    Ip_2d_grid = koren_ip_exact(V2d, G2d)

    # Bilinear interpolation on the fine grid
    from scipy.interpolate import RegularGridInterpolator
    interp_2d = RegularGridInterpolator((vpk_2d, vgk_2d), Ip_2d_grid, method='linear',
                                         bounds_error=False, fill_value=0.0)
    pts = np.column_stack([Vpk.ravel(), Vgk.ravel()])
    Ip_2d_interp = interp_2d(pts).reshape(Vpk.shape)

    errs_2d = compute_errors(Ip_exact, Ip_2d_interp, Vpk, Vgk)
    mem_2d = n_2d * n_2d * 2  # 16-bit entries
    all_results.append({
        'name': f'2D-LUT-{n_2d}',
        'errs': errs_2d,
        'n_mults': 4,  # bilinear interp: 4 muls
        'mem_bytes': mem_2d,
        'Ip': Ip_2d_interp,
    })
    print(f"  {n_2d}x{n_2d} = {mem_2d} bytes, ~4 muls (bilinear)")
    print(f"    Max err: {errs_2d['max_abs_mA']:.4f} mA ({errs_2d['max_rel_pct']:.2f}%)")
    print(f"    Mean err: {errs_2d['mean_abs_mA']:.4f} mA ({errs_2d['mean_rel_pct']:.2f}%)")
    print(f"    Focus region: max {errs_2d['focus_max_rel_pct']:.2f}%, mean {errs_2d['focus_mean_rel_pct']:.2f}%")

    # ═════════════════════════════════════════════════════════════════════════
    # Summary table
    # ═════════════════════════════════════════════════════════════════════════
    print("\n" + "=" * 90)
    print(f"{'Method':<16} {'MaxErr(mA)':>10} {'MeanErr(mA)':>12} {'FocusMax%':>10} {'FocusMean%':>11} {'Muls':>6} {'Memory':>10}")
    print("-" * 90)
    for r in all_results:
        e = r['errs']
        mem_str = f"{r['mem_bytes']} B" if r['mem_bytes'] < 1024 else f"{r['mem_bytes']//1024} KB"
        print(f"{r['name']:<16} {e['max_abs_mA']:>10.4f} {e['mean_abs_mA']:>12.4f} {e['focus_max_rel_pct']:>10.2f} {e['focus_mean_rel_pct']:>11.2f} {r['n_mults']:>6} {mem_str:>10}")
    print("=" * 90)

    # ═════════════════════════════════════════════════════════════════════════
    # Plot comparison
    # ═════════════════════════════════════════════════════════════════════════
    fig, axes = plt.subplots(3, 4, figsize=(20, 14))
    fig.suptitle('Koren Triode Approximation Comparison (12AX7)', fontsize=16, fontweight='bold')

    # Row 0: Reference + best of each approach
    # Pick best Chebyshev (deg 10), best 1D-LUT (128), CORDIC
    plot_items = [
        ('Exact Koren', Ip_exact),
        ('Chebyshev deg=10', [r for r in all_results if r['name'] == 'Cheb-10'][0]['Ip']),
        ('1D-LUT-128', [r for r in all_results if r['name'] == '1D-LUT-128'][0]['Ip']),
        ('CORDIC', [r for r in all_results if r['name'] == 'CORDIC'][0]['Ip']),
    ]

    vgk_slice_idx = N * 3 // 4  # Vgk ~ -1V
    vpk_plot = vpk_1d

    for col, (label, Ip) in enumerate(plot_items):
        ax = axes[0, col]
        im = ax.pcolormesh(vpk_1d, vgk_1d, (Ip * 1000).T, shading='auto', cmap='viridis')
        ax.set_title(label, fontsize=11)
        ax.set_xlabel('Vpk (V)')
        ax.set_ylabel('Vgk (V)')
        plt.colorbar(im, ax=ax, label='Ip (mA)')

    # Row 1: Error heatmaps
    error_items = [
        ('2D-LUT-128 err', Ip_2d_interp),
        ('Cheb-10 err', [r for r in all_results if r['name'] == 'Cheb-10'][0]['Ip']),
        ('1D-LUT-128 err', [r for r in all_results if r['name'] == '1D-LUT-128'][0]['Ip']),
        ('CORDIC err', [r for r in all_results if r['name'] == 'CORDIC'][0]['Ip']),
    ]

    for col, (label, Ip) in enumerate(error_items):
        ax = axes[1, col]
        err = np.abs(Ip_exact - Ip) * 1000  # mA
        im = ax.pcolormesh(vpk_1d, vgk_1d, err.T, shading='auto', cmap='hot_r')
        ax.set_title(label, fontsize=11)
        ax.set_xlabel('Vpk (V)')
        ax.set_ylabel('Vgk (V)')
        plt.colorbar(im, ax=ax, label='|Error| (mA)')

    # Row 2: 1D slices and summary bar chart
    # Slice at Vgk = -1V
    vgk_idx = np.argmin(np.abs(vgk_1d - (-1.0)))
    ax = axes[2, 0]
    ax.plot(vpk_1d, Ip_exact[:, vgk_idx] * 1000, 'k-', linewidth=2, label='Exact')
    for r in all_results:
        if r['name'] in ['Cheb-10', '1D-LUT-128', 'CORDIC', '2D-LUT-128']:
            ax.plot(vpk_1d, r['Ip'][:, vgk_idx] * 1000, '--', label=r['name'], alpha=0.8)
    ax.set_title('Ip vs Vpk at Vgk = -1V', fontsize=11)
    ax.set_xlabel('Vpk (V)')
    ax.set_ylabel('Ip (mA)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Slice at Vpk = 150V
    vpk_idx = np.argmin(np.abs(vpk_1d - 150.0))
    ax = axes[2, 1]
    ax.plot(vgk_1d, Ip_exact[vpk_idx, :] * 1000, 'k-', linewidth=2, label='Exact')
    for r in all_results:
        if r['name'] in ['Cheb-10', '1D-LUT-128', 'CORDIC', '2D-LUT-128']:
            ax.plot(vgk_1d, r['Ip'][vpk_idx, :] * 1000, '--', label=r['name'], alpha=0.8)
    ax.set_title('Ip vs Vgk at Vpk = 150V', fontsize=11)
    ax.set_xlabel('Vgk (V)')
    ax.set_ylabel('Ip (mA)')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Bar chart: focus region mean error
    ax = axes[2, 2]
    names = [r['name'] for r in all_results]
    focus_errs = [r['errs']['focus_mean_rel_pct'] for r in all_results]
    colors = ['#2196F3'] * 4 + ['#4CAF50'] * 4 + ['#FF9800'] + ['#9E9E9E']
    bars = ax.barh(names, focus_errs, color=colors[:len(names)])
    ax.set_xlabel('Focus Region Mean Error (%)')
    ax.set_title('Mean Relative Error (operating region)', fontsize=11)
    ax.axvline(x=1.0, color='red', linestyle='--', alpha=0.5, label='1% target')
    ax.legend(fontsize=8)

    # Bar chart: memory usage
    ax = axes[2, 3]
    mem_vals = [r['mem_bytes'] for r in all_results]
    bars = ax.barh(names, mem_vals, color=colors[:len(names)])
    ax.set_xlabel('Memory (bytes)')
    ax.set_title('Memory Usage', fontsize=11)
    ax.axvline(x=32768, color='red', linestyle='--', alpha=0.5, label='32KB (2D-LUT)')
    ax.legend(fontsize=8)
    ax.set_xscale('log')

    plt.tight_layout()
    outpath = os.path.join(DEMOS_DIR, 'koren_approx_comparison.png')
    plt.savefig(outpath, dpi=150, bbox_inches='tight')
    print(f"\nPlot saved: {outpath}")
    plt.close()

    # ═════════════════════════════════════════════════════════════════════════
    # Determine best approach
    # ═════════════════════════════════════════════════════════════════════════
    print("\n─── Recommendation ───")
    # Filter to methods with < 1% focus mean error
    good = [r for r in all_results if r['errs']['focus_mean_rel_pct'] < 1.0]
    if good:
        # Sort by memory, then muls
        good.sort(key=lambda r: (r['mem_bytes'], r['n_mults']))
        best = good[0]
        print(f"Best approach: {best['name']}")
        print(f"  Focus mean error: {best['errs']['focus_mean_rel_pct']:.3f}%")
        print(f"  Memory: {best['mem_bytes']} bytes")
        print(f"  Multiplies: {best['n_mults']}")
    else:
        print("No approach achieves < 1% in focus region. Showing best available:")
        all_results.sort(key=lambda r: r['errs']['focus_mean_rel_pct'])
        best = all_results[0]
        print(f"  {best['name']}: {best['errs']['focus_mean_rel_pct']:.3f}%")

    return all_results


if __name__ == '__main__':
    main()
