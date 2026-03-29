#!/usr/bin/env python3
"""
koren_direct.py — Piecewise 1D LUT implementation of the Koren triode equation.

Replaces the 128x128 2D LUT (32KB BRAM) with 3 small 1D LUTs (768 bytes total)
plus a handful of multiplies/adds. Matches the exact Koren equation to within
0.08% mean error in the operating region (Vpk 80-200V, Vgk -2 to 0V).

Decomposition:
  1. sqrt_lut[128]: sqrt(KVB + vpk^2),      indexed by Vpk   [0, 300]
  2. softplus_lut[128]: log1p(exp(x)),       indexed by inner [-8, 8]
  3. power_lut[128]: x^EX,                   indexed by Ed    [0, 5]

Arithmetic between LUTs:
  inner   = KP * (1/MU + vgk / sqrt_lut(vpk))     -- 1 div, 1 add, 1 mul
  Ed      = (vpk / KP) * softplus_lut(inner)       -- 1 div, 1 mul
  Ip      = power_lut(Ed) / KG1                    -- 1 div

Total: 3 LUT lookups + 6 multiply/divide ops = ~10 clock cycles on FPGA
Memory: 3 * 128 * 16-bit = 768 bytes (vs 32,768 bytes for 2D LUT)

Usage:
    from koren_direct import KorenDirect
    kd = KorenDirect()              # builds LUTs on init
    ip = kd.plate_current(vpk, vgk) # scalar or numpy array
"""

import numpy as np
import os
import sys

# ─── 12AX7 Koren constants (fitted to RCA datasheet) ────────────────────────
DEFAULT_PARAMS = {
    'mu': 92.08,
    'ex': 1.29,
    'kg1': 1304.71,
    'kp': 561.08,
    'kvb': 15101.91,
}

# Operating ranges for 12AX7 preamp
DEFAULT_RANGES = {
    'vpk_min': 0.0, 'vpk_max': 300.0,
    'vgk_min': -4.0, 'vgk_max': 0.0,
}


def koren_ip_exact(vpk, vgk, mu, ex, kg1, kp, kvb):
    """Reference: exact Koren plate current (vectorized)."""
    vpk = np.asarray(vpk, dtype=float)
    vgk = np.asarray(vgk, dtype=float)
    vpk = np.clip(vpk, 0.0, 500.0)
    vgk = np.clip(vgk, -10.0, 1.0)
    inner = kp * (1.0 / mu + vgk / np.sqrt(kvb + vpk**2 + 1e-9))
    Ed = (vpk / kp) * np.log1p(np.exp(np.clip(inner, -500, 500)))
    Ip = (np.maximum(Ed, 0.0) ** ex) / kg1
    return Ip


class KorenDirect:
    """
    Piecewise 1D LUT approximation of the Koren triode equation.

    Breaks the transcendental Koren equation into 3 one-dimensional
    lookup tables with linear interpolation, connected by simple
    multiply/add arithmetic.

    Parameters
    ----------
    n_entries : int
        Number of entries per LUT (default 128). More = more accurate.
        128 entries gives < 0.1% mean error in the operating region.
    params : dict
        Koren tube constants (mu, ex, kg1, kp, kvb).
    vpk_range : tuple
        (min, max) plate-cathode voltage range.
    vgk_range : tuple
        (min, max) grid-cathode voltage range.
    """

    def __init__(self, n_entries=128, params=None, vpk_range=None, vgk_range=None):
        p = params or DEFAULT_PARAMS
        self.mu = p['mu']
        self.ex = p['ex']
        self.kg1 = p['kg1']
        self.kp = p['kp']
        self.kvb = p['kvb']
        self.n = n_entries

        self.vpk_min, self.vpk_max = vpk_range or (0.0, 300.0)
        self.vgk_min, self.vgk_max = vgk_range or (-4.0, 0.0)

        # Determine softplus input range from operating region corners
        sqrt_at_min = np.sqrt(self.kvb + self.vpk_min**2)
        sqrt_at_max = np.sqrt(self.kvb + self.vpk_max**2)
        inner_min = self.kp * (1.0 / self.mu + self.vgk_min / sqrt_at_min)
        inner_max = self.kp * (1.0 / self.mu + self.vgk_max / sqrt_at_max)
        # Add margin
        self.sp_min = min(inner_min, -8.0) - 1.0
        self.sp_max = max(inner_max, 8.0) + 1.0

        # Determine Ed range
        sp_val_max = np.log1p(np.exp(np.clip(self.sp_max, -500, 500)))
        self.ed_max = (self.vpk_max / self.kp) * sp_val_max * 1.1  # 10% margin

        self._build_luts()

    def _build_luts(self):
        """Pre-compute the three 1D lookup tables."""
        n = self.n

        # LUT 1: sqrt(KVB + vpk^2) indexed by vpk
        self.sqrt_x = np.linspace(self.vpk_min, self.vpk_max, n)
        self.sqrt_y = np.sqrt(self.kvb + self.sqrt_x**2)

        # LUT 2: log1p(exp(x)) indexed by inner
        self.sp_x = np.linspace(self.sp_min, self.sp_max, n)
        self.sp_y = np.log1p(np.exp(np.clip(self.sp_x, -500, 500)))

        # LUT 3: x^EX indexed by Ed
        self.pow_x = np.linspace(0.0, self.ed_max, n)
        self.pow_y = self.pow_x ** self.ex

    def _lut_interp(self, x, x_min, x_max, lut_y):
        """Piecewise linear interpolation with clamping (vectorized)."""
        n = len(lut_y)
        t = (x - x_min) / (x_max - x_min) * (n - 1)
        t = np.clip(t, 0.0, n - 1.001)
        idx = np.floor(t).astype(int)
        frac = t - idx
        return lut_y[idx] * (1.0 - frac) + lut_y[np.minimum(idx + 1, n - 1)] * frac

    def plate_current(self, vpk, vgk):
        """
        Compute plate current Ip using 1D LUT decomposition.

        Parameters
        ----------
        vpk : float or array
            Plate-cathode voltage (V). Clamped to [vpk_min, vpk_max].
        vgk : float or array
            Grid-cathode voltage (V). Clamped to [vgk_min, vgk_max].

        Returns
        -------
        Ip : float or array
            Plate current in amps.
        """
        scalar = np.isscalar(vpk) and np.isscalar(vgk)
        vpk = np.atleast_1d(np.asarray(vpk, dtype=float))
        vgk = np.atleast_1d(np.asarray(vgk, dtype=float))

        vpk = np.clip(vpk, self.vpk_min, self.vpk_max)
        vgk = np.clip(vgk, self.vgk_min, self.vgk_max)

        # Step 1: sqrt(KVB + vpk^2) from LUT
        sqrt_val = self._lut_interp(vpk, self.vpk_min, self.vpk_max, self.sqrt_y)

        # Step 2: inner = KP * (1/MU + vgk / sqrt_val)
        inner = self.kp * (1.0 / self.mu + vgk / sqrt_val)

        # Step 3: softplus = log1p(exp(inner)) from LUT
        softplus = self._lut_interp(inner, self.sp_min, self.sp_max, self.sp_y)

        # Step 4: Ed = (vpk / KP) * softplus
        Ed = (vpk / self.kp) * softplus
        Ed = np.maximum(Ed, 0.0)

        # Step 5: Ed^EX from LUT
        power = self._lut_interp(Ed, 0.0, self.ed_max, self.pow_y)

        # Step 6: Ip = power / KG1
        Ip = np.maximum(power / self.kg1, 0.0)

        if scalar:
            return float(Ip[0])
        return Ip

    def memory_bytes(self):
        """Total LUT memory in bytes (16-bit entries)."""
        return 3 * self.n * 2

    def export_hex(self, output_dir, q_frac=15, suffix=''):
        """
        Export LUTs as .hex files for Verilog $readmemh.

        Parameters
        ----------
        output_dir : str
            Directory to write hex files.
        q_frac : int
            Fractional bits for fixed-point encoding.
        suffix : str
            Filename suffix (e.g., '_12ax7').
        """
        os.makedirs(output_dir, exist_ok=True)

        def to_hex16(vals, scale):
            """Convert float array to 16-bit unsigned hex strings."""
            ints = np.clip(np.round(vals * scale), 0, 65535).astype(int)
            return [f"{v:04x}" for v in ints]

        # Determine appropriate scales
        sqrt_scale = 2**q_frac / np.max(self.sqrt_y)
        sp_scale = 2**q_frac / np.max(self.sp_y)
        pow_scale = 2**q_frac / np.max(self.pow_y)

        for name, vals, scale in [
            ('sqrt_lut', self.sqrt_y, sqrt_scale),
            ('softplus_lut', self.sp_y, sp_scale),
            ('power_lut', self.pow_y, pow_scale),
        ]:
            path = os.path.join(output_dir, f"{name}{suffix}.hex")
            lines = to_hex16(vals, scale)
            with open(path, 'w') as f:
                f.write('\n'.join(lines) + '\n')

        # Also write a params file for Verilog
        params_path = os.path.join(output_dir, f"koren_1d_params{suffix}.v")
        with open(params_path, 'w') as f:
            f.write(f"// Auto-generated by koren_direct.py\n")
            f.write(f"// 1D LUT parameters for Koren triode approximation\n")
            f.write(f"parameter N_ENTRIES = {self.n};\n")
            f.write(f"parameter SQRT_SCALE = {int(sqrt_scale)};\n")
            f.write(f"parameter SP_SCALE = {int(sp_scale)};\n")
            f.write(f"parameter POW_SCALE = {int(pow_scale)};\n")
            f.write(f"parameter VPK_MIN = {int(self.vpk_min)};\n")
            f.write(f"parameter VPK_MAX = {int(self.vpk_max)};\n")
            f.write(f"parameter SP_MIN_Q16 = {int(self.sp_min * 65536)};\n")
            f.write(f"parameter SP_MAX_Q16 = {int(self.sp_max * 65536)};\n")
            f.write(f"parameter ED_MAX_Q16 = {int(self.ed_max * 65536)};\n")


# ═══════════════════════════════════════════════════════════════════════════════
# Self-test
# ═══════════════════════════════════════════════════════════════════════════════

def self_test():
    """Validate 1D LUT approach against exact Koren."""
    if sys.platform == 'win32':
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')

    print("KorenDirect self-test")
    print("=" * 60)

    kd = KorenDirect(n_entries=128)
    print(f"LUT memory: {kd.memory_bytes()} bytes ({kd.memory_bytes()//2} x 16-bit entries)")

    # Test grid
    N = 256
    vpk = np.linspace(0.0, 300.0, N)
    vgk = np.linspace(-4.0, 0.0, N)
    Vpk, Vgk = np.meshgrid(vpk, vgk, indexing='ij')

    Ip_exact = koren_ip_exact(Vpk, Vgk, **DEFAULT_PARAMS)
    Ip_approx = kd.plate_current(Vpk, Vgk)

    abs_err = np.abs(Ip_exact - Ip_approx)
    mask = Ip_exact > 1e-5
    rel_err = np.zeros_like(abs_err)
    rel_err[mask] = abs_err[mask] / Ip_exact[mask] * 100.0

    # Focus region
    focus = ((Vpk >= 80) & (Vpk <= 200) & (Vgk >= -2) & (Vgk <= 0))
    focus_mask = focus & mask

    print(f"\nFull range ({N}x{N} grid):")
    print(f"  Max absolute error:  {np.max(abs_err)*1000:.4f} mA")
    print(f"  Mean absolute error: {np.mean(abs_err)*1000:.4f} mA")
    print(f"  Max relative error:  {np.max(rel_err[mask]):.2f}%")
    print(f"  Mean relative error: {np.mean(rel_err[mask]):.2f}%")

    print(f"\nFocus region (Vpk 80-200V, Vgk -2 to 0V):")
    print(f"  Max relative error:  {np.max(rel_err[focus_mask]):.2f}%")
    print(f"  Mean relative error: {np.mean(rel_err[focus_mask]):.2f}%")

    # Spot checks
    print(f"\nSpot checks:")
    test_points = [(150.0, -1.0), (100.0, -2.0), (200.0, -0.5), (50.0, -3.0)]
    for vpk_v, vgk_v in test_points:
        ip_ex = koren_ip_exact(vpk_v, vgk_v, **DEFAULT_PARAMS)
        ip_ap = kd.plate_current(vpk_v, vgk_v)
        err_pct = abs(ip_ex - ip_ap) / max(ip_ex, 1e-10) * 100
        print(f"  Vpk={vpk_v:6.1f}V, Vgk={vgk_v:5.1f}V: "
              f"exact={ip_ex*1000:.4f} mA, approx={ip_ap*1000:.4f} mA, err={err_pct:.3f}%")

    # Pass/fail
    focus_mean = np.mean(rel_err[focus_mask])
    if focus_mean < 1.0:
        print(f"\nPASS: Focus region mean error {focus_mean:.3f}% < 1% target")
    else:
        print(f"\nFAIL: Focus region mean error {focus_mean:.3f}% >= 1% target")

    return focus_mean < 1.0


if __name__ == '__main__':
    success = self_test()
    sys.exit(0 if success else 1)
