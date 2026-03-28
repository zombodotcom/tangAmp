# Fix the Real Problems: Grid Current in Verilog + Correct Gain Staging

## Context

The project has a working FPGA tube amp chain (65% LUT, synthesized) but two fundamental problems:

1. **The static/clipping in demos is caused by missing grid current modeling** — when the grid goes positive, real tubes draw current that limits the signal. Without it, the triode output can swing to absurd voltages that destroy subsequent stages. We slapped a `tanh()` band-aid on `amp_sim.py` but that's fake physics.

2. **The interstage attenuation values are guessed** — 30-34dB was tuned by ear, not derived from actual amp schematics. Real amps have coupling networks with known component values that set the attenuation.

The fix: **integrate grid current into the Verilog triode engine** (where it belongs), then derive correct interstage coupling from circuit analysis.

## What needs to happen

### Task 1: Grid current in the Verilog triode engine

**Files:** `triode_engine.v`, `wdf_triode_wdf.v`

The grid current model from `grid_current.py` uses a 2x2 Newton-Raphson solver (Ip and Ig coupled). In the Verilog:

- Currently: `b_g = a_g` (grid reflects fully, Ig=0)
- After: `b_g = a_g - 2*R_g*Ig`, `b_k = a_k + 2*R_k*(Ip + Ig)`
- Grid current: `Ig = 0.002 * max(0, Vgk)^1.5` — needs a small LUT or piecewise approximation
- The Newton Jacobian becomes 2x2 but we can simplify: since Ig only flows when Vgk > 0 (rare, only at high overdrive), use a conditional branch — normal path (Ig=0) when Vgk < 0, extended path when Vgk >= 0

Implementation: piecewise linear approximation of `x^1.5` for the grid current (3 segments: 0-0.5V, 0.5-1V, 1-2V). No LUT needed, just comparisons and shifts. Adds ~5 clocks when grid current is active, 0 clocks when inactive (Vgk < 0).

### Task 2: Derive interstage attenuation from circuit analysis

**Files:** `amp_sim.py` (fix the preset values)

Real amp interstage coupling: Cout (22nF) → Rgrid_next (1M) forms a voltage divider with the previous stage's plate impedance (ra || Rp ≈ 40k for 12AX7). The attenuation is:

```
Atten = Rgrid / (Rp_eff + Rgrid) = 1M / (40k + 1M) ≈ 0.96 = -0.35dB
```

So the coupling network barely attenuates at all! The real limiting factor in a tube amp is:
- Grid current (limits positive swing to ~0V)
- Tube cutoff (limits negative swing to ~-4V for 12AX7)
- The next stage's input range is naturally ±2V, not ±100V

This means our interstage_atten of 30-34dB is WRONG. The correct approach: don't attenuate between stages at all (or minimally), but let grid current do the limiting naturally. This is exactly why Task 1 is the real fix.

### Task 3: Remove the tanh() band-aid

**Files:** `amp_sim.py`

Once grid current is in the triode solver, remove the `np.tanh(x / 2.0) * 2.0` hack and reduce interstage_atten back to realistic values (~1-3dB from the coupling network).

### Task 4: Update Python sims to match

**Files:** `wdf_triode_sim_wdf.py`, `wdf_triode_sim.py`, `quick_test.py`

Add grid current to the Python WDF sim so it matches the Verilog. Update validation thresholds.

### Task 5: Re-synthesize and verify

Compile, simulate, re-synthesize for Tang Nano 20K. Verify gain and THD are in realistic ranges without the static/clipping artifacts.

## Verification

1. `iverilog` compile clean
2. Verilog testbench: single stage with 2V input should show asymmetric clipping (positive peaks limited by grid current)
3. `python quick_test.py` — PASS
4. `python validate_wdf.py` — PASS (Python matches Verilog with grid current)
5. Gowin synthesis — still fits
6. Demo WAVs: no more static on high-gain presets

## Key files to modify
- `triode_engine.v` — add grid current conditional to Newton solver
- `wdf_triode_wdf.v` — add grid current to standalone module
- `amp_sim.py` — fix interstage attenuation, remove tanh hack
- `wdf_triode_sim_wdf.py` — add grid current to Python reference
- `quick_test.py` — update expected ranges
