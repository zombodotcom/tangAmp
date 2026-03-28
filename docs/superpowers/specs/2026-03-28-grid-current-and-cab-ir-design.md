# Design: Proper Grid Current Solver + Real Cabinet IRs

**Date:** 2026-03-28
**Scope:** Fix the physics core and use real measured cabinet IRs
**Target:** Synthesizable bitstream on Tang Nano 20K, validated against Python reference

---

## Problem Statement

The current Verilog triode has grid current bolted on AFTER Newton convergence (ST_OUTPUT stage). This means:
- Ip converges to the wrong value (solver doesn't know Ig exists)
- Cathode bypass cap state uses `ip_est` not `ip_total`
- Interstage coupling uses a fake 12dB attenuation to compensate
- Python amp_sim.py uses a `tanh()` band-aid for the same reason
- Cascaded stages produce artifacts at high gain

Additionally, the default cabinet IR is synthetic (Thiele-Small model) despite having real measured Celestion IRs already in `data/`.

## Solution

### 1. Full 2x2 Newton-Raphson Solver in Verilog

Replace the 1x1 Newton (solve Ip only, Ig=0) with a coupled 2x2 solver (Ip and Ig simultaneously).

**Residual equations:**
```
f1(Ip, Ig) = Ip - koren_ip(Vpk, Vgk)
f2(Ip, Ig) = Ig - 0.002 * max(0, Vgk)^1.5
```

**Voltage dependence on both currents:**
```
Vpk = (a_p - a_k) - Rp * Ip - Rk * (Ip + Ig)
Vgk = (a_g - a_k) - Rg * Ig - Rk * (Ip + Ig)
```

**2x2 Jacobian:**
```
J11 = 1 + dIp/dVpk * (Rp + Rk) + dIp/dVgk * Rk
J12 = dIp/dVpk * Rk + dIp/dVgk * (Rg + Rk)
J21 = dIg/dVgk * Rk
J22 = 1 + dIg/dVgk * (Rg + Rk)
```

**2x2 solve:**
```
det = J11 * J22 - J12 * J21
dIp = (J22 * f1 - J12 * f2) / det
dIg = (J11 * f2 - J21 * f1) / det
Ip -= dIp;  Ig -= dIg
clamp both >= 0
```

**Grid current derivative for Jacobian:**
```
dIg/dVgk = 0.003 * max(0, Vgk)^0.5    (when Vgk > 0)
         = 0                             (when Vgk <= 0)
```

**Reflected waves (all three ports now active):**
```
b_p = a_p - 2 * Rp * Ip
b_g = a_g - 2 * Rg * Ig          (was: b_g = a_g)
b_k = a_k + 2 * Rk * (Ip + Ig)  (was: 2 * Rk * Ip)
```

**Cathode bypass cap update uses total current:**
```
ck_z <= (b_cathode + 2*R_cath*(Ip+Ig)) + b_cathode - ck_z
```

**Grid current x^1.5 in fixed-point:** Use a 4th LUT (64-entry, 16-bit) mapping Vgk -> Ig for Vgk in [0, 2V]. 64 entries is sufficient for a smooth function over a 2V range. This avoids the piecewise linear hack currently in ST_OUTPUT. The derivative dIg/dVgk can be computed from adjacent LUT entries (finite difference) or a 5th small LUT.

**Clock cycle budget:** Current solver is 11 cycles. The 2x2 solver adds:
- 1 extra LUT read per iteration (grid current LUT)
- 2x2 matrix solve instead of scalar divide (4 muls + 1 div vs 1 div)
- Estimated: ~16 cycles per triode stage (vs 11 now)
- For 2 preamp stages + 1 power amp via triode_engine.v: ~48 cycles
- Budget: 562 cycles available. Plenty of headroom.

**Additional BRAM:** 64 entries x 16 bits = 128 bytes per LUT. Negligible vs the 98KB tube LUTs. Note: BSRAM is currently at 100% (46/46 blocks). The grid current LUTs are small enough to fit in distributed RAM (LUT-based) rather than BSRAM, or may fit in unused portions of existing BSRAM blocks.

**Total cycle budget with 256-tap cab IR:** 48 (triode) + 256 (cabinet FIR) + 9 (tone stack) = 313 of 562 available. Comfortable.

### 2. Remove Fake Interstage Attenuation

In `sim/amp_sim.py`, the presets have `interstage_atten` values of 30-34dB. Real coupling network attenuation:
```
Atten = Rgrid / (Rp_eff + Rgrid) = 1M / (40k + 1M) = 0.96 = -0.35dB
```

With proper grid current, the tube itself limits positive grid swings. No artificial attenuation needed. Change interstage_atten to ~0.35dB (or the exact value from the coupling network impedance).

### 3. Remove tanh() Band-Aid

In `sim/amp_sim.py`, remove `np.tanh(x / 2.0) * 2.0` soft clipping between stages. Grid current now handles this naturally.

### 4. Real Cabinet IR as Default

Replace `data/cab_ir.hex` (129-tap synthetic) with `data/cab_ir_4x12_V30_SM57.hex` (256-tap real Celestion V30).

Update `rtl/cabinet_fir.v`:
- Change tap count from 129 to 256
- Add a parameter for hex file path so different IRs can be selected at synthesis time
- Update BRAM allocation (256 x 16-bit = 512 bytes, still tiny)

Update `fpga/synthesize.tcl` to reference the new default.

### 5. Update Python Reference Sims

Update `sim/wdf_triode_sim_wdf.py` to use the same 2x2 Newton solver as `sim/grid_current.py`. This becomes the new Python reference for cross-validation.

Update `sim/quick_test.py` expected gain/range thresholds to match the new physics.

### 6. Re-synthesize and Validate

- `iverilog` compile + sim: single stage with 2Vpp input should show asymmetric clipping (positive peaks limited by grid current)
- `python sim/quick_test.py` — PASS
- `python sim/validate_wdf.py` — PASS (Python matches Verilog with grid current)
- `cd fpga && gw_sh synthesize.tcl` — fits on Tang Nano 20K
- Generate updated demo WAVs — no more static on high-gain presets

## Files Modified

| File | Change |
|------|--------|
| `rtl/wdf_triode_wdf.v` | 2x2 Newton solver, grid current LUT, updated reflected waves |
| `rtl/triode_engine.v` | Same 2x2 solver for time-multiplexed engine, grid current LUT |
| `rtl/cabinet_fir.v` | Parameterized tap count (256), hex file path parameter |
| `data/cab_ir.hex` | Replace with real V30 IR (or symlink/copy) |
| `sim/wdf_triode_sim_wdf.py` | Add 2x2 Newton with grid current to match Verilog |
| `sim/amp_sim.py` | Remove tanh(), fix interstage_atten to ~0.35dB |
| `sim/quick_test.py` | Update expected thresholds |
| `sim/validate_wdf.py` | Update thresholds for new physics |
| `sim/tube_lut_gen.py` | Generate grid current LUT hex file |
| `fpga/synthesize.tcl` | Update cab IR reference |
| `fpga/wdf_triode_wdf_tb.v` | Update testbench for grid current verification |

## New Files

| File | Purpose |
|------|---------|
| `data/ig_lut.hex` | Grid current LUT (64 entries, Vgk 0-2V) |
| `data/dig_lut.hex` | Grid current derivative LUT (64 entries) |

## Success Criteria

1. Verilog compiles clean with iverilog
2. Single-stage testbench: asymmetric clipping on positive grid swings
3. Python cross-validation passes (< 5% RMS error Verilog vs Python)
4. quick_test.py passes
5. Gowin synthesis fits (target: < 85% LUT, < 100% BSRAM)
6. Demo WAVs: clean sound on all presets, no static/artifacts at high gain

## Non-Goals

- Fitted Koren constants (kvb=15102) — separate Q16.16 overflow debugging
- Output transformer / NFB SPICE validation — separate task
- Hardware audio testing — PCM arriving 2026-03-30
- Runtime cabinet IR switching — needs pot ADC, future work
- Push-pull power amp topology — future work
