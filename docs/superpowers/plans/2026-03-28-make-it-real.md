# Make It Real — Implementation Plan

> **For agentic workers:** Use superpowers:subagent-driven-development to implement this plan.

**Goal:** Fix every remaining shortcut, get hardware working, make this something nobody can call bullshit.

**Current state:** 89 commits, 178 files, LUT 82%, BSRAM 87%, DSP 90%, all 6 validations PASS, 9.9% Verilog-Python error (from J11=2 approximation).

---

## Phase A: Fix the remaining technical debt (no hardware needed)

### Task A1: Improve Newton convergence without dIp/dVpk LUT

The dIp/dVpk LUT can't fit in Tang Nano 20K BSRAM. But J11=2 constant gives 9.9% error. Two options:

**Option 1: Run 5 iterations instead of 3.** Each extra iteration with J11=2 halves the residual. 5 iterations = ~4% of converged value. Cost: 2 extra iterations × 4 clocks × 3 stages × 2 (oversampling) = 48 extra clocks. Current total ~437, new ~485 of 562. Fits.

**Option 2: Secant method approximation.** Use Ip values from consecutive Newton iterations to estimate J11: `J11_est = (f1_prev - f1_curr) / (Ip_prev - Ip_curr)`. No BRAM read needed. Gives superlinear convergence. Cost: 1 extra register per state bank.

**Pick option 2** — better convergence per iteration, same BRAM, minimal extra logic.

**Files:** `rtl/triode_engine.v`, `rtl/wdf_triode_wdf.v`
**Validation:** Verilog-Python error should drop below 5%

### Task A2: Fix output transformer to be frequency-dependent

Current: fixed 25V/30V soft-clip threshold (arbitrary).
Correct: threshold = K / frequency (from V = 4.44*f*N*A*Bsat).

Implementation: track the low-frequency envelope using a 1st-order IIR (already in the Python `output_transformer.py`), reduce clip threshold when bass content is high. The Verilog spec is in `docs/specs/output_transformer_verilog_spec.txt`.

**Files:** `rtl/output_transformer.v`
**Validation:** Compare frequency-dependent saturation against ngspice transformer model at 40Hz, 80Hz, 200Hz, 1kHz

### Task A3: Regenerate all demo audio with corrected physics

The demo WAVs were generated with:
- Grid current K=0.002 (10x too high) — now fixed to 0.0002
- Arbitrary interstage attenuation — now from schematics
- Wrong tube types — now correct (EL84, 6V6)

Need to regenerate ALL demos so they reflect the corrected physics.

**Files:** Run `sim/amp_sim.py`, `sim/full_chain_demo.py`, `sim/generate_demos.py`
**Validation:** Listen to each preset — should sound cleaner, less harsh

### Task A4: Apply per-stage parameters in amp_sim.py

The schematic-derived presets have per-stage Rp/Rk/Ck but need verification that `simulate_preamp_stage()` handles them correctly with the new grid current K.

Test each preset: Fender Deluxe (3 stages, 6V6), Marshall JCM800 (3 stages, EL34), Vox AC30 (2 stages, EL84), Mesa Dual Rec (5 stages, 6L6), Fender Twin (3 stages, 6L6).

**Files:** `sim/amp_sim.py`
**Validation:** DC operating point and gain for each preset should be in reasonable range

### Task A5: Validate 2x oversampling quality

The linear interpolation upsampler is the simplest possible. Measure:
1. Aliasing energy above 10kHz with and without oversampling
2. Compare against 4x oversampling (Python only) as reference
3. If linear interp aliasing reduction is less than 6dB, implement a simple 3-tap FIR instead

**Files:** `sim/oversample_demo.py` (already exists), `rtl/oversample_2x.v`
**Validation:** FFT comparison showing aliasing reduction in dB

---

## Phase B: Hardware bring-up (Monday, needs PCM boards + soldering)

### Task B1: Flash self-test bitstream

Plug in Tang Nano 20K, flash `fpga/impl/pnr/project.fs`.
Verify: LED[0] blinks at 1Hz, LED[1:5] show VU level.

### Task B2: Wire PCM1808 ADC

Connect PCM1808 breakout to Tang Nano 20K GPIO:
- BCK → pin TBD (check tangamp.cst)
- LRCK → pin TBD
- DOUT → pin TBD
- VCC → 3.3V, GND → GND
- FMT0/FMT1 → set for I2S mode (check PCM1808 datasheet)

### Task B3: Wire PCM5102 DAC

Connect PCM5102 breakout:
- BCK → same as ADC BCK (shared clock)
- LRCK → same as ADC LRCK
- DIN → pin TBD
- VCC → 3.3V, GND → GND
- FMT → low (I2S mode)
- XSMT → high (unmute)

### Task B4: Synthesize I2S top-level

Switch from `tangamp_selftest.v` to `tangamp_top.v` (includes I2S RX/TX).
Add I2S pins to `tangnano20k.cst`.
Synthesize and flash.

### Task B5: Audio passthrough test

Guitar → ADC → FPGA (bypass processing) → DAC → headphones.
Verify clean passthrough before enabling triode processing.

### Task B6: Enable triode processing

Guitar → ADC → triode engine → tone stack → transformer → cab IR → DAC → headphones.
A/B test: FPGA vs Python sim with same recorded input signal.

---

## Phase C: SD card cab IR loading (after hardware works)

### Task C1: Wire SD card

Tang Nano 20K has onboard MicroSD slot. Pins: sd_clk(83), sd_cmd(82), sd_dat0(84), sd_dat3(81).
Format SD card with raw cab IR sectors (spec in `docs/specs/sd_card_cab_ir_spec.txt`).

### Task C2: Integrate sd_spi_reader.v

Wire `rtl/sd_spi_reader.v` into the top-level. On button press, read next IR sector and write to cabinet_fir tap RAM.

### Task C3: Create SD card image tool

Python script that writes cab IR hex files to raw SD sectors with the header format from the spec.

---

## Phase D: Bigger FPGA (Tang Mega 138K, when ready)

The Tang Mega 138K has 138,240 LUTs vs 20,736. This enables:

### Task D1: Restore dIp/dVpk LUT for proper Jacobian
With 6x more LUT and proportionally more BSRAM, we can have the full Jacobian. Newton convergence goes from linear (J11=2) to quadratic (proper J11). Error drops from 9.9% to <1%.

### Task D2: 4x oversampling with halfband FIR
Process nonlinearity at 192kHz. Proper 31-tap halfband filter for decimation.

### Task D3: Per-stage tube type selection
Different tube in each preamp stage (e.g., 12AX7 stage 1, 12AU7 stage 2 for a Vox-style circuit). Needs separate LUTs per tube type.

### Task D4: Reverb (spring or plate simulation)
All-pass filter network for spring reverb or diffuse FDN for plate reverb. Needs significant BRAM for delay lines.

---

## Verification checklist (run before any release)

- [ ] `cd sim && python quick_test.py` — PASS
- [ ] `cd sim && python validate_all.py` — ALL PASS
- [ ] `cd sim && python validate_transformer.py` — PASS
- [ ] `cd sim && python validate_nfb.py` — PASS
- [ ] `cd sim && python validate_6l6.py` — PASS
- [ ] `cd sim && python validate_power_tubes.py` — PASS
- [ ] `cd sim && python validate_tonestack.py` — PASS
- [ ] `cd sim && python validate_cabinet.py` — PASS
- [ ] Verilog compile: `iverilog -g2012 -o /dev/null fpga/tangamp_selftest.v rtl/*.v` — clean
- [ ] Gowin synthesis: `cd fpga && gw_sh synthesize.tcl` — bitstream generated
- [ ] FPGA fits: LUT <90%, BSRAM <95%, DSP <95%
- [ ] Hardware A/B: FPGA output matches Python sim within 15% RMS for same input

## Known trade-offs (documented, not hidden)

| Trade-off | Why | Impact | Fix on 138K? |
|-----------|-----|--------|-------------|
| J11=2 constant | Can't fit dIp/dVpk LUT in 20K BSRAM | 9.9% Verilog-Python error | Yes |
| 3 Newton iterations | Clock budget with oversampling | ~12.5% residual vs converged | Yes (more iterations) |
| Linear interpolation upsample | Simplicity | Some HF distortion | Yes (halfband FIR) |
| 12AX7 Koren original constants used in Verilog for now | Fitted kvb=15102 needs J11 fix first | 44.8% vs 10.7% datasheet error | Yes (with proper J11) |
| Output transformer fixed threshold | Frequency-dependent needs envelope detector | Wrong at extreme LF | Fixable on 20K |
