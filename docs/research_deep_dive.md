# Deep Research: Tube Modeling, WDF, and FPGA Implementation

Compiled 2026-03-28 from web research with cited sources.

---

## 1. Tube Models Compared

| Model | Params | Operations | Accuracy | Grid Current | FPGA Cost |
|-------|--------|------------|----------|-------------|-----------|
| **Leach (1995)** | 2 (K, mu) | pow(1.5) only | Low | No | Cheapest |
| **Koren (1996)** | 5 | exp+log+sqrt+pow | Good | No (needs extension) | Medium (via LUT) |
| **Cardarilli (2009)** | ~6 | exp+pow | Good in saturation | Partial | Medium |
| **Dempwolf-Zölzer (2011)** | ~8 | exp+log+transcendental | Very good | Yes (built-in) | High |

**Koren via LUT is the right choice for 20K FPGA.** The LUT sidesteps all transcendental functions. Dempwolf is theoretically superior (built-in grid current, used by RT-WDF) but needs more computation.

**Source:** Dempwolf & Zölzer, "A Physically-Motivated Triode Model," DAFx-11. https://dafx.de/paper-archive/2011/Papers/76_e.pdf

---

## 2. WDF 3-Port Triode: Three Approaches

### A. Newton-Raphson at 3-Port Junction (what tangAmp uses)
Standard approach. 2x2 Newton for Vpk and Vgk simultaneously. Used by most academic implementations. Our 3-iteration approach is validated by the literature.

### B. R-Type Adaptor (Werner 2016)
Handles arbitrary topologies via scattering matrix. The Fender Bassman 5F6-A needs a 25-port R-type adaptor. **Far too large for 20K FPGA.** Good for software implementations (chowdsp_wdf uses this).

**Source:** Werner PhD thesis, Stanford 2016. https://purl.stanford.edu/jy057cz8322

### C. Modified Blockwise Method (Zhang & Smith, DAFx 2018)
Decomposes cascade at coupling capacitors. Each 2-triode block solved independently. **This validates tangAmp's architecture** — our `triode_engine.v` time-multiplexes stages separated by coupling caps, which is exactly the blockwise decomposition.

**Source:** Zhang & Smith, DAFx-18. https://www.dafx.de/paper-archive/2018/papers/DAFx2018_paper_25.pdf

---

## 3. CRITICAL FINDING: ADAA Instead of Oversampling

**Antiderivative Antialiasing (ADAA)** was applied specifically to WDF by Chowdhury (DAFx 2020):

Instead of computing `y = f(x)`, compute:
```
y = (F(x[n]) - F(x[n-1])) / (x[n] - x[n-1])
```
where F is the antiderivative of f.

- **First-order ADAA:** ~20dB aliasing reduction. Needs one antiderivative value + one division.
- **Second-order ADAA:** ~40dB reduction. Equivalent to 4x oversampling.

**This is directly applicable to tangAmp.** Instead of 2x oversampling (which halves our clock budget), we can store F(x) in the same LUT alongside Ip and get equivalent or better aliasing reduction. Cost: one extra BRAM column + one division per sample.

**Sources:**
- Chowdhury, "Antiderivative Antialiasing in Nonlinear WDFs," DAFx-20. https://dafx2020.mdw.ac.at/proceedings/papers/DAFx2020_paper_35.pdf
- Chowdhury, AES Journal 2021. https://www.researchgate.net/publication/356185570
- Parker et al., practical considerations. https://jatinchowdhury18.medium.com/practical-considerations-for-antiderivative-anti-aliasing-d5847167f510

---

## 4. Division Elimination — The #1 FPGA Optimization

The Verilog `/` operator synthesizes to a combinational divider: **~700 LUTs per 32-bit division.** We have ~20 divisions across the RTL. Eliminating them could free ~7000 LUTs (from 83% → ~48% utilization).

### Three categories:

**A. Division by constants** (IP_SCALE, DIP_SCALE, KG1, etc.)
Replace with reciprocal multiply: `/ 10000` → `* 429497 >>> 32`
**Savings: ~3500-4200 LUTs. Zero cost. Immediate.**

**B. Division by dynamic determinant** (`/ det` in Newton step)
Newton-Raphson reciprocal: `x_{n+1} = x_n * (2 - d*x_n)`, 4 iterations with 32-entry seed LUT.
**Cost: ~80 LUTs + 1 DSP. Saves ~1400 LUTs. 4 clocks per reciprocal.**

**C. Division for LUT index** (`offset / step` in koren_direct.v)
Step sizes are constants → same as Category A.
**Savings: ~2100 LUTs.**

**Source:** Project F tutorials. https://projectf.io/posts/division-in-verilog/

---

## 5. Power Supply Sag (Cheap to Add)

Real tube amps have B+ voltage drop under load (tube rectifier ≈ 600Ω internal resistance):
```
V_supply = V_ideal - I_load * R_rectifier
```
With RC time constant ~100ms for GZ34 rectifier.

**Implementation:** One first-order IIR filter on B+ voltage, modulated by output signal level. ~20 LUTs. Adds the "squishy" compression feel that distinguishes tube amps from solid-state.

**Source:** https://www.ampbook.com/mobile/dsp/power-supply/

---

## 6. Neural Amp Modeling — NOT Feasible on 20K FPGA

NAM Nano (~3000 params) needs ~640 MACs per sample. With 24 DSP slices × 562 clocks = 13,488 MACs/sample available — technically possible but would consume ALL DSP resources, leaving nothing for tone stack, cab IR, etc.

**Verdict:** WDF approach is more FPGA-efficient. Neural modeling is for GPUs/CPUs.

**Source:** https://www.neuralampmodeler.com/

---

## 7. Gowin GW2A DSP Block Details

Each DSP macro contains: 2 pre-adders + 2 × 18×18 multipliers + 1 three-input ALU with 54-bit accumulate.

**Critical:** Our Q16.16 (32-bit) multiplies need 4 × 18×18 multiplier slots = 2 DSP blocks each. With 24 DSP blocks, we can have at most 12 concurrent 32-bit multiplies. But with time-multiplexing, one pair of DSP blocks can do ~280 multiplies per sample.

Synthesis hints:
- `(* syn_use_dsp = "yes" *)` — force DSP inference
- `(* syn_sharing = "on" *)` — encourage resource sharing

**Source:** Gowin DSP User Guide UG287. https://cdn.gowinsemi.com.cn/UG287E.pdf

---

## 8. I2S Clock Math

Our 27MHz / 9 = 3MHz BCK → 3MHz / 64 = **46,875 Hz** (not 48kHz, 2.3% slow).
For exact 48kHz: need PLL to generate 24.576 MHz (512 × 48000) or 12.288 MHz.
PCM5102A auto-detects, PCM1802 in slave mode follows BCK. So 46.875kHz works.

---

## 9. Pot Reading: MCP3008 SPI ADC

**Recommended:** MCP3008 ($2, 10-bit, 8 channels, SPI).
- SPI Mode 0, 100kHz clock, 4 FPGA pins
- ~50-80 LUTs for SPI master
- 4 channels × 10 bits = smooth knob control

**Source:** https://hackmd.io/@ampheo/how-to-interface-a-potentiometer-with-an-fpga-using-an-external-spi-adc

---

## 10. Oversampling Requirements

| Product | Oversampling | Notes |
|---------|-------------|-------|
| Most amp sim plugins | 4x-8x | User selectable |
| Kemper/Helix | 2x-4x | Fixed |
| FPGA implementations | None or 2x | Clock budget limited |

At 27MHz: 2x (96kHz) gives 281 clocks — tight. 4x (192kHz) gives 140 clocks — not enough for Newton iteration. **ADAA is the better path** (see Section 3).

---

## 11. Output Transformer — Current Approach is Fine

Bandpass + soft clip is what most commercial products use. Full Jiles-Atherton hysteresis modeling exists (Holters, DAFx-16) but is overkill for 20K FPGA.

The frequency-dependent saturation (V/f) we attempted is the correct physics but exceeded LUT budget. Will be feasible after division elimination frees ~7000 LUTs.

**Source:** Holters, DAFx-16. https://www.hsu-hh.de/ant/wp-content/uploads/sites/699/2017/10/Holters_jamodel_DAFx16.pdf

---

## 12. Bernardini 2024: Extended Fixed-Point Solvers (Game-Changer)

**"Wave Digital Extended Fixed-Point Solvers for Circuits With Multiple One-Port Nonlinearities"**
- Bernardini, Giampiccolo, Bozzo, Fontana (Politecnico di Milano)
- IEEE TCAS-I, 2024
- https://ieeexplore.ieee.org/iel8/8919/4358591/10745881.pdf

Family of solvers parameterized by "order" from 0 to infinity:
- **Order 0:** Standard WD fixed-point iteration (simplest, slowest)
- **Order 1-2:** Superlinear convergence, **NO Jacobian inverse needed**
- **Order ∞:** Full Newton-Raphson (fastest, needs Jacobian)

**For tangAmp:** Order-1 solver could replace our 2x2 Newton, eliminating the determinant division entirely. Same convergence speed, simpler implementation. This is the paper to read next.

---

## 13. tangAmp's Position in the Literature

**tangAmp appears to be the only project** implementing WDF triode modeling with Newton-Raphson on a small consumer FPGA ($25 Tang Nano 20K).

| Project | Platform | Nonlinearity | Approach |
|---------|----------|-------------|----------|
| Wu/Zhao (IEEE TVLSI 2024) | Xilinx Virtex 7 (huge) | Transistor | WDF + LUT |
| Hernandez/Hsieh (2016) | NI CompactRIO | BJT/Triode | WDF + LabVIEW |
| Zhang/Smith (DAFx 2018) | CPU (C++) | Triode (Koren) | WDF blockwise |
| Korora Audio Spira | FPGA (unknown) | None (effects only) | Direct DSP |
| FPGAmp (GitHub) | FPGA | None (basic effects) | Direct DSP |
| **tangAmp** | **Gowin GW2AR-18 ($25)** | **Triode (Koren, Newton)** | **WDF + 2D LUT** |

Commercial products (Kemper, Helix, Fractal) all use DSP chips, not FPGAs (except newest Line 6 Stadium with FPGA+ML accelerator).

---

## 14. Key Papers to Read Next

1. **Bernardini 2024** — extended fixed-point solvers (could eliminate our Jacobian division)
2. **Chowdhury DAFx-20** — ADAA in WDF (replace oversampling)
3. **Giampiccolo 2021** — WDF nonlinear transformer (proper OT saturation)
4. **D'Angelo DAFx-19** — Lambert W for explicit WDF solutions (grid current diode)
5. **VIOLA framework** (2024) — automatic SPICE-to-WDF conversion (github.com/polimi-ispl/viola)

---

## Action Items (Priority Order)

1. **Eliminate all `/` operators** — saves ~7000 LUTs, enables everything else
2. **Implement ADAA** instead of oversampling — better aliasing reduction, cheaper
3. **Integrate koren_direct.v** (1D LUT) — frees 42 BSRAM blocks
4. **Investigate Bernardini order-1 solver** — may eliminate Jacobian entirely
5. **Add power supply sag** — cheap, big realism improvement
6. **Add noise gate** (from Blasie thesis pattern) — ~50 LUTs
7. **MCP3008 SPI for pots** — hardware task
8. **Frequency-dependent transformer** — feasible after LUT savings from #1
