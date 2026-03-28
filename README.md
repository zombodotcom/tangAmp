# tangAmp

### Physics-based tube amp emulation on a $25 FPGA

Wave Digital Filter modeling of real vacuum tube circuits on the [Sipeed Tang Nano 20K](https://wiki.sipeed.com/hardware/en/tang/tang-nano-20k/nano-20k.html). Not waveshaping. Not convolution. The actual circuit topology — series/parallel adaptors, triode nonlinearity solved via Newton-Raphson iteration, 2D lookup tables from the Koren equation.

Four independent solvers agree within 0.4%.

<p align="center">
  <img src="demos/wdf_sim_wdf_waveforms.png" width="700" alt="12AX7 triode stage WDF simulation showing DC settling, input/output waveforms, and asymmetric clipping">
</p>

---

## Signal Chain

```
Guitar ──> PCM1802 ADC ──> Triode Engine ──> Tone Stack ──> Cabinet IR ──> PCM5102 DAC ──> Speaker
              I2S rx        2x 12AX7 WDF     3-band IIR     129-tap FIR       I2S tx
             i2s_rx.v     triode_engine.v   tone_stack.v   cabinet_fir.v     i2s_tx.v
```

All processing in pure RTL at 48kHz. No soft CPU, no external DRAM. ~180 clock cycles per sample out of 562 available.

## Harmonic Content

A pure 440Hz sine in, rich tube harmonics out — the asymmetric transfer function generates the characteristic even/odd harmonic series that gives tubes their sound:

<p align="center">
  <img src="demos/wdf_sim_wdf_harmonics.png" width="600" alt="Harmonic spectrum showing tube-generated harmonics from a pure sine input">
</p>

## Tube Models

Plate and transfer curves generated from the Koren equation, curve-fitted to RCA datasheet measurements:

<p align="center">
  <img src="demos/tube_plot.png" width="700" alt="12AX7 and 6L6 plate curves and transfer curves">
</p>

| Tube | mu | Role | Character |
|------|----|------|-----------|
| **12AX7** | 100 | Preamp | High gain, standard guitar amp |
| **12AU7** | 27.5 | Preamp | Lower gain, cleaner |
| **6SL7** | 90.4 | Preamp | Vintage octal |
| **6L6** | 10.1 | Power amp | Fender clean headroom |
| **EL34** | 11.0 | Power amp | Marshall aggressive midrange |
| **300B** | 3.95 | Power amp | Audiophile warm |

## Validation

Four independent solvers confirm the WDF implementation produces correct gain at the correct operating point:

| Solver | Method | Gain | Error |
|--------|--------|------|-------|
| Python WDF | Floating-point, 5 N-R iterations | 34.1 dB | reference |
| Verilog WDF | Q16.16 fixed-point, 2 N-R iterations | 34.1 dB | 0.36% |
| ngspice | SPICE nodal analysis | 34.3 dB | 0.21 dB |
| chowdsp_wdf C++ | Established WDF library | 34.1 dB | 0.18% |

<p align="center">
  <img src="demos/validation_spice.png" width="700" alt="SPICE vs WDF validation showing plate voltage, AC output, and spectrum">
</p>

Physics validation passes all 5 tests — datasheet Ip curve match, analytical small-signal gain, DC sweep (150 operating points), energy conservation (<0.001% error), and flat frequency response:

<p align="center">
  <img src="demos/validation_report.png" width="700" alt="Physics validation report: datasheet match, DC sweep, frequency response">
</p>

## Tone Stack

Circuit-derived biquad IIR filters matching the Fender Bassman and Marshall JCM800 tone stack topologies:

<p align="center">
  <img src="demos/tonestack_response.png" width="700" alt="Tone stack frequency response for Fender and Marshall circuits">
</p>

## Cabinet Impulse Responses

129-tap FIR convolution with real measured Celestion IRs (V30, G12H-75, G12H-150) and synthetic Thiele-Small models:

<p align="center">
  <img src="demos/cabinet_ir.png" width="700" alt="Cabinet IR time and frequency domain for 1x12 and 4x12 configurations">
</p>

## Amp Presets

Five amp voicings — each with distinct clipping character and spectral signature:

<p align="center">
  <img src="demos/amp_presets.png" width="500" alt="Waveform and spectrum comparison across 5 amp presets">
</p>

## FPGA Resource Usage

Gowin GW2AR-LV18QN88C8/I7 on Tang Nano 20K:

| Resource | Used | Available | |
|----------|-----:|----------:|---|
| **LUT** | 15,810 | 20,736 | `################----` 77% |
| **Registers** | 780 | 15,750 | `#-------------------` 5% |
| **BSRAM** | 46 | 46 | `####################` 100% |
| **DSP** | 13.5 | 24 | `############--------` 57% |

Q16.16 signed fixed-point throughout. 48kHz sample rate. 27MHz clock.

## Hardware BOM

| Component | Part | Cost |
|-----------|------|-----:|
| FPGA | Sipeed Tang Nano 20K (Gowin GW2AR-18) | ~$25 |
| ADC | PCM1802 / PCM1808 (24-bit I2S) | ~$5 |
| DAC | PCM5102 (32-bit I2S) | ~$5 |
| | **Total** | **~$35** |

## Project Structure

```
rtl/           Synthesizable Verilog (triode, tone stack, cabinet, I2S, top)
fpga/          Synthesis scripts, constraints, testbenches
sim/           Python simulation, validation, demo generation
data/          Hex LUTs (tube curves) and cabinet IRs
demos/         Generated audio, plots, validation images
docs/          Plans, specs, reference documents
ui/            React dashboard
```

## Quick Start

```bash
# Python simulation + validation (from sim/)
cd sim
python quick_test.py              # smoke test (<1s)
python validate_all.py            # full 4-way validation
python full_chain_demo.py         # generate demo audio

# Verilog simulation (from project root, requires Icarus Verilog)
iverilog -g2012 -o wdf_sim_v fpga/wdf_triode_wdf_tb.v rtl/wdf_triode_wdf.v && vvp wdf_sim_v

# FPGA build (requires Gowin EDA)
cd sim && python tube_lut_gen.py  # regenerate LUTs -> data/
cd fpga && gw_sh synthesize.tcl   # synthesize bitstream
programmer_cli --device GW2AR-18C --run 2 --fsFile fpga/impl/pnr/project.fs
```

The self-test bitstream (`fpga/tangamp_selftest.v`) generates an internal sine wave and drives VU meter LEDs — no external hardware needed to verify the FPGA works.

## WDF Architecture

The triode is a 3-port nonlinear element at the root of a Wave Digital Filter binary tree. Wave variables (incident/reflected voltage pairs) propagate through the tree, and the triode is solved via 2D LUT + Newton-Raphson iteration:

```
                ┌─────────────┐
                │  TRIODE ROOT │
                │  (Ip, Ig LUT)│
                └──┬────┬────┬┘
                   │    │    │
            plate  │  grid │  cathode
                   │    │    │
              ┌────┴┐ ┌┴────┐ ┌┴──────┐
              │Series│ │Series│ │Parallel│
              └┬───┬┘ └┬───┬┘ └┬──────┬┘
               │   │   │   │   │      │
              Rp  V_B  Cin  Rg  Rk    Ck
             100k 200V 22nF 1M 1.5k  22uF
```

Each Newton-Raphson iteration:
1. Extract port voltages from incident waves
2. Look up Ip and dIp/dVgk, dIp/dVpk from 256x256 BRAM LUTs
3. Compute Jacobian correction
4. Update reflected waves

Two iterations per sample. 14 clock cycles total per triode stage.

## What Needs Fixing

> See [STATUS.md](STATUS.md) for the full breakdown.

**Critical** — Interstage coupling uses a fake 12dB attenuation instead of the real coupling network. Grid current is computed after Newton convergence instead of inside the iteration. These cause artifacts in high-gain presets.

**Medium** — Output transformer and NFB loop lack SPICE validation. Amp preset values are guessed, not derived from real schematics.

**Next up** — Flash the bitstream on real hardware, plug in a guitar.

## References

- K. Werner, *"Virtual Analog Modeling of Audio Circuitry Using Wave Digital Filters,"* Stanford PhD thesis, 2016. [PDF](https://purl.stanford.edu/jy057cz8322)
- J. Pakarinen & M. Karjalainen, *"Enhanced Wave Digital Triode Model,"* IEEE Trans., 2010. [Link](https://ieeexplore.ieee.org/abstract/document/5272282/)
- S. D'Angelo et al., *"Wave Digital Filter Adaptors for Arbitrary Topologies,"* 2019. [Link](https://link.springer.com/article/10.1007/s00034-019-01331-7)
- Y. Zhao & S. Hsieh, *"FPGA Implementation of Wave Digital Filters,"* IEEE, 2023. [Link](https://ieeexplore.ieee.org/document/10322655)
- J. Chowdhury, *"chowdsp_wdf,"* 2022. [Paper](https://arxiv.org/abs/2210.12554) / [Code](https://github.com/Chowdhury-DSP/chowdsp_wdf)

## License

MIT
