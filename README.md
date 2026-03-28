# tangAmp -- FPGA Tube Amp Emulator

**Physics-based guitar tube amplifier emulation on the $25 Tang Nano 20K FPGA.**

Wave Digital Filter (WDF) modeling of real vacuum tube circuits -- not waveshaping, not convolution of the nonlinear part. The actual circuit topology is encoded as a binary tree of series/parallel adaptors with the triode solved at the root via 2D lookup tables derived from the Koren equation.

Validated by four independent solvers (Python, Verilog, ngspice, chowdsp_wdf C++) agreeing within 0.4%.

## Signal Chain

```
Guitar -> ADC -> [Triode Engine] -> [Tone Stack] -> [Power Amp] -> [Cabinet IR] -> DAC -> Speaker
           |      2-stage 12AX7     3-band IIR       6L6/EL34      129-tap FIR       |
         PCM1802  triode_engine.v   tone_stack.v   power_amp.v    cabinet_fir.v    PCM5102
```

## Features

- **Physics-based WDF triode modeling** with Newton-Raphson iteration at the root
- **6 tube types**: 12AX7, 12AU7, 6SL7 (preamp) / 6L6, EL34, 300B (power amp)
- **Cascaded preamp stages** (1-3 stages with coupling caps)
- **Cathode bypass capacitor** for low-frequency gain boost
- **Circuit-derived tone stack** (Fender/Marshall/Vox/Mesa presets)
- **Speaker cabinet IR convolution** (1x12 and 4x12 impulse responses)
- **Output transformer model** with frequency-dependent core saturation
- **Power supply sag** with tube/solid-state rectifier presets
- **Grid current modeling** for realistic high-gain behavior
- **4-way cross-validation**: Python WDF, Verilog RTL, ngspice SPICE, chowdsp_wdf C++
- **Synthesized bitstream** for Tang Nano 20K (77% LUT utilization)
- **59 demo audio files** across 5 amp presets

## Quick Start

```bash
# Generate tube LUTs (Koren equation -> BRAM hex files)
python tube_lut_gen.py

# Smoke test -- verifies LUTs, simulation, gain (<1 second)
python quick_test.py

# Full 4-way validation suite
python validate_all.py

# Generate demo audio (dry + wet WAVs for all presets)
python full_chain_demo.py

# 5 amp presets: Fender Deluxe, Marshall Plexi, JCM800, Vox AC30, Mesa Dual Rec
python amp_sim.py
```

## Hardware

| Component | Part | Cost |
|-----------|------|------|
| FPGA | Sipeed Tang Nano 20K (Gowin GW2AR-18) | ~$25 |
| ADC | PCM1802 / PCM1808 (24-bit, I2S) | ~$5 |
| DAC | PCM5102 (32-bit, I2S) | ~$5 |
| **Total BOM** | | **~$35** |

The Tang Nano 20K provides 20,736 LUTs, 828KB BRAM, 48 DSP slices, and a 27MHz crystal. All signal processing runs in pure RTL -- no soft CPU, no external DRAM.

## FPGA Resource Usage

Gowin GW2AR-LV18QN88C8/I7, synthesized with GowinSynthesis:

| Resource | Used | Available | Utilization |
|----------|------|-----------|-------------|
| LUT | 15,810 | 20,736 | 77% |
| Registers | 780 | 15,750 | 5% |
| BSRAM | 46 | 46 | 100% |
| DSP | 13.5 | 24 | 57% |

Fixed-point Q16.16 throughout. 48kHz sample rate. ~180 clock cycles per sample out of 562 available (27MHz / 48kHz).

## Validation Results

Four independent solvers confirm the WDF implementation:

| Solver | Method | Gain | Error vs Reference |
|--------|--------|------|--------------------|
| Python WDF | Floating-point, 5 N-R iterations | 34.1 dB | -- (reference) |
| Verilog WDF | Q16.16 fixed-point, 2 N-R iterations | 34.1 dB | 0.36% |
| ngspice | SPICE nodal analysis | 34.3 dB | 0.21 dB |
| chowdsp_wdf C++ | Established WDF library | 34.1 dB | 0.18% |

Additional physics validation: datasheet Ip curve match, analytical gain formula, energy conservation, frequency response, tube type comparison.

## Verilog Modules

| Module | Description | Clocks |
|--------|-------------|--------|
| `wdf_triode_wdf.v` | Single WDF triode stage (12AX7, cathode bypass cap) | 14 |
| `triode_engine.v` | Time-multiplexed N-stage cascade, shared LUT BRAMs | -- |
| `tone_stack_iir.v` | 3 cascaded biquad IIR sections (bass/mid/treble) | 9 |
| `cabinet_fir.v` | 129-tap FIR convolution for speaker cabinet | 129 |
| `power_amp_stage.v` | Power amp triode stage (6L6/EL34) | -- |
| `i2s_rx.v` | I2S receiver for PCM1802 ADC | -- |
| `i2s_tx.v` | I2S transmitter for PCM5102 DAC | -- |
| `clk_audio_gen.v` | 27MHz to BCK/LRCK clock generation | -- |
| `tangamp_top.v` | Full chain integration | -- |
| `fpga/tangamp_selftest.v` | Self-test with internal sine + VU meter LEDs | -- |

## Python Scripts

| Script | Purpose |
|--------|---------|
| `tube_lut_gen.py` | Generate Koren LUTs for 6 tube types |
| `fit_tube_model.py` | Curve-fit Koren constants to RCA datasheet data |
| `wdf_triode_sim.py` | Newton-Raphson floating-point reference simulation |
| `wdf_triode_sim_wdf.py` | WDF wave-variable simulation |
| `tone_stack.py` | Tone stack with 4 presets (Fender/Marshall/Vox/scooped) |
| `cabinet_ir.py` | Cabinet IR simulation (1x12, 4x12) |
| `full_chain_demo.py` | Complete amp chain demo audio generation |
| `amp_sim.py` | 5 amp presets with dry/wet audio output |
| `validate_all.py` | Run all validation suites |
| `validate_wdf.py` | Python vs Verilog cross-validation |
| `validate_physics.py` | Physics validation (5 tests) |
| `validate_spice.py` | ngspice independent validation |
| `validate_6l6.py` | 6L6 power tube validation |
| `quick_test.py` | Sub-1-second smoke test |

## Demo Audio

59 WAV files in `demos/`, including:

- **Amp presets**: `amp_fender_deluxe.wav`, `amp_marshall_plexi.wav`, `amp_marshall_jcm800.wav`, `amp_vox_ac30.wav`, `amp_mesa_dual_rec.wav`
- **Gain sweep**: `amp_marshall_gain1.wav` through `amp_marshall_gain10.wav`
- **Tube comparison**: `wet_riff_12AX7.wav`, `wet_riff_12AU7.wav`, `wet_riff_6SL7.wav`, `wet_riff_EL34.wav`, `wet_riff_6L6.wav`
- **Cascaded stages**: `wet_1stage.wav`, `wet_2stage.wav`, `wet_3stage.wav`
- **Tone presets**: `tone_fender_clean.wav`, `tone_marshall_crunch.wav`, `tone_scooped_metal.wav`
- **Cabinet types**: `wet_cab_1x12.wav`, `wet_cab_4x12.wav`, `wet_nocab.wav`
- **Power amp tubes**: `poweramp_6l6.wav`, `poweramp_el34.wav`, `poweramp_300b.wav`
- **Power supply sag**: `sag_none.wav`, `sag_solidstate.wav`, `sag_tube_rectifier.wav`

## Building for FPGA

```bash
# Prerequisites: Gowin EDA (free license) or GowinSynthesis CLI

# Regenerate all hex files
python tube_lut_gen.py
python gen_cab_taps.py

# Synthesize
cd fpga && gw_sh synthesize.tcl

# Flash to Tang Nano 20K
programmer_cli --device GW2AR-18C --run 2 --fsFile fpga/impl/pnr/project.fs
```

The self-test bitstream (`tangamp_selftest.v`) generates an internal sine wave and drives the VU meter LEDs -- no external ADC/DAC needed to verify the FPGA is working.

## Simulation (Icarus Verilog)

```bash
# Single triode stage
iverilog -g2012 -o wdf_sim_v wdf_triode_wdf_tb.v wdf_triode_wdf.v && vvp wdf_sim_v
python analyze_tb.py wdf_tb_output.txt

# Full chain
iverilog -g2012 -o sim_full fpga/tangamp_fullchain_tb.v fpga/tangamp_selftest.v \
  triode_engine.v tone_stack_iir.v cabinet_fir.v wdf_triode_wdf.v && vvp sim_full
```

## WDF Architecture

The triode is modeled as a 3-port nonlinear element at the root of a WDF binary tree:

```
              [TRIODE ROOT]
             /      |      \
       plate_tree  grid   cathode_tree
           |        |          |
       [Series]  [Series]  [Parallel]
       /     \   /     \   /       \
     Rp   Vs(B+) Cin   Rg  Rk      Ck
```

Wave variables propagate up the tree (incident waves), the triode computes reflected waves using a 2D LUT (plate current from Vgk/Vpk), and results propagate back down. Two Newton-Raphson iterations per sample ensure convergence.

## Tube Types

Constants curve-fitted to RCA datasheet data using `fit_tube_model.py`:

| Tube | mu | Role | Character |
|------|----|------|-----------|
| 12AX7 | 100 | Preamp | High gain, standard guitar amp |
| 12AU7 | 27.5 | Preamp | Lower gain, cleaner |
| 6SL7 | 90.4 | Preamp | Vintage octal |
| 6L6 | 10.1 | Power amp | Fender clean headroom |
| EL34 | 11.0 | Power amp | Marshall aggressive midrange |
| 300B | 3.95 | Power amp | Audiophile, warm |

## References

### Papers
- K. Werner, "Virtual Analog Modeling of Audio Circuitry Using Wave Digital Filters," Stanford PhD thesis, 2016. [Link](https://purl.stanford.edu/jy057cz8322)
- J. Pakarinen and M. Karjalainen, "Enhanced Wave Digital Triode Model for Real-Time Tube Amplifier Emulation," IEEE Trans., 2010. [Link](https://ieeexplore.ieee.org/abstract/document/5272282/)
- S. D'Angelo et al., "Wave Digital Filter Adaptors for Arbitrary Topologies and Multiport Nonlinearities," 2019. [Link](https://link.springer.com/article/10.1007/s00034-019-01331-7)
- Y. Zhao and S. Hsieh, "FPGA Implementation of Wave Digital Filters," IEEE, 2023. [Link](https://ieeexplore.ieee.org/document/10322655)
- J. Chowdhury, "chowdsp_wdf: An Advanced C++ Library for Wave Digital Circuit Modelling," 2022. [Link](https://arxiv.org/abs/2210.12554)

### Code
- [chowdsp_wdf](https://github.com/Chowdhury-DSP/chowdsp_wdf) -- C++ WDF library (used for validation)
- [RT-WDF](https://github.com/RT-WDF/rt-wdf_lib) -- R-type adaptor support

## License

MIT
