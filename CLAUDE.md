# tangAmp -- FPGA Tube Amp Emulator on Tang Nano 20K

## Project Overview
WDF (Wave Digital Filter) based guitar tube amplifier emulation on the Sipeed Tang Nano 20K FPGA (~$25, Gowin GW2A, ~20K LUTs, 828KB BRAM, 27MHz clock).

Models actual circuit physics using incident/reflected wave pairs. Tube nonlinearity via Koren equation captured in 2D LUTs loaded into BRAM. Validated by 4 independent solvers.

## Architecture
- **Approach:** WDF for nonlinear preamp + biquad IIR tone stack + FIR cabinet IR
- **Fixed point:** Q16.16 signed 32-bit throughout
- **Sample rate:** 48kHz
- **Clock:** 27MHz (562 clocks per audio sample, ~313 used with 256-tap FIR)
- **LUT format:** 256x256 entries, 16-bit values, loaded via $readmemh

## Signal Chain (Implemented)
```
Guitar -> [I2S ADC] -> [Triode Engine] -> [Tone Stack] -> [Cabinet IR] -> [I2S DAC] -> Speaker
            PCM1802     2-stage 12AX7      3-band EQ       256-tap FIR      PCM5102
            i2s_rx.v    triode_engine.v    tone_stack.v    cabinet_fir.v    i2s_tx.v
```

## Directory Structure
```
rtl/           Verilog source modules (synthesizable RTL)
fpga/          Synthesis scripts, constraints, testbenches
sim/           Python simulation, validation, tools, demos
data/          Hex files (tube LUTs + cabinet IRs)
demos/         Generated demo audio, plots, validation images
docs/          Plans, specs, reference docs
ui/            React dashboard
chowdsp_wdf/  C++ WDF reference library (submodule)
```

## FPGA Resource Usage (Gowin GW2AR-LV18QN88C8/I7)
| Resource | Used | Available | Utilization |
|----------|------|-----------|-------------|
| LUT      | 17,142 | 20,736 | 83% |
| Registers| 991 | 15,750 | 7% |
| BSRAM    | 40 | 46 | 87% |
| DSP      | 21.5 | 24 | 90% |

## Validation Chain (4 independent solvers agree at 34.1dB)
1. Python WDF (floating-point reference, 2x2 Newton with grid current)
2. Verilog WDF (Q16.16 fixed-point, 3.2% RMS error with 2x2 Newton)
3. ngspice (SPICE nodal analysis, 0.21dB error)
4. chowdsp_wdf C++ (established WDF library, 0.18% error)
5. Physics validation: datasheet match, analytical gain, energy conservation, frequency response

## Verilog Modules (rtl/)
- `wdf_triode_wdf.v` -- Single WDF triode stage (12AX7, 2x2 Newton with grid current, ~16 clocks)
- `triode_engine.v` -- Time-multiplexed N-stage cascade sharing one set of LUT BRAMs
- `tone_stack_iir.v` -- 3 cascaded biquad IIR (bass/mid/treble), 9 clocks
- `cabinet_fir.v` -- 256-tap FIR convolution for speaker cabinet (real Celestion V30 IR), 256 clocks
- `output_transformer.v` -- Bandpass + soft clip
- `nfb_register.v` -- Negative feedback register
- `power_amp_wdf.v` -- 6L6 power amp WDF stage
- `clk_audio_gen.v` -- 27MHz to BCK(3MHz)/LRCK(46.875kHz) clock gen
- `i2s_rx.v` -- I2S receiver for PCM1802 ADC
- `i2s_tx.v` -- I2S transmitter for PCM5102 DAC
- `tangamp_top.v` -- Full I2S audio chain integration

## FPGA Files (fpga/)
- `tangamp_selftest.v` -- Self-test (internal sine, VU meter LEDs, no external hardware)
- `synthesize.tcl` -- Gowin synthesis script
- `tangnano20k.cst` -- Pin constraints
- `*_tb.v` -- Testbenches

## Python Scripts (sim/)
- `tube_lut_gen.py` -- LUT generator (Koren equation, 6 tube types) -> data/
- `wdf_triode_sim.py` -- Newton-Raphson reference simulation
- `wdf_triode_sim_wdf.py` -- WDF wave-variable simulation
- `wdf_triode_sim_v2.py` -- Cascaded stages simulation
- `tone_stack.py` -- Tone stack simulation (4 presets)
- `cabinet_ir.py` -- Cabinet IR simulation (1x12, 4x12)
- `full_chain_demo.py` -- Complete amp chain (Fender/Marshall/Metal/Jazz presets)
- `fit_tube_model.py` -- Curve-fit Koren constants to RCA datasheet
- `validate_wdf.py` -- Cross-validation (Python vs Verilog)
- `validate_physics.py` -- Physics validation (5 tests)
- `validate_spice.py` -- ngspice independent validation
- `validate_chowdsp.cpp` -- chowdsp_wdf C++ independent validation
- `quick_test.py` -- Sub-1-second smoke test
- `run_tests.py` -- Full parallel test pipeline
- `generate_demos.py` -- Demo audio generation
- `gen_cab_taps.py` -- Cabinet FIR hex file generator -> data/
- `process_cab_irs.py` -- Process WAV cabinet IRs to hex -> data/

## Build/Sim Workflow
```bash
# All sim/validation scripts run from sim/ directory
cd sim

# Regenerate LUTs and cabinet IR (outputs to data/)
python tube_lut_gen.py
python gen_cab_taps.py

# Quick smoke test (<1s)
python quick_test.py

# Simulate single triode (from project root)
cd ..
iverilog -g2012 -o wdf_sim_v fpga/wdf_triode_wdf_tb.v rtl/wdf_triode_wdf.v && vvp wdf_sim_v
python sim/analyze_tb.py wdf_tb_output.txt

# Full chain simulation (from project root)
iverilog -g2012 -o sim_full fpga/tangamp_fullchain_tb.v fpga/tangamp_selftest.v rtl/triode_engine.v rtl/tone_stack_iir.v rtl/cabinet_fir.v rtl/wdf_triode_wdf.v && vvp sim_full

# Cross-validation (from sim/)
cd sim
python validate_wdf.py && python validate_physics.py && python validate_spice.py

# Synthesize for Tang Nano 20K
cd fpga && gw_sh synthesize.tcl

# Flash bitstream
programmer_cli --device GW2AR-18C --run 2 --fsFile fpga/impl/pnr/project.fs
```

## Tube Types Supported (Koren constants)
- **Preamp:** 12AX7 (mu=100), 12AU7 (mu=27.48, fitted), 6SL7 (mu=90.41)
- **Power amp:** EL34 (mu=10.98), 6L6 (mu=10.11), 300B (mu=3.95), EL84 (mu=18.39, fitted), 6V6 (mu=10.33, fitted)
- Curve-fitted 12AX7/12AU7 constants in `sim/fit_tube_model.py` (10.7%/2.6% mean error vs RCA datasheet)
- Curve-fitted EL84/6V6 triode-connected constants (7.9%/5.7% mean error vs Mullard/RCA datasheets)

## Key References

### Papers
- **Kurt Werner PhD thesis (2016)** -- WDF theory, scattering matrices. https://purl.stanford.edu/jy057cz8322
- **Pakarinen & Karjalainen (2010)** -- First bidirectional WDF triode. https://ieeexplore.ieee.org/abstract/document/5272282/
- **D'Angelo et al. (2019)** -- Nonlinear 3-terminal devices in WDF. https://link.springer.com/article/10.1007/s00034-019-01331-7
- **Zhao & Hsieh (2023)** -- FPGA WDF, closest to our approach. https://ieeexplore.ieee.org/document/10322655
- **Chowdhury (2022)** -- chowdsp_wdf library paper. https://arxiv.org/abs/2210.12554

### GitHub Repos
- **chowdsp_wdf** -- C++ WDF reference. https://github.com/Chowdhury-DSP/chowdsp_wdf
- **RT-WDF** -- R-type adaptor support. https://github.com/RT-WDF/rt-wdf_lib

## Development Phases
1. ~~Audio I/O (I2S for PCM1802 ADC + PCM5102 DAC)~~ DONE
2. ~~Single WDF triode stage~~ DONE (34.1dB, 4-way validated)
3. ~~Cascaded stages + cathode bypass cap~~ DONE (2-stage in Verilog, 3-stage in Python)
4. ~~Tone stack~~ DONE (3-band biquad IIR)
5. ~~Cabinet IR~~ DONE (256-tap FIR, real Celestion V30)
6. ~~FPGA synthesis~~ DONE (77% LUT, bitstream ready — needs re-synth with 2x2 Newton)
7. ~~Grid current fix~~ DONE (2x2 Newton, proper interstage coupling, 3.2% Verilog vs Python)
8. Re-synthesize with Gowin EDA (verify fit with 2x2 solver + 256-tap FIR)
9. Hardware testing with real guitar signal (PCM arriving 2026-03-30)
10. External pots for tone controls (needs SPI ADC for knob reading)
11. Power amp stage (6L6/EL34 push-pull)
