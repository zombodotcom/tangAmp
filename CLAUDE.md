# tangAmp -- FPGA Tube Amp Emulator on Tang Nano 20K

## Project Overview
WDF (Wave Digital Filter) based guitar tube amplifier emulation on the Sipeed Tang Nano 20K FPGA (~$25, Gowin GW2A, ~20K LUTs, 828KB BRAM, 27MHz clock).

Models actual circuit physics using incident/reflected wave pairs. Tube nonlinearity via Koren equation captured in 2D LUTs loaded into BRAM. Validated by 4 independent solvers.

## Architecture
- **Approach:** WDF for nonlinear preamp + biquad IIR tone stack + FIR cabinet IR
- **Fixed point:** Q16.16 signed 32-bit throughout
- **Sample rate:** 48kHz
- **Clock:** 27MHz (562 clocks per audio sample, ~180 used)
- **LUT format:** 256x256 entries, 16-bit values, loaded via $readmemh

## Signal Chain (Implemented)
```
Guitar -> [I2S ADC] -> [Triode Engine] -> [Tone Stack] -> [Cabinet IR] -> [I2S DAC] -> Speaker
            PCM1802     2-stage 12AX7      3-band EQ       129-tap FIR      PCM5102
            i2s_rx.v    triode_engine.v    tone_stack.v    cabinet_fir.v    i2s_tx.v
```

## FPGA Resource Usage (Gowin GW2AR-LV18QN88C8/I7)
| Resource | Used | Available | Utilization |
|----------|------|-----------|-------------|
| LUT      | 15,810 | 20,736 | 77% |
| Registers| 780 | 15,750 | 5% |
| BSRAM    | 46 | 46 | 100% |
| DSP      | 13.5 | 24 | 57% |

## Validation Chain (4 independent solvers agree at 34.1dB)
1. Python WDF (floating-point reference)
2. Verilog WDF (Q16.16 fixed-point, 0.36% error)
3. ngspice (SPICE nodal analysis, 0.21dB error)
4. chowdsp_wdf C++ (established WDF library, 0.18% error)
5. Physics validation: datasheet match, analytical gain, energy conservation, frequency response

## Verilog Modules
- `wdf_triode_wdf.v` -- Single WDF triode stage (12AX7, cathode bypass cap, 14 clocks)
- `triode_engine.v` -- Time-multiplexed N-stage cascade sharing one set of LUT BRAMs
- `tone_stack_iir.v` -- 3 cascaded biquad IIR (bass/mid/treble), 9 clocks
- `cabinet_fir.v` -- 129-tap FIR convolution for speaker cabinet, 129 clocks
- `clk_audio_gen.v` -- 27MHz to BCK(3MHz)/LRCK(46.875kHz) clock gen
- `i2s_rx.v` -- I2S receiver for PCM1802 ADC
- `i2s_tx.v` -- I2S transmitter for PCM5102 DAC
- `tangamp_top.v` -- Full I2S audio chain integration
- `fpga/tangamp_selftest.v` -- Self-test (internal sine, VU meter LEDs, no external hardware)

## Python Scripts
- `tube_lut_gen.py` -- LUT generator (Koren equation, 6 tube types)
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
- `gen_cab_taps.py` -- Cabinet FIR hex file generator

## Build/Sim Workflow
```bash
# Regenerate LUTs and cabinet IR
python tube_lut_gen.py
python gen_cab_taps.py

# Quick smoke test (<1s)
python quick_test.py

# Simulate single triode
iverilog -g2012 -o wdf_sim_v wdf_triode_wdf_tb.v wdf_triode_wdf.v && vvp wdf_sim_v
python analyze_tb.py wdf_tb_output.txt

# Full chain simulation
iverilog -g2012 -o sim_full fpga/tangamp_fullchain_tb.v fpga/tangamp_selftest.v triode_engine.v tone_stack_iir.v cabinet_fir.v wdf_triode_wdf.v && vvp sim_full

# Cross-validation
python validate_wdf.py && python validate_physics.py && python validate_spice.py

# Synthesize for Tang Nano 20K
cd fpga && gw_sh synthesize.tcl

# Flash bitstream
programmer_cli --device GW2AR-18C --run 2 --fsFile fpga/impl/pnr/project.fs
```

## Tube Types Supported (Koren constants)
- **Preamp:** 12AX7 (mu=100), 12AU7 (mu=27.48, fitted), 6SL7 (mu=90.41)
- **Power amp:** EL34 (mu=10.98), 6L6 (mu=10.11), 300B (mu=3.95)
- Curve-fitted 12AX7/12AU7 constants in `fit_tube_model.py` (10.7%/2.6% mean error vs RCA datasheet)

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
5. ~~Cabinet IR~~ DONE (129-tap FIR)
6. ~~FPGA synthesis~~ DONE (77% LUT, bitstream ready)
7. External pots for tone controls (needs ADC for knob reading)
8. Power amp stage (6L6/EL34 push-pull)
9. Hardware testing with real guitar signal
