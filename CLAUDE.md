# tangAmp -- FPGA Tube Amp Emulator on Tang Nano 20K

## Project Overview
WDF (Wave Digital Filter) based guitar tube amplifier emulation on the Sipeed Tang Nano 20K FPGA (~$25, Gowin GW2A, ~20K LUTs, 828KB BRAM, 27MHz clock).

Models actual circuit physics using incident/reflected wave pairs through a binary tree structure. Tube nonlinearity via Koren equation captured in 2D LUTs loaded into BRAM.

## Architecture
- **Approach:** WDF for nonlinear preamp/power amp stages + IR convolution for linear cabinet simulation
- **Fixed point:** Q16.16 signed 32-bit throughout
- **Sample rate:** 48kHz
- **Clock:** 27MHz (562 clocks per audio sample)
- **LUT format:** 256x256 entries, 16-bit values, loaded via $readmemh

## Full Signal Chain (Target)
Guitar -> PCM1802 ADC (I2S) -> Preamp stages (12AX7 WDF, cascaded) -> Tone stack -> Power amp stage (6L6/EL34/300B WDF) -> Cabinet IR convolution -> PCM5102 DAC (I2S) -> Output

## Tube Types Supported (Koren constants)
- **Preamp:** 12AX7 (mu=100), 12AU7 (mu=21.5), 6SL7 (mu=70)
- **Power amp:** 6L6 (mu=8.7, Fender clean), EL34 (mu=11, Marshall aggressive), 300B (mu=3.9, audiophile warm)

## Current Files
- `wdf_triode.v` -- Core WDF triode stage (single 12AX7, 11-state pipeline)
- `wdf_triode_tb.v` -- Testbench (440Hz sine, 48kHz, captures output)
- `tube_lut_gen.py` -- Python LUT generator (Koren equation, multi-tube support)
- `analyze_tb.py` -- Post-sim analysis (waveforms + FFT harmonics)
- `ip_lut.hex` / `dip_dvgk_lut.hex` -- Precomputed tube LUTs
- `lut_params.v` -- Auto-generated Verilog parameters

## Known Issues
- **Junction scattering coefficients are wrong** -- ST_SER_S2, ST_SER_S1, ST_PAR_P1 use rough approximations instead of exact port resistance ratios. Produces ~8000V output instead of ~100V.
- Fixed-point overflow/scaling throughout the pipeline needs careful calibration.

## Build/Sim Workflow
1. `python tube_lut_gen.py` -- regenerate LUTs
2. `iverilog -g2012 -o sim wdf_triode_tb.v wdf_triode.v && vvp sim` -- simulate
3. `python analyze_tb.py tb_output.txt` -- analyze output

## Key References
- Kurt Werner's 2016 Stanford PhD thesis (WDF theory, chapters 3-4)
- Koren triode model equation
- chowdsp_wdf (C++ reference implementation)
- RT-WDF (C++ real-time WDF library)
- Korora Audio (commercial FPGA pedal, proved 250us latency)

## Development Phases
1. Audio I/O (I2S for PCM1802 ADC + PCM5102 DAC)
2. Simple nonlinearity (soft clipping LUT, prove pipeline)
3. Single WDF triode stage (fix scattering math) <-- WE ARE HERE
4. Build outward (tone stack, cascaded stages, power amp, cabinet IR)
