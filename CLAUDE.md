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

### Papers
- **Kurt Werner PhD thesis (2016)** -- "Virtual Analog Modeling of Audio Circuitry Using Wave Digital Filters," Stanford. Chapters 3-4: WDF theory, scattering matrices, series/parallel adaptors. https://purl.stanford.edu/jy057cz8322
- **Pakarinen & Karjalainen (2010)** -- "Enhanced Wave Digital Triode Model for Real-Time Tube Amplifier Emulation," IEEE Trans. First bidirectional WDF triode. https://ieeexplore.ieee.org/abstract/document/5272282/
- **D'Angelo et al. (2019)** -- Nonlinear 3-terminal devices (triode/MOSFET/JFET) in WDF. Modified Newton-Raphson. https://link.springer.com/article/10.1007/s00034-019-01331-7
- **Zhao & Hsieh (2023)** -- FPGA WDF on Xilinx Virtex-7, 600MHz, 77K LUTs, nonlinear LUT in BRAM. Closest to our approach. https://ieeexplore.ieee.org/document/10322655
- **David Yeh WDF Tutorial** -- Compact intro to WDF basics. https://ccrma.stanford.edu/~dtyeh/papers/wdftutorial.pdf
- **Chowdhury (2022)** -- chowdsp_wdf library design paper. https://arxiv.org/abs/2210.12554
- **DAFx-18** -- Cascaded vacuum tube WDF with modified blockwise Newton. https://www.dafx.de/paper-archive/2018/papers/DAFx2018_paper_25.pdf
- **Dunkel et al. (2016)** -- Fender Bassman 5F6-A WDF case study, 4 triodes, 25-port R-type adaptor. https://www.dafx.de/paper-archive/details.php?id=D55omfyIRYck-aEax3691Q

### GitHub Repos
- **chowdsp_wdf** -- Best C++ reference for adaptor scattering code. https://github.com/Chowdhury-DSP/chowdsp_wdf
- **RT-WDF** -- R-type adaptor support, Bassman example. https://github.com/RT-WDF/rt-wdf_lib
- **WaveDigitalFilters** -- TR-808, diode clipper, Baxandall EQ examples. https://github.com/jatinchowdhury18/WaveDigitalFilters

### FPGA Reference
- **Korora Audio Spira pedal** -- SystemVerilog, 96kHz, 250us latency, pure RTL. https://medium.com/@korora_audio/a-case-study-using-fpga-for-audio-dsp-eab4859bdde2

## WDF Scattering Math (Quick Reference)

### Wave Variables
- `a = v + R0*i` (incident), `b = v - R0*i` (reflected)
- `v = (a+b)/2`, `i = (a-b)/(2*R0)`

### One-Port Elements
- **Resistor:** R0 = R, b = 0 (absorbs everything)
- **Capacitor (bilinear):** R0 = 1/(2*Fs*C), b = z (state), update z = a after each sample
- **Voltage source:** R0 ≈ small, b = 2*V - a

### Series Adaptor (3-port)
- R_parent = R1 + R2
- gamma = R1 / (R1 + R2)
- Upward: b_parent = -(b1 + b2)
- Downward: a1 = b1 - gamma*(x + b1 + b2), a2 = -(x + a1)

### Parallel Adaptor (3-port)
- G_parent = G1 + G2 (conductances)
- gamma = G1 / (G1 + G2)
- Upward: bDiff = b2 - b1, b_parent = b2 - gamma*bDiff
- Downward: a2 = x + b_parent - b2, a1 = a2 + bDiff

### Triode at Root (3-port nonlinear)
- Ig ≈ 0 (no grid current): b_g = a_g
- Ip from Koren LUT: b_p = a_p - 2*R_p*Ip, b_k = a_k + 2*R_k*Ip
- Newton-Raphson: solve Ip = koren(Vpk, Vgk) where Vpk = (a_p-a_k) - (R_p+R_k)*Ip

## WDF Binary Tree for Common-Cathode Triode
```
            [TRIODE 3-PORT ROOT]
           /        |          \
     plate_tree  grid_tree  cathode_tree
        |            |            |
    [Series]     [Series]    [Parallel]
    /      \     /      \    /        \
  Rp    Vs(B+) Cin     Rg  Rk        Ck
```

## Diagnosis: Current Verilog Bug
The current `wdf_triode.v` does a single LUT lookup per sample using stale estimates (no iteration). Python reference uses 5 Newton-Raphson iterations per sample. Result: Verilog outputs 47.2dB/±60V instead of correct 29.1dB/±14V. Also subtracts hardcoded 100V DC instead of actual quiescent ~146V. Fix: implement proper WDF tree with 2 Newton iterations at the root (plan in `docs/superpowers/plans/2026-03-27-proper-wdf-triode.md`).

## Development Phases
1. Audio I/O (I2S for PCM1802 ADC + PCM5102 DAC)
2. Simple nonlinearity (soft clipping LUT, prove pipeline)
3. Single WDF triode stage (fix scattering math) <-- WE ARE HERE
4. Build outward (tone stack, cascaded stages, power amp, cabinet IR)
