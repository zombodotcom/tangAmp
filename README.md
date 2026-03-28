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
| LUT | 13,771 | 20,736 | 67% |
| Registers | 962 | 15,750 | 7% |
| BSRAM | 42 | 46 | 92% |
| DSP | 23 | 24 | 96% |

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

## What's Wrong and What Needs Fixing

This section is honest about every shortcut. See `STATUS.md` for the full breakdown.

### Critical (breaks the sound)

1. **Interstage coupling is faked.** We use an arbitrary 12dB attenuation between stages instead of modeling the actual coupling network (22nF cap + 1M grid resistor + grid current loading). The physics for this exists in `coupling_cap_blocking.py` — it needs to REPLACE the fake attenuation, not sit as a separate demo. This is the root cause of the static/artifacts in high-gain demos.

2. **Grid current is bolted on wrong.** We compute Ig AFTER Newton converges and adjust the output. The correct way (already in `grid_current.py`) is a 2x2 Newton solver where Ip and Ig are solved simultaneously so grid current affects the operating point during iteration, not after.

3. **Fitted Koren constants break Verilog.** The curve-fitted 12AX7 (10.7% error vs 44.8% original) has kvb=15102 which causes fixed-point divergence in the Verilog Newton solver. We're still using the worse original constants (kvb=300) for the FPGA. Need to debug the Q16.16 overflow with large kvb values.

### Medium (sounds wrong but doesn't break)

4. **Cabinet IR is synthetic.** We modeled it from Thiele-Small parameters instead of using a real measured impulse response. There are hundreds of free measured cab IRs available (Celestion, OwnHammer free packs, Wilkinson Audio, GuitarHack, Seacow Cabs). Just download one and load it as the FIR taps.

5. **Output transformer not validated.** It's a bandpass + piecewise soft clip. Needs SPICE comparison against an actual transformer model to verify the frequency response and saturation curves are realistic.

6. **NFB loop not validated.** We subtract a scaled output signal but haven't verified loop gain, phase margin, or stability matches a real amp schematic. Should compare against SPICE sim of a Fender Deluxe feedback network.

7. **Amp preset values are guessed.** The gain/master/attenuation numbers for each preset (Fender, Marshall, Vox, Mesa) were made up, not derived from actual amp schematics. Real schematics are available online for all of these.

### Minor (refinements)

8. **No oversampling.** 48kHz with hard tube nonlinearity creates aliasing. 2x oversampling (96kHz internal, decimate to 48kHz output) would clean this up. We have 380 spare clocks — enough for a simple 2x decimation filter.

9. **EL34/300B Koren constants** diverge 50-100% at extreme operating points. Need curve-fitting like we did for 12AX7/12AU7.

10. **Python-only features not in Verilog:** coupling cap blocking, Miller effect, power supply sag, noise, tremolo, presence/resonance. Each has a Verilog spec document (`*_verilog_spec.txt`) but hasn't been implemented in RTL yet.

## Development Roadmap (What To Do Next)

### Phase 1: Fix the fakes (make it correct)
- [ ] Replace 12dB attenuation with proper WDF coupling cap model between stages
- [ ] Move grid current inside Newton iteration (2x2 Jacobian in Verilog)
- [ ] Debug fitted Koren constants (kvb=15102) in Q16.16 fixed-point
- [ ] Download and integrate real measured cabinet IR (e.g. Celestion V30 free IR)
- [ ] Validate output transformer against ngspice transformer model
- [ ] Validate NFB loop against ngspice Fender Deluxe feedback network
- [ ] Derive preset values from actual amp schematics (Fender 5E3, Marshall JCM800, Vox AC30)

### Phase 2: Add the missing physics to Verilog
- [ ] Miller effect LPF between stages (~12 clocks for 3 stages, spec in `miller_verilog_spec.txt`)
- [ ] Power supply sag modulating VB (~3 clocks, spec in `sag_verilog_spec.txt`)
- [ ] 2x oversampling with decimation filter
- [ ] Curve-fit EL34/300B Koren constants

### Phase 3: Hardware bring-up
- [ ] Flash self-test bitstream, verify LEDs
- [ ] Wire PCM1808 ADC + PCM5102 DAC
- [ ] Synthesize full I2S chain (`tangamp_top.v`)
- [ ] Guitar in → FPGA → headphones out
- [ ] A/B test: FPGA output vs Python sim with same recorded input
- [ ] Add potentiometers via SPI ADC (MCP3008) for knob controls

### Phase 4: Polish
- [ ] Noise modeling in Verilog (LFSR + 120Hz hum)
- [ ] Bias tremolo effect
- [ ] Presence/resonance controls via NFB shaping
- [ ] Multiple cabinet IR profiles (switchable)
- [ ] UART debug output for monitoring internal signals

## License

MIT
