# tangAmp — Project Status (2026-03-28)

## What's REAL and VERIFIED (Verilog, on FPGA bitstream)

| Component | Method | Validated by | Error | Status |
|-----------|--------|-------------|-------|--------|
| 12AX7 WDF triode preamp | Koren LUT + Newton-Raphson | Python, Verilog, ngspice, chowdsp_wdf C++ | 0.36% | SOLID |
| 6L6 WDF triode power amp | Same engine, different LUTs | Python, ngspice, analytical | 0.10% | SOLID |
| Cathode bypass cap (Ck) | WDF parallel adaptor | Python match | 0.00% | SOLID |
| Grid current limiting | Piecewise linear Vgk^1.5 | In Verilog + Python | new, needs cross-validation | NEW |
| Tone stack (Fender/Marshall) | Circuit-derived biquad IIR | vs Yeh analog transfer fn | 1.51dB max | SOLID |
| Cabinet IR | 129-tap FIR, Thiele-Small model | vs Eminence speaker data | shape match | OK |
| Output transformer | Bandpass + soft clip | — | not independently validated | NEEDS VALIDATION |
| Negative feedback loop | Subtract scaled output from input | — | not validated vs real amp | NEEDS VALIDATION |
| Time-multiplexed 2-stage + power amp | Shared BRAM engine | Verilog testbench PASS | — | OK |

**FPGA: GW2AR-LV18QN88C8/I7 — LUT 67%, BSRAM 92%, DSP 96%**

## What's PYTHON-ONLY (not in Verilog)

These exist as standalone scripts with demos but are NOT in the FPGA signal chain:

| Feature | File | What it does | Verilog spec? |
|---------|------|-------------|---------------|
| Coupling cap blocking | `coupling_cap_blocking.py` | DC bias shift from asymmetric clipping | No |
| Miller effect | `miller_effect.py` | Parasitic cap LPF between stages (18kHz) | `miller_verilog_spec.txt` |
| Power supply sag | `power_supply_sag.py` | Dynamic VB from current draw | `sag_verilog_spec.txt` |
| Amp noise | `amp_noise.py` | Thermal, shot, 1/f, 120Hz hum | `noise_verilog_spec.txt` |
| Bias tremolo | `bias_tremolo.py` | LFO-modulated cathode bias | `tremolo_verilog_spec.txt` |
| Presence/resonance | `presence_resonance.py` | NFB-shaped tone controls | `presence_resonance_verilog_spec.txt` |

## What's FAKED or HACKY

| Issue | Where | What's wrong | Fix |
|-------|-------|-------------|-----|
| Interstage attenuation = 12dB | `amp_sim.py` | Tuned by trial, not derived from schematic | Need SPICE sim of actual coupling network |
| Grid current Vgk clamp at 2V | `amp_sim.py` | Band-aid to prevent numerical explosion | Implement proper coupling cap blocking in the solver |
| amp_sim.py gain/master tables | `amp_sim.py` | Arbitrary numbers, not from real amp measurements | Need real amp schematic analysis |
| Cabinet IR is synthetic | `cabinet_ir.py` | Approximation, not measured from a real speaker | Use a real measured IR (free packs available online) |
| Output transformer saturation | `output_transformer.v` | Piecewise linear soft clip, not physics-based core model | Would need hysteresis model (complex) |
| 3-stage demos may still have artifacts | `demos/` | Grid current + 12dB atten may not fully fix all presets | Need to listen and verify |
| EL34/300B Koren constants | `tube_lut_gen.py` | Diverge 50-100% at extreme operating points | Curve-fit like we did for 12AX7/12AU7 |

## What's NEEDED to be a real product

### Hardware (have Tang Nano 20K, need):
- PCM1808 ADC (~$5) — guitar input
- PCM5102 DAC (~$5) — audio output
- Potentiometers + small ADC (MCP3008) — knob controls

### Software/FPGA:
1. **Flash and test** the current bitstream on the actual board
2. **Integrate I2S** — modules exist (`i2s_rx.v`, `i2s_tx.v`), tested in sim, not in synthesis
3. **Add pot reading** — SPI ADC for bass/mid/treble/gain/master knobs
4. **Measured cabinet IRs** — replace synthetic with real speaker measurements
5. **Oversampling** — current 48kHz has aliasing from tube nonlinearity. 2x or 4x oversampling would help (we have clock budget)

### Validation still needed:
1. Cross-validate grid current: Python vs Verilog (new, not yet compared)
2. Validate output transformer against SPICE
3. Validate NFB loop against SPICE
4. Hardware A/B test: FPGA output vs Python sim with same input signal

## Key Files for Development

### Core Verilog (the product):
```
wdf_triode_wdf.v      — single WDF triode stage (the proven core)
triode_engine.v        — time-mux N stages + power amp, shared BRAM
tone_stack_iir.v       — 3 biquad IIR (Fender/Marshall coefficients)
output_transformer.v   — bandpass + soft clip
nfb_register.v         — negative feedback register
cabinet_fir.v          — 129-tap FIR convolution
clk_audio_gen.v        — I2S clock generation
i2s_rx.v / i2s_tx.v    — I2S ADC/DAC interface
tangamp_top.v          — full I2S audio chain
fpga/tangamp_selftest.v — self-test (internal sine, LED VU meter)
```

### Python (validation + demos):
```
tube_lut_gen.py        — generates all LUT hex files
validate_all.py        — runs ALL validation in one command
quick_test.py          — 1-second smoke test
validate_wdf.py        — cross-validation (Python vs Verilog)
validate_physics.py    — 5 physics tests
validate_spice.py      — ngspice independent validation
validate_6l6.py        — 6L6 power tube validation
amp_sim.py             — full amp simulator with presets
```

### Build commands:
```bash
python tube_lut_gen.py                    # regenerate LUTs
python quick_test.py                       # smoke test
python validate_all.py                     # full validation
export PATH="$PATH:/c/iverilog/bin"
iverilog -g2012 -o sim_verified fpga/tangamp_verified_tb.v fpga/tangamp_selftest.v triode_engine.v wdf_triode_wdf.v tone_stack_iir.v output_transformer.v nfb_register.v cabinet_fir.v && vvp sim_verified
cd fpga && gw_sh synthesize.tcl           # Gowin synthesis
```

## Research References (for continuing development)
- Kurt Werner PhD thesis (2016): https://purl.stanford.edu/jy057cz8322
- chowdsp_wdf C++ library: https://github.com/Chowdhury-DSP/chowdsp_wdf
- Pakarinen & Karjalainen (2010) triode WDF: https://ieeexplore.ieee.org/abstract/document/5272282/
- Zhao & Hsieh (2023) FPGA WDF: https://ieeexplore.ieee.org/document/10322655
- Korora Audio FPGA pedal: https://medium.com/@korora_audio/a-case-study-using-fpga-for-audio-dsp-eab4859bdde2
- Norman Koren tube model: normankoren.com
- paengdesign SPICE fits: paengdesign.wordpress.com

## Session Log (what was done 2026-03-27 to 2026-03-28)
1. Installed iverilog on Windows, ran first simulation
2. Fixed the broken triode (47dB → 34dB) with proper WDF tree
3. Validated with 4 solvers: Python, Verilog, ngspice, chowdsp_wdf C++
4. Added cathode bypass cap (34dB gain)
5. Added cascaded stages (Python: 2/3 stage, Verilog: time-multiplexed)
6. Added tone stack (circuit-derived Fender/Marshall coefficients)
7. Added cabinet IR (Thiele-Small speaker model, 129-tap FIR)
8. Added real 6L6 power amp (WDF triode, separate LUTs)
9. Added output transformer (bandpass + saturation)
10. Added negative feedback loop
11. Added grid current limiting (fixes cascaded stage clipping)
12. Curve-fit Koren constants (12AX7: 44.8%→10.7% error, 12AU7: 31%→2.6%)
13. 20 parallel agents built: physics models, validation, demos, cleanup
14. Synthesized full chain: LUT 67%, BSRAM 92%, DSP 96%
15. 108 demo WAV files, 5 amp presets
16. README.md, VALIDATION_REPORT.md, React dashboard
