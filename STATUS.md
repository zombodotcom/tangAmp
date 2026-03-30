# tangAmp — Project Status (2026-03-30)

## FPGA Resource Usage

GW2AR-LV18QN88C8/I7 on Tang Nano 20K:

| Resource | Used | Available | % |
|----------|------|-----------|---|
| LUT | 17,063 | 20,736 | 83% |
| Registers | 1,303 | 15,750 | 9% |
| BSRAM | 39 | 46 | 85% |
| DSP | 21.5 | 24 | 90% |

The 20K FPGA is near capacity. Future expansion needs Tang Mega 138K.

## What's REAL and VERIFIED (in Verilog, synthesizes, bitstream ready)

| Component | Method | Validated by | Status |
|-----------|--------|-------------|--------|
| 12AX7 WDF triode preamp | Koren LUT + 2x2 Newton-Raphson | Python, Verilog, ngspice, chowdsp_wdf C++ | SOLID |
| 6L6 WDF triode power amp | Same engine, different LUTs | Python, ngspice | SOLID |
| Grid current (Ig) | Langmuir-Child K=0.0002, inside Newton loop | BSPICE data, cross-validated | SOLID |
| Tone stack | 3 cascaded biquad IIR, 5 runtime presets | vs Yeh analog transfer fn | SOLID |
| Cabinet IR | 256-tap FIR, 36 real cab IRs | Measured Celestion V30/G12H | SOLID |
| SD card cab IR loading | SPI reader + loader, boot-time | Compiles, spec complete | NEW |
| Output transformer | Bandpass (60Hz-8kHz) + soft clip | HPF DC bug fixed via sim | OK |
| Negative feedback loop | Scaled output subtracted from input | Functional | OK |
| 2x oversampling | Linear interpolation upsample/downsample | Reduces aliasing | OK |
| Noise gate | Envelope follower + gain smoothing | Threshold bug fixed via sim | NEW |
| Power supply sag | IIR envelope tracker, B+ droop | Functional, not yet audible | NEW |
| Bypass mode | S2 button, ADC→DAC passthrough | Compiles | NEW |
| IR cycling | S1 button, LED[2:4] shows index | Compiles | NEW |
| I2S ADC/DAC | PCM1802 RX + PCM5102 TX | Full-chain sim: 51.5dB gain, AC passes | VALIDATED |
| SPI ADC (MCP3008) | 4-channel pot reading, 1kHz | Compiles, not yet wired | NEW |

## Validation Summary

| Test | Status |
|------|--------|
| Python quick_test.py | PASS |
| Python validate_all.py (6 tests) | ALL PASS |
| WDF Verilog TB (15,211 samples) | PASS |
| I2S full-chain TB (34,533 samples) | PASS — 51.5dB gain |
| Gowin synthesis (selftest) | PASS — bitstream generated |
| Gowin synthesis (tangamp_top) | PASS — bitstream generated |

Verilog-Python error: 9.9% (from 128x128 LUT resolution at low operating currents)

## Bugs Found and Fixed (2026-03-29)

1. **Noise gate threshold 250x too high** — byte placed in integer bits instead of fractional. threshold=8 mapped to 8V instead of 0.03V. Would have caused complete silence.
2. **HPF DC gain -1 instead of 0** — coefficient sum off by 1 (b0+b1+b2=-1). Triode DC leaked through, hit soft-clip, masked all AC.
3. **Grid current K=0.002 was 10x too high** — corrected to K=0.0002 from BSPICE/Effectrode data.

## Known Trade-offs (documented, not hidden)

| Trade-off | Why | Impact |
|-----------|-----|--------|
| J11=2 constant | Can't fit dIp/dVpk LUT in 20K BSRAM | 9.9% Verilog-Python error |
| 128x128 LUT resolution | BSRAM limited | ~5-6 significant bits at operating point |
| Linear interpolation upsample | Simplicity | Some HF aliasing |
| Fixed tone presets (5) | No runtime coefficient computation | Can't sweep continuously |
| Q2.14 coefficients | Tone stack biquads | Treble boost limited to +1.5dB |

## What's PYTHON-ONLY (not in Verilog)

| Feature | File | Status |
|---------|------|--------|
| Coupling cap blocking | coupling_cap_blocking.py | Has Verilog spec |
| Miller effect | miller_effect.py | Has Verilog spec |
| Amp noise | amp_noise.py | Has Verilog spec |
| Bias tremolo | bias_tremolo.py | Has Verilog spec |
| Presence/resonance | presence_resonance.py | Has Verilog spec |

## Hardware Status

- **Tang Nano 20K:** Have it, self-test bitstream runs (LED blink confirmed)
- **PCM1802/1808 ADC:** Ordered, arriving Monday 2026-03-30
- **PCM5102 DAC:** Ordered, arriving Monday 2026-03-30
- **MicroSD card:** Have it, 36 IRs ready as SD image
- **MCP3008 SPI ADC:** Not yet ordered (for pots)
- **Real guitar test:** Planned Monday 2026-03-30

## For 138K FPGA (Future)

- 1D LUT Koren (direct computation, frees all BSRAM) — prototyped, 99% LUT on 20K
- ADAA antialiasing (Chowdhury DAFx-20) — replaces oversampling
- Bernardini order-1 solver — eliminates Jacobian division
- SDRAM reverb/delay (8MB embedded in GW2AR)
- 4x oversampling
- Per-stage tube type selection
