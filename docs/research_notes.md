# Research Notes — Verified Data Sources

## Grid Current (12AX7)

**Model:** Ig = K * max(0, Vgk)^1.5 (Langmuir-Child law)

**K coefficient:** 0.0002 A/V^1.5

**Sources:**
- BSPICE tube models (Effectrode): RGI = 2000 ohm, diode IS=1nA
  - At Vgk=+1V: Ig ≈ 0.2mA. Source: effectrode.com/knowledge-base/the-accurate-bspice-tube-models/
- Electromagnetic World blog: perveance G = 1.73e-6 A/V^1.5, Rgk ≈ 2.7k ohm
  - Source: electromagneticworld.blogspot.com/2017/05/triode-vacuum-tube-modeling.html
- diyAudio forum measurements: ideal diode + 0.5V + 3k series resistor
  - At Vgk=+1V: Ig ≈ 0.167mA. Source: diyaudio.com/community/threads/spice-model-with-grid-current-12ax7.160561/
- Dempwolf & Zolzer (DAFx 2011): physically-motivated model with measured data
  - Source: dafx.de/paper-archive/2011/Papers/76_e.pdf

**Previous K=0.002 was 10x too high** — gave 2mA at Vgk=+1V instead of ~0.2mA.

## Output Transformer Saturation

**Core material:** M6 grain-oriented silicon steel (EI laminations)
**Bsat:** 1.4T (guitar amp design target), 1.5T absolute max

**Saturation equation:** V_rms_max = 4.44 * f * N * A * Bsat
- Saturation depends on V/f (lower frequencies saturate first)
- Fender Deluxe OT (~20H, 6.6k:8 ohm): does NOT saturate at normal guitar frequencies
- Tweed Deluxe (deliberately undersized OT): saturates at high volume for compression

**Sources:**
- diyAudio OT saturation thread
- Marshall Forum OT saturation thread
- Mercury Magnetics: mercurymagnetics.com
- Hammond 125E specifications
- sound-au.com/xfmr.htm

**Our current model** (fixed 25V/30V threshold) is physically wrong. Correct: threshold should scale with 1/f. But the bandpass filter already handles the frequency shaping, so the soft-clip mainly affects extreme peaks.

## Newton-Raphson Convergence

**RT-WDF library:** tolerance 1e-6, max 50 iterations, average 3.03 iterations with proper Jacobian
- Source: github.com/RT-WDF/rt-wdf_lib, rt-wdf_nlSolvers.h

**chowdsp_wdf:** uses Wright Omega function for diodes (closed-form + 1 refinement), does NOT provide built-in triode solver
- Source: github.com/Chowdhury-DSP/chowdsp_wdf, omega.h

**Our implementation:** 2-3 iterations with J11 constant approximation
- With J11=2 (constant): convergence is linear (each iteration ~halves error)
- With proper J11 from dIp/dVpk: convergence is quadratic (much faster)
- Clock budget allows up to 50 iterations if needed

## Koren Model Limitations

**Norman Koren explicitly stated:** "I didn't spend a lot of effort on modeling the triode's action for Vgk > 0" and "that region of operation still isn't well understood."
- Source: normankoren.com/Audio/Tubemodspice_article.html

The Koren equation only models plate current (Ip). Grid current (Ig) must be modeled separately.
