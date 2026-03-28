r"""
amp_sim_v2.py
Definitive full guitar amp simulator v2 -- all physics models integrated.

Signal chain per sample:
  input * gain -> [NFB subtraction] ->
  for each preamp stage:
    [Miller LPF] -> [Triode WDF (with grid current, 2x2 Jacobian)] ->
    [Coupling Cap (WDF, DC blocking accumulation)]
  -> [Tone Stack] -> master_volume ->
  [Power Amp Triode (with PSU sag-modulated VB)] ->
  [Output Transformer (bandpass + freq-dependent saturation)] ->
  [Cabinet IR] -> [NFB signal extraction] ->
  output

Physics models integrated:
  1. Grid current: Langmuir-Child Ig model, 2x2 Jacobian Newton solver
  2. Coupling cap blocking: WDF capacitor accumulates DC from asymmetric clipping
  3. Miller effect: 1st-order LPF between stages (parasitic Cgp * (1+Av))
  4. Power supply sag: rectifier model modulates VB per-sample
  5. Output transformer: bandpass + frequency-dependent core saturation
  6. Negative feedback: delayed output subtraction at preamp input

Self-contained -- no imports from other project modules.
"""

import numpy as np
import math
import os
import sys
import time
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

try:
    from scipy.io import wavfile
    from scipy.signal import firwin2
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

FS = 48000.0

# =============================================================================
# Tube Constants (Koren model)
# =============================================================================

TUBES = {
    "12AX7": dict(mu=100.0, ex=1.4, kg1=1060.0, kp=600.0, kvb=300.0),
    "12AU7": dict(mu=27.48, ex=1.03, kg1=466.13, kp=135.10, kvb=24224.55),
    "6SL7":  dict(mu=90.41, ex=1.25, kg1=597.32, kp=511.97, kvb=6747.79),
    "EL34":  dict(mu=10.98, ex=1.42, kg1=249.65, kp=43.2,   kvb=333.0),
    "6L6":   dict(mu=10.11, ex=1.37, kg1=406.6,  kp=31.2,   kvb=640.7),
    "300B":  dict(mu=3.95,  ex=1.4,  kg1=1550.0, kp=65.0,   kvb=300.0),
}

# =============================================================================
# Parasitic Capacitances (datasheet values for Miller effect)
# =============================================================================

PARASITIC_CAPS = {
    "12AX7": dict(Cgk=1.6e-12, Cgp=1.7e-12, Cpk=0.46e-12, Av=50.0),
    "12AU7": dict(Cgk=1.5e-12, Cgp=1.5e-12, Cpk=0.50e-12, Av=17.0),
    "6SL7":  dict(Cgk=3.0e-12, Cgp=3.2e-12, Cpk=1.1e-12,  Av=44.0),
    "EL34":  dict(Cgk=10.0e-12, Cgp=0.9e-12, Cpk=7.0e-12,  Av=5.0),
    "6L6":   dict(Cgk=10.0e-12, Cgp=0.9e-12, Cpk=7.0e-12,  Av=5.0),
    "300B":  dict(Cgk=7.5e-12,  Cgp=8.5e-12, Cpk=4.0e-12,  Av=3.5),
}

# =============================================================================
# Koren Plate Current Model
# =============================================================================

def koren_ip(vpk, vgk, mu, ex, kg1, kp, kvb):
    """Plate current Ip (amps) from Koren equation."""
    vpk = float(np.clip(vpk, 0.0, 600.0))
    vgk = float(np.clip(vgk, -50.0, 5.0))
    if vpk <= 0:
        return 0.0
    inner = kp * (1.0 / mu + vgk / math.sqrt(kvb + vpk * vpk))
    inner = float(np.clip(inner, -500, 500))
    Ed = (vpk / kp) * math.log1p(math.exp(inner))
    if Ed <= 0:
        return 0.0
    return (Ed ** ex) / kg1


def koren_dip_dvpk(vpk, vgk, tube, h=0.1):
    return (koren_ip(vpk + h, vgk, **tube) - koren_ip(vpk - h, vgk, **tube)) / (2 * h)


def koren_dip_dvgk(vpk, vgk, tube, h=0.1):
    return (koren_ip(vpk, vgk + h, **tube) - koren_ip(vpk, vgk - h, **tube)) / (2 * h)


# =============================================================================
# Grid Current Model (Langmuir-Child)
# =============================================================================

IG_MAX = 0.002     # Max grid current scaling (A)
VG_ONSET = 0.0     # Onset voltage (Vgk threshold)


def grid_current(vgk, ig_max=IG_MAX, vg_onset=VG_ONSET):
    """Grid current Ig (amps) from Langmuir-Child 3/2 power law."""
    if vgk <= vg_onset:
        return 0.0
    return ig_max * (vgk - vg_onset) ** 1.5


def grid_current_deriv(vgk, ig_max=IG_MAX, vg_onset=VG_ONSET):
    """dIg/dVgk for Newton-Raphson Jacobian."""
    if vgk <= vg_onset:
        return 0.0
    return 1.5 * ig_max * (vgk - vg_onset) ** 0.5


# =============================================================================
# Miller Effect LPF
# =============================================================================

def compute_miller_f3db(tube_name, r_source=100000.0):
    """Compute Miller -3dB frequency for given tube and source impedance."""
    caps = PARASITIC_CAPS[tube_name]
    c_miller = caps['Cgp'] * (1.0 + caps['Av'])
    c_total = caps['Cgk'] + c_miller
    return 1.0 / (2.0 * math.pi * r_source * c_total)


def miller_lpf_coeffs(f_3dB, fs=FS):
    """1st-order IIR LPF via bilinear transform. Returns (b0, b1, a1)."""
    wc = 2.0 * math.pi * f_3dB
    wd = 2.0 * fs * math.tan(wc / (2.0 * fs))
    alpha = wd / (2.0 * fs + wd)
    b0 = alpha
    b1 = alpha
    a1 = -(2.0 * fs - wd) / (2.0 * fs + wd)
    return b0, b1, a1


# =============================================================================
# Power Supply Sag Model
# =============================================================================

class PowerSupply:
    """Rectifier + filter cap power supply with sag and recovery."""

    PRESETS = {
        "none":         dict(sag_amount=0.0,  recovery_rate=0.01),
        "solid_state":  dict(sag_amount=0.1,  recovery_rate=0.005),
        "tube_gz34":    dict(sag_amount=0.35, recovery_rate=0.0008),
        "tube_5u4":     dict(sag_amount=0.5,  recovery_rate=0.0005),
    }

    def __init__(self, v_nominal=400.0, sag_amount=0.3, recovery_rate=0.001,
                 v_min_ratio=0.7):
        self.v_nominal = v_nominal
        self.v_supply = v_nominal
        self.sag_amount = sag_amount
        self.recovery_rate = recovery_rate
        self.v_min = v_nominal * v_min_ratio

    @classmethod
    def from_preset(cls, preset_name, v_nominal=400.0):
        if preset_name not in cls.PRESETS:
            raise ValueError(f"Unknown preset: {preset_name}")
        return cls(v_nominal=v_nominal, **cls.PRESETS[preset_name])

    def reset(self):
        self.v_supply = self.v_nominal

    def update(self, current_draw_ma):
        """Update supply voltage given current draw in mA. Returns voltage."""
        v_target = self.v_nominal - self.sag_amount * current_draw_ma
        v_target = max(v_target, self.v_min)
        self.v_supply += self.recovery_rate * (v_target - self.v_supply)
        return self.v_supply


# =============================================================================
# Output Transformer (bandpass + frequency-dependent saturation)
# =============================================================================

def biquad_highpass(fc, fs, Q=0.7071):
    w0 = 2.0 * math.pi * fc / fs
    alpha = math.sin(w0) / (2.0 * Q)
    cos_w0 = math.cos(w0)
    b0 = (1.0 + cos_w0) / 2.0
    b1 = -(1.0 + cos_w0)
    b2 = (1.0 + cos_w0) / 2.0
    a0 = 1.0 + alpha
    a1 = -2.0 * cos_w0
    a2 = 1.0 - alpha
    return np.array([b0/a0, b1/a0, b2/a0, 1.0, a1/a0, a2/a0])


def biquad_lowpass(fc, fs, Q=0.7071):
    w0 = 2.0 * math.pi * fc / fs
    alpha = math.sin(w0) / (2.0 * Q)
    cos_w0 = math.cos(w0)
    b0 = (1.0 - cos_w0) / 2.0
    b1 = 1.0 - cos_w0
    b2 = (1.0 - cos_w0) / 2.0
    a0 = 1.0 + alpha
    a1 = -2.0 * cos_w0
    a2 = 1.0 - alpha
    return np.array([b0/a0, b1/a0, b2/a0, 1.0, a1/a0, a2/a0])


def biquad_process(coeffs, x):
    """Apply biquad filter (Direct Form II Transposed)."""
    b0, b1, b2, _, a1, a2 = coeffs
    y = np.zeros_like(x)
    z1 = 0.0
    z2 = 0.0
    for n in range(len(x)):
        inp = x[n]
        out = b0 * inp + z1
        z1 = b1 * inp - a1 * out + z2
        z2 = b2 * inp - a2 * out
        y[n] = out
    return y


def output_transformer(samples, fs=FS, hpf_fc=60.0, lpf_fc=8000.0,
                        sat_threshold=1.0, sat_lf_depth=0.6,
                        lf_envelope_fc=200.0, impedance_ratio=50.0):
    """
    Output transformer: bandpass filtering + frequency-dependent saturation.
    Low frequencies saturate the core first (flux ~ V/f).
    """
    x = np.array(samples, dtype=np.float64)

    # Bandpass
    hpf_coeffs = biquad_highpass(hpf_fc, fs, Q=0.7071)
    x = biquad_process(hpf_coeffs, x)
    lpf_coeffs = biquad_lowpass(lpf_fc, fs, Q=0.7071)
    x = biquad_process(lpf_coeffs, x)

    # LF envelope for frequency-dependent saturation
    env_coeffs = biquad_lowpass(lf_envelope_fc, fs, Q=0.5)
    lf_envelope = biquad_process(env_coeffs, np.abs(x))

    peak = np.max(np.abs(x))
    if peak < 1e-12:
        return x

    env_norm = lf_envelope / peak
    dyn_threshold = sat_threshold * (1.0 - sat_lf_depth * np.clip(env_norm, 0, 1))

    x_norm = x / peak
    out = np.zeros_like(x)
    for n in range(len(x)):
        thresh = max(dyn_threshold[n], 0.05)
        out[n] = thresh * math.tanh(x_norm[n] / thresh)

    out *= peak
    turns_ratio = math.sqrt(impedance_ratio)
    out /= turns_ratio
    return out


# =============================================================================
# Biquad Tone Stack
# =============================================================================

def biquad_coeffs(ftype, fc, fs, gain_db=0.0, Q=0.707):
    A = 10.0 ** (gain_db / 40.0)
    w0 = 2.0 * math.pi * fc / fs
    cos_w0 = math.cos(w0)
    sin_w0 = math.sin(w0)
    alpha = sin_w0 / (2.0 * Q)

    if ftype == 'lowshelf':
        sqA = math.sqrt(A)
        b0 = A * ((A + 1) - (A - 1) * cos_w0 + 2 * sqA * alpha)
        b1 = 2 * A * ((A - 1) - (A + 1) * cos_w0)
        b2 = A * ((A + 1) - (A - 1) * cos_w0 - 2 * sqA * alpha)
        a0 = (A + 1) + (A - 1) * cos_w0 + 2 * sqA * alpha
        a1 = -2 * ((A - 1) + (A + 1) * cos_w0)
        a2 = (A + 1) + (A - 1) * cos_w0 - 2 * sqA * alpha
    elif ftype == 'highshelf':
        sqA = math.sqrt(A)
        b0 = A * ((A + 1) + (A - 1) * cos_w0 + 2 * sqA * alpha)
        b1 = -2 * A * ((A - 1) + (A + 1) * cos_w0)
        b2 = A * ((A + 1) + (A - 1) * cos_w0 - 2 * sqA * alpha)
        a0 = (A + 1) - (A - 1) * cos_w0 + 2 * sqA * alpha
        a1 = 2 * ((A - 1) - (A + 1) * cos_w0)
        a2 = (A + 1) - (A - 1) * cos_w0 - 2 * sqA * alpha
    elif ftype == 'peaking':
        b0 = 1 + alpha * A
        b1 = -2 * cos_w0
        b2 = 1 - alpha * A
        a0 = 1 + alpha / A
        a1 = -2 * cos_w0
        a2 = 1 - alpha / A
    else:
        raise ValueError(f"Unknown filter type: {ftype}")

    b = np.array([b0, b1, b2]) / a0
    a = np.array([1.0, a1 / a0, a2 / a0])
    return b, a


def apply_biquad(samples, b, a):
    n = len(samples)
    y = np.zeros(n)
    d1 = 0.0
    d2 = 0.0
    for i in range(n):
        x = samples[i]
        out = b[0] * x + d1
        d1 = b[1] * x - a[1] * out + d2
        d2 = b[2] * x - a[2] * out
        y[i] = out
    return y


def tone_stack(samples, bass, mid, treble, fs=FS):
    bass_db = (bass - 5.0) * 3.0
    mid_db = (mid - 5.0) * 2.4
    treble_db = (treble - 5.0) * 3.0

    b_bass, a_bass = biquad_coeffs('lowshelf', 200.0, fs, bass_db, Q=0.707)
    y = apply_biquad(samples, b_bass, a_bass)
    b_mid, a_mid = biquad_coeffs('peaking', 800.0, fs, mid_db, Q=0.7)
    y = apply_biquad(y, b_mid, a_mid)
    b_tre, a_tre = biquad_coeffs('highshelf', 3000.0, fs, treble_db, Q=0.707)
    y = apply_biquad(y, b_tre, a_tre)
    return y


# =============================================================================
# Cabinet IR
# =============================================================================

def _apply_peak(mag, freqs, fc, gain_db, Q):
    gain_lin = 10.0 ** (gain_db / 20.0)
    bw = fc / Q
    shape = 1.0 / (1.0 + ((freqs - fc) / (bw / 2.0)) ** 2)
    mag *= (1.0 + (gain_lin - 1.0) * shape)


def _apply_lpf(mag, freqs, fc):
    ratio = freqs / fc
    atten = 1.0 / np.sqrt(1.0 + ratio ** 4)
    mag *= atten


def make_cabinet_ir(cab_type, n_taps=257, fs=48000):
    nyq = fs / 2.0
    n_freqs = 512
    freqs_hz = np.linspace(0, nyq, n_freqs)
    mag = np.ones(n_freqs)

    if cab_type == '1x12_open':
        _apply_peak(mag, freqs_hz, 100.0, 6.0, 2.0)
        _apply_peak(mag, freqs_hz, 2500.0, 3.0, 1.5)
        _apply_lpf(mag, freqs_hz, 5000.0)
    elif cab_type == '2x12_open':
        _apply_peak(mag, freqs_hz, 90.0, 7.0, 2.5)
        _apply_peak(mag, freqs_hz, 2000.0, 2.5, 1.2)
        _apply_lpf(mag, freqs_hz, 5500.0)
    elif cab_type == '4x12_closed':
        _apply_peak(mag, freqs_hz, 80.0, 8.0, 3.0)
        _apply_peak(mag, freqs_hz, 3500.0, 2.0, 1.0)
        _apply_lpf(mag, freqs_hz, 4500.0)
    else:
        raise ValueError(f"Unknown cabinet type: {cab_type}")

    freqs_norm = freqs_hz / nyq
    if HAS_SCIPY:
        ir = firwin2(n_taps, freqs_norm, mag)
    else:
        full_mag = np.zeros(n_taps)
        n_half = n_taps // 2 + 1
        freq_interp = np.linspace(0, 1, n_half)
        mag_interp = np.interp(freq_interp, freqs_norm, mag)
        full_mag[:n_half] = mag_interp
        full_mag[n_half:] = mag_interp[-2:0:-1]
        ir = np.real(np.fft.ifft(full_mag))
        ir = np.roll(ir, n_taps // 2)

    ir *= np.hanning(n_taps)
    peak = np.max(np.abs(ir))
    if peak > 1e-12:
        ir /= peak
    return ir


def apply_cabinet(samples, ir_taps):
    return np.convolve(samples, ir_taps, mode='full')[:len(samples)]


# =============================================================================
# WAV output
# =============================================================================

def save_wav(filename, signal, fs=48000):
    if not HAS_SCIPY:
        print(f"  [scipy not available, skipping {filename}]")
        return
    peak = np.max(np.abs(signal))
    if peak > 1e-12:
        signal = signal / peak * 0.9
    sig16 = np.clip(signal * 32767, -32768, 32767).astype(np.int16)
    wavfile.write(filename, int(fs), sig16)
    print(f"  Saved: {filename}")


# =============================================================================
# THD measurement
# =============================================================================

def compute_thd(signal, fs, fundamental_freq, n_harmonics=8):
    """Compute THD as percentage."""
    N = len(signal)
    w = np.hanning(N)
    sp = np.abs(np.fft.rfft(signal * w))
    freqs = np.fft.rfftfreq(N, 1.0 / fs)
    search_width = max(3, int(N * 5 / fs))

    fund_idx = np.argmin(np.abs(freqs - fundamental_freq))
    lo = max(0, fund_idx - search_width)
    hi = min(len(sp), fund_idx + search_width + 1)
    fund_power = np.max(sp[lo:hi]) ** 2

    harm_power = 0.0
    for h in range(2, n_harmonics + 1):
        h_freq = fundamental_freq * h
        h_idx = np.argmin(np.abs(freqs - h_freq))
        lo = max(0, h_idx - search_width)
        hi = min(len(sp), h_idx + search_width + 1)
        harm_power += np.max(sp[lo:hi]) ** 2

    if fund_power < 1e-30:
        return 0.0
    return 100.0 * math.sqrt(harm_power / fund_power)


# =============================================================================
# AmpSimV2 -- Integrated Simulator Class
# =============================================================================

class AmpSimV2:
    """
    Complete guitar amplifier simulator v2 with all physics models.

    Per-sample signal chain:
      input -> NFB subtraction -> [preamp stages with Miller/grid current/blocking cap]
      -> tone stack -> master volume -> [power amp with PSU sag]
      -> [output transformer] -> [cabinet IR] -> NFB extraction -> output
    """

    # Preamp circuit constants
    PREAMP_VB = 200.0
    PREAMP_RP = 100000.0
    PREAMP_RG = 1000000.0
    PREAMP_RK = 1500.0
    PREAMP_CIN = 22e-9
    PREAMP_CK = 22e-6

    # Gain knob (0-10)
    GAIN_TABLE = {
        0: 0.5, 1: 1.0, 2: 2.0, 3: 3.5, 4: 5.0,
        5: 8.0, 6: 12.0, 7: 18.0, 8: 25.0, 9: 35.0, 10: 50.0,
    }
    MASTER_TABLE = {
        0: 0.01, 1: 0.03, 2: 0.06, 3: 0.10, 4: 0.16,
        5: 0.25, 6: 0.35, 7: 0.50, 8: 0.70, 9: 0.85, 10: 1.0,
    }

    def __init__(self, preset='fender_deluxe'):
        presets = {
            'fender_deluxe': dict(
                name='Fender Deluxe Reverb',
                preamp_stages=2, preamp_tube='12AX7',
                interstage_atten=20.0,
                tone_bass=6, tone_mid=5, tone_treble=7,
                power_tube='6L6', power_vb=400, power_rp=2000, power_rk=250,
                cabinet='1x12_open',
                input_gain=5, master_vol=5,
                # v2 additions
                nfb_amount=0.15,
                rectifier='tube_gz34',
                xf_hpf=60.0, xf_lpf=8000.0, xf_sat=0.8, xf_lf_depth=0.5,
            ),
            'marshall_jcm800': dict(
                name='Marshall JCM800',
                preamp_stages=3, preamp_tube='12AX7',
                interstage_atten=16.0,
                tone_bass=5, tone_mid=8, tone_treble=6,
                power_tube='EL34', power_vb=450, power_rp=1700, power_rk=200,
                cabinet='4x12_closed',
                input_gain=7, master_vol=6,
                nfb_amount=0.05,
                rectifier='solid_state',
                xf_hpf=50.0, xf_lpf=7000.0, xf_sat=0.7, xf_lf_depth=0.6,
            ),
            'vox_ac30': dict(
                name='Vox AC30',
                preamp_stages=2, preamp_tube='12AX7',
                interstage_atten=18.0,
                tone_bass=7, tone_mid=4, tone_treble=8,
                power_tube='EL34', power_vb=380, power_rp=1800, power_rk=220,
                cabinet='2x12_open',
                input_gain=6, master_vol=6,
                nfb_amount=0.0,   # Vox has NO NFB
                rectifier='tube_gz34',
                xf_hpf=70.0, xf_lpf=9000.0, xf_sat=0.75, xf_lf_depth=0.5,
            ),
            'mesa_dual_rec': dict(
                name='Mesa Dual Rectifier',
                preamp_stages=3, preamp_tube='12AX7',
                interstage_atten=14.0,
                tone_bass=8, tone_mid=3, tone_treble=7,
                power_tube='6L6', power_vb=420, power_rp=1900, power_rk=230,
                cabinet='4x12_closed',
                input_gain=8, master_vol=5,
                nfb_amount=0.03,
                rectifier='tube_5u4',   # heavy sag
                xf_hpf=50.0, xf_lpf=7500.0, xf_sat=0.65, xf_lf_depth=0.7,
            ),
            'fender_twin': dict(
                name='Fender Twin Clean',
                preamp_stages=1, preamp_tube='12AX7',
                interstage_atten=20.0,
                tone_bass=5, tone_mid=6, tone_treble=6,
                power_tube='6L6', power_vb=400, power_rp=2000, power_rk=250,
                cabinet='2x12_open',
                input_gain=3, master_vol=5,
                nfb_amount=0.25,          # heavy NFB, very clean
                rectifier='solid_state',
                xf_hpf=55.0, xf_lpf=9000.0, xf_sat=0.9, xf_lf_depth=0.4,
            ),
        }

        if preset not in presets:
            raise ValueError(f"Unknown preset: {preset}. Options: {list(presets.keys())}")

        p = presets[preset]
        for k, v in p.items():
            setattr(self, k, v)

        # Pre-build cabinet IR
        self.cab_ir = make_cabinet_ir(self.cabinet, n_taps=257, fs=int(FS))

        # Pre-compute derived constants
        self._init_preamp_constants()
        self._init_miller_coeffs()

    def _init_preamp_constants(self):
        """Pre-compute WDF preamp constants."""
        # Cathode bypass cap WDF port resistance
        R_CK = 1.0 / (2.0 * FS * self.PREAMP_CK)
        G_rk = 1.0 / self.PREAMP_RK
        G_ck = 1.0 / R_CK
        G_total = G_rk + G_ck
        self._R_cathode = 1.0 / G_total
        self._gamma_k = G_rk / G_total

        # Input coupling HP filter (for first stage from guitar)
        tau_hp = self.PREAMP_RG * self.PREAMP_CIN
        k_hp = 2.0 * FS * tau_hp
        self._c_hp = (k_hp - 1.0) / (k_hp + 1.0)
        self._hp_gain = (1.0 + self._c_hp) / 2.0

        # Coupling cap WDF constants (for interstage)
        R_cc = 1.0 / (2.0 * FS * self.PREAMP_CIN)
        self._gamma_cc = R_cc / (R_cc + self.PREAMP_RG)

    def _init_miller_coeffs(self):
        """Pre-compute Miller LPF coefficients for preamp tube."""
        f_3dB = compute_miller_f3db(self.preamp_tube, r_source=self.PREAMP_RP)
        self._miller_b0, self._miller_b1, self._miller_a1 = miller_lpf_coeffs(f_3dB)

    def process(self, audio_in, settle=2000):
        """
        Process audio through the complete amp chain with all physics models.

        Uses iterative NFB: run chain twice when NFB > 0. The second pass
        subtracts scaled/phase-corrected output from the preamp input.
        """
        n_total = len(audio_in)

        # Input gain
        gain_scale = self.GAIN_TABLE.get(self.input_gain, 8.0)
        x_raw = audio_in * gain_scale

        # Build power supply
        psu = PowerSupply.from_preset(self.rectifier, v_nominal=self.power_vb)

        # First pass: open-loop (or single pass if no NFB)
        nfb_signal = np.zeros(n_total)
        result_0 = self._run_chain(x_raw, nfb_signal, psu, settle)

        if self.nfb_amount > 0.0:
            # Compute NFB correction from first pass
            power_audio_0 = result_0[settle:]
            in_rms = np.sqrt(np.mean(x_raw[settle:] ** 2)) + 1e-12
            out_rms = np.sqrt(np.mean(power_audio_0 ** 2)) + 1e-12
            turns_ratio = out_rms / in_rms

            # Detect phase inversion
            check_len = min(4000, len(power_audio_0))
            corr = np.corrcoef(x_raw[settle:settle + check_len],
                               result_0[settle:settle + check_len])[0, 1]
            phase_sign = 1.0 if corr >= 0 else -1.0

            # Build NFB signal with 1-sample delay
            fb_normalized = np.zeros(n_total)
            fb_normalized[1:] = phase_sign * result_0[:-1] / turns_ratio
            nfb_signal = self.nfb_amount * fb_normalized

            # Second pass with NFB
            psu.reset()
            result_1 = self._run_chain(x_raw, nfb_signal, psu, settle)
            power_out = result_1
        else:
            power_out = result_0

        # Output transformer
        power_audio = power_out[settle:]
        xf_out = output_transformer(
            power_audio, fs=FS,
            hpf_fc=self.xf_hpf, lpf_fc=self.xf_lpf,
            sat_threshold=self.xf_sat, sat_lf_depth=self.xf_lf_depth)

        # Cabinet IR
        final = apply_cabinet(xf_out, self.cab_ir)
        return final

    def _run_chain(self, x_raw, nfb_signal, psu, settle):
        """
        Run the full signal chain (preamp + tone stack + power amp).
        Returns full-length power amp output (including settle).

        This is the core per-sample loop integrating:
          - NFB subtraction
          - Miller LPF per stage
          - Grid current in Newton solver
          - WDF coupling cap between stages
          - Power supply sag on power amp
        """
        n_total = len(x_raw)
        tube = TUBES[self.preamp_tube]
        power_tube = TUBES[self.power_tube]
        interstage_lin = 10.0 ** (self.interstage_atten / 20.0)
        master_scale = self.MASTER_TABLE.get(self.master_vol, 0.25)

        R_p = self.PREAMP_RP
        R_g = self.PREAMP_RG
        R_k = self._R_cathode

        n_stages = self.preamp_stages

        # === Per-stage state arrays ===
        # Input HP filter state (for first stage)
        hp_y = 0.0
        hp_x_prev = 0.0

        # Per-stage state
        prev_ip = [0.5e-3] * n_stages
        prev_ig = [0.0] * n_stages
        z_ck = [0.0] * n_stages          # cathode bypass cap state

        # Miller LPF state per stage (applied at input of stage 1+)
        miller_x_prev = [0.0] * n_stages
        miller_y_prev = [0.0] * n_stages

        # Coupling cap state per stage (between stages)
        z_cc = [0.0] * n_stages

        # Power amp state
        pa_hp_y = 0.0
        pa_hp_x_prev = 0.0
        pa_prev_ip = 1.0e-3

        # Power amp HP filter coefficients
        pa_rg = 1e6
        pa_cin = 22e-9
        pa_tau = pa_rg * pa_cin
        pa_k = 2.0 * FS * pa_tau
        pa_c_hp = (pa_k - 1.0) / (pa_k + 1.0)
        pa_hp_gain = (1.0 + pa_c_hp) / 2.0

        # Preamp output array (used for tone stack)
        preamp_out = np.zeros(n_total)
        power_out = np.zeros(n_total)

        # Preamp DC estimate for AC coupling (running average during settle)
        vp_dc_est = [self.PREAMP_VB / 2.0] * n_stages  # rough initial guess

        # =====================================================================
        # MAIN SAMPLE LOOP
        # =====================================================================
        progress_step = max(1, n_total // 10)
        for n in range(n_total):
            if n % progress_step == 0 and n > 0:
                sys.stdout.write('.')
                sys.stdout.flush()

            # --- Input with NFB subtraction ---
            x = x_raw[n] - nfb_signal[n]

            # --- PREAMP STAGES ---
            stage_input = x
            for s in range(n_stages):
                # --- Input coupling ---
                if s == 0:
                    # First stage: guitar input through coupling cap (simple HP)
                    hp_y = self._c_hp * hp_y + self._hp_gain * (stage_input - hp_x_prev)
                    hp_x_prev = stage_input
                    v_grid = hp_y
                else:
                    # Interstage: Miller LPF then WDF coupling cap (blocking)
                    # Miller LPF
                    m_out = (self._miller_b0 * stage_input +
                             self._miller_b1 * miller_x_prev[s] -
                             self._miller_a1 * miller_y_prev[s])
                    miller_x_prev[s] = stage_input
                    miller_y_prev[s] = m_out

                    vin_filtered = m_out

                    # WDF Coupling Cap (blocking distortion)
                    # Series adaptor: cap + Rg, driven by voltage source vin
                    b_cc = z_cc[s]
                    b_rg = 0.0
                    b_parent = -(b_cc + b_rg)
                    a_parent = 2.0 * vin_filtered - b_parent

                    xx = a_parent + b_cc + b_rg
                    a_cc = b_cc - self._gamma_cc * xx
                    a_rg = -(xx + a_cc)

                    v_grid = (a_rg + b_rg) / 2.0
                    z_cc[s] = a_cc

                # --- WDF Triode with Grid Current ---
                b_plate = self.PREAMP_VB
                b_grid = v_grid

                # Cathode bypass cap: parallel Rk || Ck
                b_rk = 0.0
                b_ck = z_ck[s]
                bDiff = b_ck - b_rk
                b_cathode = b_ck - self._gamma_k * bDiff

                a_p = b_plate
                a_g = b_grid
                a_k = b_cathode

                # Newton-Raphson: 2x2 system (Ip, Ig)
                Ip = prev_ip[s]
                Ig = prev_ig[s]

                for iteration in range(8):
                    Vpk = (a_p - a_k) - R_p * Ip - R_k * (Ip + Ig)
                    Vgk = (a_g - a_k) - R_g * Ig - R_k * (Ip + Ig)

                    ip_model = koren_ip(Vpk, Vgk, **tube)
                    ig_model = grid_current(Vgk)

                    f1 = Ip - ip_model
                    f2 = Ig - ig_model

                    if abs(f1) < 1e-10 and abs(f2) < 1e-10:
                        break

                    # Jacobian
                    dip_dvpk_val = koren_dip_dvpk(Vpk, Vgk, tube)
                    dip_dvgk_val = koren_dip_dvgk(Vpk, Vgk, tube)
                    dig_dvgk = grid_current_deriv(Vgk)

                    J11 = 1.0 + dip_dvpk_val * (R_p + R_k) + dip_dvgk_val * R_k
                    J12 = dip_dvpk_val * R_k + dip_dvgk_val * (R_g + R_k)
                    J21 = dig_dvgk * R_k
                    J22 = 1.0 + dig_dvgk * (R_g + R_k)

                    det = J11 * J22 - J12 * J21
                    if abs(det) < 1e-30:
                        break

                    dIp = (J22 * f1 - J12 * f2) / det
                    dIg = (J11 * f2 - J21 * f1) / det

                    Ip -= dIp
                    Ig -= dIg
                    Ip = max(Ip, 0.0)
                    Ig = max(Ig, 0.0)

                prev_ip[s] = Ip
                prev_ig[s] = Ig

                # Reflected waves
                b_p = a_p - 2.0 * R_p * Ip
                b_k = a_k + 2.0 * R_k * (Ip + Ig)

                # Update cathode bypass cap
                a_ck = b_k + b_cathode - b_ck
                z_ck[s] = a_ck

                # Plate voltage
                v_plate = (a_p + b_p) / 2.0

                # DC estimate (exponential average during settle, then fixed)
                if n < settle:
                    alpha_dc = 0.01
                    vp_dc_est[s] = vp_dc_est[s] * (1.0 - alpha_dc) + v_plate * alpha_dc

                # AC-coupled output for next stage
                ac_out = v_plate - vp_dc_est[s]

                if s < n_stages - 1:
                    stage_input = ac_out / interstage_lin
                else:
                    preamp_out[n] = ac_out

            # End of preamp stages for this sample

        # === TONE STACK (batch, audio portion) ===
        toned = tone_stack(preamp_out[settle:], self.tone_bass, self.tone_mid,
                           self.tone_treble, fs=FS)
        toned *= master_scale

        # === POWER AMP (per-sample with PSU sag) ===
        # Prepend settle zeros to toned signal
        pa_input = np.concatenate([np.zeros(settle), toned])

        for n in range(n_total):
            vin = pa_input[n]

            # HP coupling
            pa_hp_y = pa_c_hp * pa_hp_y + pa_hp_gain * (vin - pa_hp_x_prev)
            pa_hp_x_prev = vin
            v_grid = pa_hp_y

            # PSU sag: get current supply voltage
            vb_eff = psu.v_supply

            a_p = vb_eff
            a_g = v_grid
            a_k = 0.0

            Ip = pa_prev_ip
            for iteration in range(8):
                Vpk = (a_p - a_k) - (self.power_rp + self.power_rk) * Ip
                Vgk = a_g - a_k - self.power_rk * Ip
                ip_model = koren_ip(Vpk, Vgk, **power_tube)
                f_val = Ip - ip_model
                if abs(f_val) < 1e-9:
                    break
                dip_dvpk_v = koren_dip_dvpk(Vpk, Vgk, power_tube)
                dip_dvgk_v = koren_dip_dvgk(Vpk, Vgk, power_tube)
                df_dIp = 1.0 + dip_dvpk_v * (self.power_rp + self.power_rk) + dip_dvgk_v * self.power_rk
                if abs(df_dIp) < 1e-15:
                    break
                Ip -= f_val / df_dIp
                Ip = max(Ip, 0.0)

            pa_prev_ip = Ip
            v_plate = vb_eff - self.power_rp * Ip
            power_out[n] = v_plate

            # Feed current draw to PSU sag model
            psu.update(Ip * 1000.0)  # convert A to mA

        # DC-couple power amp output
        sl = slice(max(settle - 200, 0), settle)
        pa_dc = power_out[sl].mean()
        power_out -= pa_dc

        return power_out


# =============================================================================
# Main: Generate demos for all 5 presets
# =============================================================================

if __name__ == "__main__":
    demos_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "demos")
    os.makedirs(demos_dir, exist_ok=True)

    # Input: E power chord (82+123+165 Hz), 1.5 seconds
    duration = 1.5
    n_settle = 1000
    n_audio = int(duration * FS)
    n_total = n_settle + n_audio
    t = np.arange(n_total) / FS

    audio_in = np.zeros(n_total)
    audio_in[n_settle:] = (0.15 * np.sin(2 * np.pi * 82.0 * t[n_settle:]) +
                           0.15 * np.sin(2 * np.pi * 123.0 * t[n_settle:]) +
                           0.15 * np.sin(2 * np.pi * 165.0 * t[n_settle:]))

    preset_names = ['fender_deluxe', 'marshall_jcm800', 'vox_ac30',
                    'mesa_dual_rec', 'fender_twin']

    results = []

    print("=" * 70)
    print("AmpSim V2 -- Full Physics Integration")
    print("  Grid current, coupling cap blocking, Miller LPF,")
    print("  PSU sag, output transformer, negative feedback")
    print("=" * 70)

    for preset in preset_names:
        amp = AmpSimV2(preset)
        print(f"\n{'='*60}")
        print(f"Processing: {amp.name}")
        print(f"  Preamp: {amp.preamp_stages}x {amp.preamp_tube}, "
              f"gain={amp.input_gain}, master={amp.master_vol}")
        print(f"  Tone: B={amp.tone_bass} M={amp.tone_mid} T={amp.tone_treble}")
        print(f"  Power: {amp.power_tube} VB={amp.power_vb}V, cab={amp.cabinet}")
        print(f"  NFB={amp.nfb_amount}, rectifier={amp.rectifier}")
        print(f"  Transformer: HPF={amp.xf_hpf}Hz LPF={amp.xf_lpf}Hz "
              f"sat={amp.xf_sat} lf_depth={amp.xf_lf_depth}")
        print(f"{'='*60}")
        sys.stdout.flush()

        t0 = time.time()
        final = amp.process(audio_in, settle=n_settle)
        elapsed = time.time() - t0
        print()  # newline after progress dots

        peak = np.max(np.abs(final))
        rms = np.sqrt(np.mean(final ** 2))
        in_rms = np.sqrt(np.mean(audio_in[n_settle:] ** 2))
        gain_db = 20.0 * np.log10(rms / (in_rms + 1e-12) + 1e-12)
        thd = compute_thd(final, FS, 82.0)

        print(f"  Time: {elapsed:.1f}s")
        print(f"  Output peak={peak:.2f}, RMS={rms:.3f}")
        print(f"  Gain={gain_db:.1f}dB, THD={thd:.1f}%")

        # Save WAV
        wav_path = os.path.join(demos_dir, f"v2_{preset}.wav")
        save_wav(wav_path, final, int(FS))

        results.append({
            'name': amp.name,
            'preset': preset,
            'signal': final,
            'elapsed': elapsed,
            'gain_db': gain_db,
            'thd': thd,
            'peak': peak,
            'rms': rms,
        })

    # =========================================================================
    # Processing Stats Summary
    # =========================================================================
    print(f"\n{'='*70}")
    print("Processing Stats Summary")
    print(f"{'='*70}")
    print(f"{'Preset':<25s} {'Time(s)':>8s} {'Gain(dB)':>10s} {'THD(%)':>8s} {'Peak':>8s} {'RMS':>8s}")
    print("-" * 70)
    for r in results:
        print(f"{r['name']:<25s} {r['elapsed']:>8.1f} {r['gain_db']:>10.1f} "
              f"{r['thd']:>8.1f} {r['peak']:>8.2f} {r['rms']:>8.3f}")

    # =========================================================================
    # Comparison Plot
    # =========================================================================
    print(f"\nGenerating comparison plot...")
    n_presets = len(results)
    fig, axes = plt.subplots(n_presets, 2, figsize=(16, 4 * n_presets))
    fig.suptitle("AmpSim V2 -- All Physics Models\n"
                 "E Power Chord (82+123+165 Hz) | Grid Current, Blocking Cap, Miller LPF,\n"
                 "PSU Sag, Output Transformer, Negative Feedback",
                 fontsize=13, fontweight='bold')

    colors = ['#1f77b4', '#d62728', '#2ca02c', '#ff7f0e', '#9467bd']
    t_audio = np.arange(n_audio) / FS

    for i, r in enumerate(results):
        signal = r['signal']

        # Waveform (first 50ms)
        ax = axes[i, 0]
        mask = t_audio[:len(signal)] < 0.05
        ax.plot(t_audio[:len(signal)][mask] * 1000, signal[mask],
                color=colors[i], linewidth=0.8)
        ax.set_ylabel("Amplitude")
        ax.set_title(f"{r['name']} -- Waveform (gain={r['gain_db']:.0f}dB, THD={r['thd']:.0f}%)")
        ax.grid(True, alpha=0.3)
        if i == n_presets - 1:
            ax.set_xlabel("Time (ms)")

        # Spectrum
        ax = axes[i, 1]
        N = len(signal)
        w = np.hanning(N)
        sp = np.abs(np.fft.rfft(signal * w))
        sp_db = 20 * np.log10(sp / (np.max(sp) + 1e-12) + 1e-12)
        f = np.fft.rfftfreq(N, 1.0 / FS)
        ax.plot(f, sp_db, color=colors[i], linewidth=0.8)
        ax.set_xlim(0, 8000)
        ax.set_ylim(-80, 5)
        ax.set_ylabel("dB")
        nfb_str = f"NFB={r['preset']}"
        ax.set_title(f"{r['name']} -- Spectrum")
        ax.grid(True, alpha=0.3)
        for freq in [82, 123, 165]:
            ax.axvline(freq, color='gray', alpha=0.3, ls='--', lw=0.5)
        if i == n_presets - 1:
            ax.set_xlabel("Frequency (Hz)")

    plt.tight_layout()
    plot_path = os.path.join(demos_dir, "v2_amp_comparison.png")
    plt.savefig(plot_path, dpi=150)
    print(f"Saved: {plot_path}")
    plt.close()

    print("\nDone.")
