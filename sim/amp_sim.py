r"""
amp_sim.py
Definitive full guitar amp simulator with all blocks:
  Input gain -> Preamp stages (12AX7 WDF) -> Tone stack -> Master volume ->
  Power amp (6L6/EL34 WDF) -> Cabinet IR -> Output

Preset-based: Fender Deluxe, Marshall JCM800, Vox AC30, Mesa Dual Rec, Fender Twin.
Generates WAV demos and comparison plots.
"""

import numpy as np
import os
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
# Tube Constants
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
# Koren Model (parameterized)
# =============================================================================

def koren_ip(vpk, vgk, mu, ex, kg1, kp, kvb):
    vpk = float(np.clip(vpk, 0.0, 600.0))
    vgk = float(np.clip(vgk, -50.0, 5.0))
    if vpk <= 0:
        return 0.0
    inner = kp * (1.0 / mu + vgk / np.sqrt(kvb + vpk * vpk))
    inner = np.clip(inner, -500, 500)
    Ed = (vpk / kp) * np.log1p(np.exp(float(inner)))
    if Ed <= 0:
        return 0.0
    return (Ed ** ex) / kg1


def koren_dip_dvpk(vpk, vgk, tube, h=0.1):
    return (koren_ip(vpk + h, vgk, **tube) - koren_ip(vpk - h, vgk, **tube)) / (2 * h)


def koren_dip_dvgk(vpk, vgk, tube, h=0.1):
    return (koren_ip(vpk, vgk + h, **tube) - koren_ip(vpk, vgk - h, **tube)) / (2 * h)


# =============================================================================
# Preamp Triode Stage (12AX7, with cathode bypass)
# =============================================================================

# Preamp circuit constants
PREAMP_VB  = 200.0
PREAMP_RP  = 100000.0
PREAMP_RG  = 1000000.0
PREAMP_RK  = 1500.0
PREAMP_CIN = 22e-9
PREAMP_CK  = 22e-6

# Derived
_tau_hp = PREAMP_RG * PREAMP_CIN
_k_hp = 2.0 * FS * _tau_hp
_c_hp = (_k_hp - 1.0) / (_k_hp + 1.0)
_hp_gain = (1.0 + _c_hp) / 2.0

_R_CK = 1.0 / (2.0 * FS * PREAMP_CK)
_G_rk = 1.0 / PREAMP_RK
_G_ck = 1.0 / _R_CK
_G_total = _G_rk + _G_ck
_R_cathode_bypass = 1.0 / _G_total
_gamma_k = _G_rk / _G_total


def simulate_preamp_stage(audio_in, tube_name='12AX7', use_bypass=True, settle=2000):
    """Simulate one preamp triode stage using WDF with Newton-Raphson."""
    tube = TUBES[tube_name]
    n_total = len(audio_in)
    out_vplate = np.zeros(n_total)

    R_p = PREAMP_RP
    R_k = _R_cathode_bypass if use_bypass else PREAMP_RK

    prev_ip = 0.5e-3
    hp_y = 0.0
    hp_x_prev = 0.0
    z_ck = 0.0

    for n in range(n_total):
        vin = audio_in[n]
        hp_y = _c_hp * hp_y + _hp_gain * (vin - hp_x_prev)
        hp_x_prev = vin
        v_grid = hp_y

        b_plate = PREAMP_VB
        b_grid = v_grid

        if use_bypass:
            b_rk = 0.0
            b_ck = z_ck
            bDiff = b_ck - b_rk
            b_cathode = b_ck - _gamma_k * bDiff
        else:
            b_cathode = 0.0

        a_p = b_plate
        a_g = b_grid
        a_k = b_cathode

        Ip = prev_ip
        for iteration in range(20):
            Vpk = (a_p - a_k) - (R_p + R_k) * Ip
            Vgk = a_g - a_k - R_k * Ip
            ip_model = koren_ip(Vpk, Vgk, **tube)
            f_val = Ip - ip_model
            if abs(f_val) < 1e-10:
                break
            dip_dvpk_v = koren_dip_dvpk(Vpk, Vgk, tube)
            dip_dvgk_v = koren_dip_dvgk(Vpk, Vgk, tube)
            df_dIp = 1.0 + dip_dvpk_v * (R_p + R_k) + dip_dvgk_v * R_k
            if abs(df_dIp) < 1e-15:
                break
            Ip -= f_val / df_dIp
            Ip = max(Ip, 0.0)

        prev_ip = Ip

        # Grid current: limits positive peaks when Vgk > 0
        # Clamp to max +2V (coupling cap charges prevent higher)
        Vgk_final = min(a_g - a_k - R_k * Ip, 2.0)
        Ig = 0.002 * max(0.0, Vgk_final) ** 1.5
        Ip_total = Ip + Ig

        b_p = a_p - 2.0 * R_p * Ip_total
        b_k = a_k + 2.0 * R_k * Ip_total
        v_plate = (a_p + b_p) / 2.0
        out_vplate[n] = v_plate

        if use_bypass:
            a_ck = b_k + b_cathode - b_ck
            z_ck = a_ck

    sl = slice(max(settle - 100, 0), settle)
    vp_dc = out_vplate[sl].mean()
    return out_vplate, vp_dc


# =============================================================================
# Power Amp Stage
# =============================================================================

def simulate_power_amp(audio_in, tube_name='6L6', vb=400.0, rp=2000.0, rk=250.0,
                       rg=1e6, cin=22e-9, sag_amount=0.3, clip_level=200.0,
                       fs=48000.0, settle=2000):
    """Power amp WDF triode stage with sag and transformer saturation."""
    tube = TUBES[tube_name]
    n_total = len(audio_in)
    out_vplate = np.zeros(n_total)

    tau_hp = rg * cin
    k_hp = 2.0 * fs * tau_hp
    c_hp = (k_hp - 1.0) / (k_hp + 1.0)
    hp_gain = (1.0 + c_hp) / 2.0

    sag_tau = 0.05
    sag_alpha = 1.0 / (1.0 + 2.0 * fs * sag_tau)
    sag_level = 0.0

    prev_ip = 1.0e-3
    hp_y = 0.0
    hp_x_prev = 0.0

    for n in range(n_total):
        vin = audio_in[n]
        hp_y = c_hp * hp_y + hp_gain * (vin - hp_x_prev)
        hp_x_prev = vin
        v_grid = hp_y

        vb_eff = vb - sag_amount * sag_level * 100.0

        a_p = vb_eff
        a_g = v_grid
        a_k = 0.0

        Ip = prev_ip
        for iteration in range(15):
            Vpk = (a_p - a_k) - (rp + rk) * Ip
            Vgk = a_g - a_k - rk * Ip
            ip_model = koren_ip(Vpk, Vgk, **tube)
            f_val = Ip - ip_model
            if abs(f_val) < 1e-9:
                break
            dip_dvpk_v = koren_dip_dvpk(Vpk, Vgk, tube)
            dip_dvgk_v = koren_dip_dvgk(Vpk, Vgk, tube)
            df_dIp = 1.0 + dip_dvpk_v * (rp + rk) + dip_dvgk_v * rk
            if abs(df_dIp) < 1e-15:
                break
            Ip -= f_val / df_dIp
            Ip = max(Ip, 0.0)

        prev_ip = Ip
        # Grid current
        Vgk_final = a_g - a_k - rk * Ip
        Ig = 0.002 * max(0.0, Vgk_final) ** 1.5
        Ip_total = Ip + Ig
        v_plate = vb_eff - rp * Ip_total
        out_vplate[n] = v_plate

        if n >= settle:
            sag_level = sag_level * (1.0 - sag_alpha) + abs(Ip) * sag_alpha

    sl = slice(max(settle - 100, 0), settle)
    vp_dc = out_vplate[sl].mean()
    ac_out = out_vplate - vp_dc
    # Grid current in the WDF solver now handles clipping naturally
    return ac_out, vp_dc


# =============================================================================
# Biquad Tone Stack (from full_chain_demo.py)
# =============================================================================

def biquad_coeffs(ftype, fc, fs, gain_db=0.0, Q=0.707):
    A = 10.0 ** (gain_db / 40.0)
    w0 = 2.0 * np.pi * fc / fs
    cos_w0 = np.cos(w0)
    sin_w0 = np.sin(w0)
    alpha = sin_w0 / (2.0 * Q)

    if ftype == 'lowshelf':
        sqA = np.sqrt(A)
        b0 = A * ((A + 1) - (A - 1) * cos_w0 + 2 * sqA * alpha)
        b1 = 2 * A * ((A - 1) - (A + 1) * cos_w0)
        b2 = A * ((A + 1) - (A - 1) * cos_w0 - 2 * sqA * alpha)
        a0 = (A + 1) + (A - 1) * cos_w0 + 2 * sqA * alpha
        a1 = -2 * ((A - 1) + (A + 1) * cos_w0)
        a2 = (A + 1) + (A - 1) * cos_w0 - 2 * sqA * alpha
    elif ftype == 'highshelf':
        sqA = np.sqrt(A)
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


def tone_stack(samples, bass, mid, treble, fs=48000):
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
# AmpSim Class
# =============================================================================

class AmpSim:
    """Complete guitar amplifier simulator."""

    # Gain knob (0-10) -> input voltage multiplier
    # 0 = unity, 10 = heavy overdrive
    GAIN_TABLE = {
        0: 0.5,  1: 1.0,  2: 2.0,  3: 3.5,  4: 5.0,
        5: 8.0,  6: 12.0, 7: 18.0, 8: 25.0, 9: 35.0, 10: 50.0,
    }

    # Master volume knob (0-10) -> linear attenuation
    MASTER_TABLE = {
        0: 0.01, 1: 0.03, 2: 0.06, 3: 0.10, 4: 0.16,
        5: 0.25, 6: 0.35, 7: 0.50, 8: 0.70, 9: 0.85, 10: 1.0,
    }

    def __init__(self, preset='fender_deluxe'):
        presets = {
            'fender_deluxe': dict(
                name='Fender Deluxe Reverb',
                preamp_stages=2, preamp_tube='12AX7',
                interstage_atten=0.35,  # real coupling network: Rgrid/(Rp+Rgrid) = -0.35dB
                tone_bass=6, tone_mid=5, tone_treble=7,
                power_tube='6L6', power_vb=400, power_rp=2000, power_rk=250,
                power_sag=0.3, power_clip=200,
                cabinet='1x12_open',
                input_gain=5, master_vol=5,
            ),
            'marshall_jcm800': dict(
                name='Marshall JCM800',
                preamp_stages=3, preamp_tube='12AX7',
                interstage_atten=0.35,  # real coupling network: Rgrid/(Rp+Rgrid) = -0.35dB
                tone_bass=5, tone_mid=8, tone_treble=6,
                power_tube='EL34', power_vb=450, power_rp=1700, power_rk=200,
                power_sag=0.2, power_clip=180,
                cabinet='4x12_closed',
                input_gain=7, master_vol=6,
            ),
            'vox_ac30': dict(
                name='Vox AC30',
                preamp_stages=2, preamp_tube='12AX7',
                interstage_atten=0.35,  # real coupling network: Rgrid/(Rp+Rgrid) = -0.35dB
                tone_bass=7, tone_mid=4, tone_treble=8,
                power_tube='EL34', power_vb=380, power_rp=1800, power_rk=220,
                power_sag=0.25, power_clip=170,
                cabinet='1x12_open',
                input_gain=6, master_vol=6,
            ),
            'mesa_dual_rec': dict(
                name='Mesa Dual Rectifier',
                preamp_stages=3, preamp_tube='12AX7',
                interstage_atten=0.35,  # real coupling network: Rgrid/(Rp+Rgrid) = -0.35dB
                tone_bass=8, tone_mid=3, tone_treble=7,
                power_tube='6L6', power_vb=420, power_rp=1900, power_rk=230,
                power_sag=0.4, power_clip=190,
                cabinet='4x12_closed',
                input_gain=8, master_vol=5,
            ),
            'fender_twin': dict(
                name='Fender Twin Clean',
                preamp_stages=1, preamp_tube='12AX7',
                interstage_atten=0.35,  # real coupling network: Rgrid/(Rp+Rgrid) = -0.35dB
                tone_bass=5, tone_mid=6, tone_treble=6,
                power_tube='6L6', power_vb=400, power_rp=2000, power_rk=250,
                power_sag=0.15, power_clip=250,
                cabinet='2x12_open',
                input_gain=3, master_vol=5,
            ),
        }

        if preset not in presets:
            raise ValueError(f"Unknown preset: {preset}. Choose from: {list(presets.keys())}")

        p = presets[preset]
        self.name = p['name']
        self.preamp_stages = p['preamp_stages']
        self.preamp_tube = p['preamp_tube']
        self.interstage_atten = p['interstage_atten']
        self.tone_bass = p['tone_bass']
        self.tone_mid = p['tone_mid']
        self.tone_treble = p['tone_treble']
        self.power_tube = p['power_tube']
        self.power_vb = p['power_vb']
        self.power_rp = p['power_rp']
        self.power_rk = p['power_rk']
        self.power_sag = p['power_sag']
        self.power_clip = p['power_clip']
        self.cabinet = p['cabinet']
        self.input_gain = p['input_gain']
        self.master_vol = p['master_vol']

        # Pre-build cabinet IR
        self.cab_ir = make_cabinet_ir(self.cabinet, n_taps=257, fs=int(FS))

    def set_gain(self, gain):
        """Set input gain knob (0-10)."""
        self.input_gain = max(0, min(10, int(round(gain))))

    def set_master(self, master):
        """Set master volume knob (0-10)."""
        self.master_vol = max(0, min(10, int(round(master))))

    def process(self, audio_in, settle=2000):
        """
        Process audio through full amp chain.

        Parameters
        ----------
        audio_in : ndarray
            Input signal (including settle samples at start).
        settle : int
            Number of settle samples.

        Returns
        -------
        final : ndarray
            Processed output (settle samples removed).
        """
        n_total = len(audio_in)

        # Input gain
        gain_scale = self.GAIN_TABLE.get(self.input_gain, 8.0)
        x = audio_in * gain_scale

        # Preamp stages
        interstage_lin = 10.0 ** (self.interstage_atten / 20.0)

        for stage in range(self.preamp_stages):
            vplate, vp_dc = simulate_preamp_stage(
                x, tube_name=self.preamp_tube, use_bypass=True, settle=settle)
            ac_out = vplate - vp_dc

            if stage < self.preamp_stages - 1:
                # Interstage coupling: minimal attenuation (real amps ~1dB)
                # Grid current in the next stage naturally limits the signal
                x = ac_out / interstage_lin
            else:
                x = ac_out

        # Tone stack (on audio portion only for efficiency, but apply to all)
        preamp_out = x[settle:]
        toned = tone_stack(preamp_out, self.tone_bass, self.tone_mid,
                           self.tone_treble, fs=FS)

        # Master volume
        master_scale = self.MASTER_TABLE.get(self.master_vol, 0.25)
        toned = toned * master_scale

        # Power amp: need to add settle samples back
        power_in = np.concatenate([np.zeros(settle), toned])
        power_out, _ = simulate_power_amp(
            power_in, tube_name=self.power_tube,
            vb=self.power_vb, rp=self.power_rp, rk=self.power_rk,
            sag_amount=self.power_sag, clip_level=self.power_clip,
            fs=FS, settle=settle)

        power_audio = power_out[settle:]

        # Cabinet IR
        final = apply_cabinet(power_audio, self.cab_ir)

        return final


# =============================================================================
# Main
# =============================================================================

if __name__ == "__main__":
    demos_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "demos")
    os.makedirs(demos_dir, exist_ok=True)

    # Input: E power chord (82+123+165 Hz) at 0.15V each, 3 seconds
    duration = 3.0
    n_settle = 2000
    n_audio = int(duration * FS)
    n_total = n_settle + n_audio
    t = np.arange(n_total) / FS

    audio_in = np.zeros(n_total)
    audio_in[n_settle:] = (0.15 * np.sin(2 * np.pi * 82.0 * t[n_settle:]) +
                           0.15 * np.sin(2 * np.pi * 123.0 * t[n_settle:]) +
                           0.15 * np.sin(2 * np.pi * 165.0 * t[n_settle:]))

    # =========================================================================
    # Generate all preset demos
    # =========================================================================
    preset_names = ['fender_deluxe', 'marshall_jcm800', 'vox_ac30',
                    'mesa_dual_rec', 'fender_twin']

    results = []

    for preset in preset_names:
        amp = AmpSim(preset)
        print(f"\n{'='*60}")
        print(f"Processing: {amp.name}")
        print(f"  Preamp: {amp.preamp_stages}x {amp.preamp_tube}, "
              f"gain={amp.input_gain}, master={amp.master_vol}")
        print(f"  Tone: B={amp.tone_bass} M={amp.tone_mid} T={amp.tone_treble}")
        print(f"  Power: {amp.power_tube} VB={amp.power_vb}V, cab={amp.cabinet}")
        print(f"{'='*60}")

        final = amp.process(audio_in, settle=n_settle)

        peak = np.max(np.abs(final))
        rms = np.sqrt(np.mean(final ** 2))
        print(f"  Output peak={peak:.1f}V, RMS={rms:.1f}V")

        fname = preset.replace('_', '_')
        wav_path = os.path.join(demos_dir, f"amp_{fname}.wav")
        save_wav(wav_path, final, int(FS))

        results.append((amp.name, final))

    # =========================================================================
    # Comparison plot
    # =========================================================================
    print("\nGenerating preset comparison plot...")
    n_presets = len(results)
    fig, axes = plt.subplots(n_presets, 2, figsize=(16, 4 * n_presets))
    fig.suptitle("Amp Presets -- E Power Chord (82+123+165 Hz)",
                 fontsize=14, fontweight='bold')

    colors = ['#1f77b4', '#d62728', '#2ca02c', '#ff7f0e', '#9467bd']
    t_audio = np.arange(n_audio) / FS

    for i, (name, signal) in enumerate(results):
        # Waveform (first 50ms)
        ax = axes[i, 0]
        mask = t_audio < 0.05
        ax.plot(t_audio[mask] * 1000, signal[mask], color=colors[i], linewidth=0.8)
        ax.set_ylabel("Amplitude")
        ax.set_title(f"{name} -- Waveform")
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
        ax.set_title(f"{name} -- Spectrum")
        ax.grid(True, alpha=0.3)
        for freq in [82, 123, 165]:
            ax.axvline(freq, color='gray', alpha=0.3, ls='--', lw=0.5)
        if i == n_presets - 1:
            ax.set_xlabel("Frequency (Hz)")

    plt.tight_layout()
    plot_path = os.path.join(demos_dir, "amp_presets.png")
    plt.savefig(plot_path, dpi=150)
    print(f"Saved: {plot_path}")
    plt.close()

    # =========================================================================
    # Marshall gain sweep
    # =========================================================================
    print(f"\n{'='*60}")
    print("Marshall JCM800 -- Gain Sweep")
    print("=" * 60)

    gain_levels = [1, 3, 5, 7, 10]
    gain_results = []

    for gain in gain_levels:
        amp = AmpSim('marshall_jcm800')
        amp.set_gain(gain)
        amp.set_master(5)

        print(f"\n  Gain={gain} ...")
        final = amp.process(audio_in, settle=n_settle)
        peak = np.max(np.abs(final))
        rms = np.sqrt(np.mean(final ** 2))
        print(f"    Output peak={peak:.1f}V, RMS={rms:.1f}V")

        wav_path = os.path.join(demos_dir, f"amp_marshall_gain{gain}.wav")
        save_wav(wav_path, final, int(FS))
        gain_results.append((gain, final))

    # Gain sweep plot
    fig, axes = plt.subplots(len(gain_levels), 2, figsize=(16, 4 * len(gain_levels)))
    fig.suptitle("Marshall JCM800 -- Gain Sweep (1=clean to 10=full overdrive)",
                 fontsize=14, fontweight='bold')

    for i, (gain, signal) in enumerate(gain_results):
        ax = axes[i, 0]
        mask = t_audio < 0.05
        ax.plot(t_audio[mask] * 1000, signal[mask], color='#d62728', linewidth=0.8)
        ax.set_ylabel("Amplitude")
        ax.set_title(f"Gain={gain} -- Waveform")
        ax.grid(True, alpha=0.3)
        if i == len(gain_levels) - 1:
            ax.set_xlabel("Time (ms)")

        ax = axes[i, 1]
        N = len(signal)
        w = np.hanning(N)
        sp = np.abs(np.fft.rfft(signal * w))
        sp_db = 20 * np.log10(sp / (np.max(sp) + 1e-12) + 1e-12)
        f = np.fft.rfftfreq(N, 1.0 / FS)
        ax.plot(f, sp_db, color='#d62728', linewidth=0.8)
        ax.set_xlim(0, 8000)
        ax.set_ylim(-80, 5)
        ax.set_ylabel("dB")
        ax.set_title(f"Gain={gain} -- Spectrum")
        ax.grid(True, alpha=0.3)
        for freq in [82, 123, 165]:
            ax.axvline(freq, color='gray', alpha=0.3, ls='--', lw=0.5)
        if i == len(gain_levels) - 1:
            ax.set_xlabel("Frequency (Hz)")

    plt.tight_layout()
    plot_path = os.path.join(demos_dir, "amp_marshall_gain_sweep.png")
    plt.savefig(plot_path, dpi=150)
    print(f"\nSaved: {plot_path}")
    plt.close()

    print("\nDone.")
