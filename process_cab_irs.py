#!/usr/bin/env python3
"""
Process downloaded cabinet impulse response WAV files for FPGA use.
- Resample to 48kHz
- Truncate to 256 samples (5.3ms at 48kHz)
- Normalize peak to 1.0
- Quantize to Q1.15 (signed 16-bit)
- Write hex files for Verilog $readmemh
"""

import numpy as np
import os
import glob
from scipy.io import wavfile
from scipy.signal import resample_poly
from math import gcd

TARGET_SR = 48000
TARGET_LEN = 256  # 5.3ms at 48kHz

def load_wav(path):
    sr, data = wavfile.read(path)
    if data.dtype == np.int16:
        data = data.astype(np.float64) / 32768.0
    elif data.dtype == np.int32:
        data = data.astype(np.float64) / 2147483648.0
    elif data.dtype == np.float32 or data.dtype == np.float64:
        data = data.astype(np.float64)
    else:
        data = data.astype(np.float64) / max(np.max(np.abs(data)), 1e-12)
    if data.ndim > 1:
        data = data[:, 0]
    return sr, data

def resample_to_48k(sr, data):
    if sr == TARGET_SR:
        return data
    g = gcd(sr, TARGET_SR)
    up = TARGET_SR // g
    down = sr // g
    return resample_poly(data, up, down)

def process_ir(data):
    ir = data[:TARGET_LEN].copy()
    if len(ir) < TARGET_LEN:
        ir = np.pad(ir, (0, TARGET_LEN - len(ir)))
    peak = np.max(np.abs(ir))
    if peak > 0:
        ir = ir / peak
    return ir

def quantize_q15(ir):
    scaled = np.clip(ir * 32767.0, -32768, 32767)
    return scaled.astype(np.int16)

def write_hex(ir_q15, path):
    with open(path, "w") as f:
        for val in ir_q15:
            uval = int(val) & 0xFFFF
            f.write(f"{uval:04x}\n")

def analyze_freq_response(ir, sr, name):
    fft_data = np.fft.rfft(ir, n=1024)
    mag = 20 * np.log10(np.abs(fft_data) + 1e-12)
    freqs = np.fft.rfftfreq(1024, 1.0/sr)

    peak_idx = np.argmax(mag)
    peak_freq = freqs[peak_idx]
    peak_db = mag[peak_idx]

    above_3db = freqs[mag > peak_db - 3]
    if len(above_3db) > 1:
        bw_low, bw_high = above_3db[0], above_3db[-1]
    else:
        bw_low = bw_high = peak_freq

    def band_energy(f_low, f_high):
        mask = (freqs >= f_low) & (freqs <= f_high)
        return np.mean(mag[mask]) if np.any(mask) else -60

    print(f"\n  {name}:")
    print(f"    Peak frequency: {peak_freq:.0f} Hz ({peak_db:.1f} dB)")
    print(f"    -3dB bandwidth: {bw_low:.0f} - {bw_high:.0f} Hz")
    print(f"    Bass (80-300 Hz):  {band_energy(80, 300):.1f} dB")
    print(f"    Mids (300-2k Hz):  {band_energy(300, 2000):.1f} dB")
    print(f"    Presence (2-5k Hz): {band_energy(2000, 5000):.1f} dB")
    print(f"    Treble (5-10k Hz): {band_energy(5000, 10000):.1f} dB")


def main():
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    cab_ir_dir = "cab_irs"

    print("=" * 60)
    print("PROCESSING REAL CABINET IMPULSE RESPONSES")
    print("=" * 60)

    ir_selections = {
        "4x12_V30_SM57": os.path.join(cab_ir_dir, "Science 4x12 IRs", "Science 4x12 V30 8 Ohm", "V30 SM57", "Science 4x12 V30 SM57 Brighter.wav"),
        "4x12_V30_MD421": os.path.join(cab_ir_dir, "Science 4x12 IRs", "Science 4x12 V30 8 Ohm", "V30 MD 421-U", "Science 4x12 V30 MD 421-U Brighter.wav"),
        "4x12_G12H75_SM57": os.path.join(cab_ir_dir, "Science 4x12 IRs", "Science 4x12 G12H-75 8 Ohm", "Science 4x12 G12H-75 SM57", "Science 4x12 G12H-75 SM57 Brighter.wav"),
        "4x12_G12H150_SM57": os.path.join(cab_ir_dir, "Science 4x12 IRs", "Science 4x12 G12H-150 8 Ohm", "Science 4x12 G12H-150 SM57", "Science 4x12 G12H-150 SM57 Brighter.wav"),
        "4x12_V30_SM57_dark": os.path.join(cab_ir_dir, "Science 4x12 IRs", "Science 4x12 V30 8 Ohm", "V30 SM57", "Science 4x12 V30 SM57 Darker.wav"),
        "4x12_G12H75_SM57_dark": os.path.join(cab_ir_dir, "Science 4x12 IRs", "Science 4x12 G12H-75 8 Ohm", "Science 4x12 G12H-75 SM57", "Science 4x12 G12H-75 SM57 Darker.wav"),
    }

    # Add Djammincabs selections
    for i in [1, 10, 25, 50]:
        fname = f"Djammincabs Guitar IR -{i:03d}.wav"
        path = os.path.join(cab_ir_dir, "djammincabs", fname)
        if os.path.exists(path):
            ir_selections[f"djammin_{i:03d}"] = path

    processed_count = 0
    print(f"\nSelected {len(ir_selections)} IRs for processing:")

    for name, path in ir_selections.items():
        if not os.path.exists(path):
            print(f"  SKIP: {name} -- file not found: {path}")
            continue

        try:
            sr, data = load_wav(path)
            print(f"\n  Loading {name}: {sr}Hz, {len(data)} samples, {len(data)/sr*1000:.1f}ms")

            data_48k = resample_to_48k(sr, data)
            ir = process_ir(data_48k)
            ir_q15 = quantize_q15(ir)

            hex_path = f"cab_ir_{name}.hex"
            write_hex(ir_q15, hex_path)
            print(f"    -> {hex_path} ({len(ir_q15)} samples, Q1.15)")

            analyze_freq_response(ir, TARGET_SR, name)
            processed_count += 1

        except Exception as e:
            print(f"  ERROR processing {name}: {e}")
            import traceback
            traceback.print_exc()

    # DD4WH embedded IR (1x12 style, 44.1kHz, 512 samples)
    print("\n" + "=" * 60)
    print("PROCESSING DD4WH EMBEDDED CABINET IR (1x12 style, 44.1kHz)")
    print("=" * 60)

    dd4wh_coeffs = [
        0.21631,0.34689,0.29825,0.35770,0.18213,0.10291,-0.07526,-0.14740,
        -0.15997,-0.12085,-0.01587,0.03448,0.09750,0.07587,0.06842,0.01251,
        -0.00076,-0.01514,0.00522,0.02310,0.03232,0.02982,0.00797,-0.00076,
        -0.01340,0.00076,0.00967,0.03192,0.03677,0.03711,0.02206,0.00372,
        -0.01047,-0.01898,-0.01151,-0.00232,0.01740,0.02814,0.03976,0.03912,
        0.03671,0.02832,0.01993,0.01300,0.00494,0.00061,-0.00784,-0.01205,
        -0.02036,-0.02301,-0.02628,-0.02365,-0.01917,-0.01233,-0.00424,0.00049,
        0.00540,0.00458,0.00516,0.00177,0.00146,-0.00031,-0.00003,-0.00052,
        -0.00192,-0.00336,-0.00726,-0.00912,-0.01282,-0.01294,-0.01410,-0.01245,
        -0.01178,-0.01068,-0.01010,-0.01086,-0.01068,-0.01245,-0.01196,-0.01349,
        -0.01276,-0.01392,-0.01382,-0.01492,-0.01602,-0.01706,-0.01910,-0.01971,
        -0.02206,-0.02222,-0.02420,-0.02390,-0.02493,-0.02420,-0.02393,-0.02295,
        -0.02155,-0.02051,-0.01855,-0.01782,-0.01611,-0.01605,-0.01535,-0.01590,
        -0.01633,-0.01703,-0.01807,-0.01843,-0.01956,-0.01941,-0.02023,-0.01978,
        -0.02008,-0.01956,-0.01920,-0.01868,-0.01773,-0.01746,-0.01627,-0.01627,
        -0.01520,-0.01535,-0.01462,-0.01474,-0.01447,-0.01447,-0.01474,-0.01459,
        -0.01523,-0.01483,-0.01547,-0.01492,-0.01523,-0.01465,-0.01456,-0.01416,
        -0.01373,-0.01358,-0.01285,-0.01288,-0.01202,-0.01221,-0.01154,-0.01178,
        -0.01154,-0.01175,-0.01196,-0.01199,-0.01254,-0.01233,-0.01291,-0.01257,
        -0.01303,-0.01263,-0.01282,-0.01251,-0.01233,-0.01218,-0.01172,-0.01172,
        -0.01108,-0.01117,-0.01056,-0.01065,-0.01022,-0.01016,-0.00998,-0.00967,
        -0.00964,-0.00909,-0.00912,-0.00842,-0.00842,-0.00772,-0.00760,-0.00705,
        -0.00671,-0.00635,-0.00580,-0.00565,-0.00494,-0.00491,-0.00421,-0.00418,
        -0.00360,-0.00348,-0.00308,-0.00284,-0.00269,-0.00232,-0.00235,-0.00192,
        -0.00198,-0.00159,-0.00159,-0.00125,-0.00116,-0.00095,-0.00067,-0.00064,
        -0.00027,-0.00031,0.00003,0.00000,0.00027,0.00027,0.00049,0.00061,
        0.00067,0.00095,0.00095,0.00131,0.00125,0.00162,0.00162,0.00192,
        0.00198,0.00217,0.00238,0.00244,0.00278,0.00275,0.00311,0.00308,
        0.00342,0.00345,0.00369,0.00378,0.00391,0.00412,0.00409,0.00436,
        0.00427,0.00455,0.00446,0.00467,0.00467,0.00479,0.00488,0.00491,
        0.00510,0.00507,0.00531,0.00522,0.00546,0.00543,0.00562,0.00562,
        0.00568,0.00577,0.00574,0.00589,0.00583,0.00598,0.00589,0.00604,
        0.00598,0.00607,0.00610,0.00607,0.00620,0.00610,0.00623,0.00610,
        0.00626,0.00613,0.00623,0.00613,0.00613,0.00613,0.00607,0.00610,
        0.00598,0.00607,0.00592,0.00601,0.00589,0.00589,0.00583,0.00580,
        0.00580,0.00571,0.00577,0.00562,0.00568,0.00555,0.00555,0.00546,
        0.00543,0.00537,0.00528,0.00528,0.00513,0.00516,0.00500,0.00500,
        0.00488,0.00488,0.00479,0.00473,0.00470,0.00458,0.00458,0.00443,
        0.00443,0.00430,0.00427,0.00418,0.00412,0.00403,0.00394,0.00388,
        0.00375,0.00375,0.00360,0.00360,0.00345,0.00342,0.00333,0.00323,
        0.00317,0.00308,0.00305,0.00293,0.00290,0.00278,0.00275,0.00262,
        0.00256,0.00247,0.00238,0.00232,0.00220,0.00217,0.00204,0.00201,
        0.00189,0.00183,0.00174,0.00165,0.00159,0.00150,0.00143,0.00134,
        0.00128,0.00116,0.00113,0.00101,0.00095,0.00089,0.00079,0.00073,
        0.00064,0.00061,0.00049,0.00046,0.00037,0.00034,0.00024,0.00018,
        0.00012,0.00006,0.00000,-0.00003,-0.00006,-0.00015,-0.00018,-0.00024,
        -0.00027,-0.00034,-0.00037,-0.00043,-0.00046,-0.00049,-0.00055,-0.00058,
        -0.00064,-0.00064,-0.00070,-0.00070,-0.00076,-0.00079,-0.00082,-0.00085,
        -0.00085,-0.00089,-0.00089,-0.00095,-0.00095,-0.00098,-0.00098,-0.00101,
        -0.00104,-0.00104,-0.00104,-0.00104,-0.00107,-0.00107,-0.00110,-0.00110,
        -0.00110,-0.00110,-0.00110,-0.00113,-0.00110,-0.00113,-0.00110,-0.00113,
        -0.00113,-0.00113,-0.00113,-0.00113,-0.00110,-0.00110,-0.00110,-0.00110,
        -0.00110,-0.00110,-0.00110,-0.00107,-0.00107,-0.00107,-0.00104,-0.00104,
        -0.00104,-0.00101,-0.00101,-0.00101,-0.00098,-0.00098,-0.00095,-0.00095,
        -0.00095,-0.00092,-0.00092,-0.00089,-0.00089,-0.00085,-0.00085,-0.00082,
        -0.00082,-0.00082,-0.00079,-0.00079,-0.00076,-0.00076,-0.00073,-0.00073,
        -0.00070,-0.00070,-0.00067,-0.00067,-0.00064,-0.00064,-0.00061,-0.00061,
        -0.00058,-0.00058,-0.00055,-0.00055,-0.00052,-0.00052,-0.00049,-0.00049,
        -0.00049,-0.00046,-0.00046,-0.00043,-0.00043,-0.00040,-0.00040,-0.00040,
        -0.00037,-0.00037,-0.00034,-0.00034,-0.00034,-0.00031,-0.00031,-0.00027,
        -0.00027,-0.00027,-0.00024,-0.00024,-0.00024,-0.00021,-0.00021,-0.00021,
        -0.00018,-0.00018,-0.00018,-0.00015,-0.00015,-0.00015,-0.00012,-0.00012,
        -0.00012,-0.00012,-0.00009,-0.00009,-0.00009,-0.00006,-0.00006,-0.00006,
        -0.00006,-0.00003,-0.00003,-0.00003,-0.00003,-0.00003,0.00000,0.00000
    ]

    data_441 = np.array(dd4wh_coeffs, dtype=np.float64)
    print(f"DD4WH IR: 44100Hz, {len(data_441)} samples")

    data_48k = resample_to_48k(44100, data_441)
    print(f"  Resampled: {len(data_48k)} samples at 48kHz")

    ir = process_ir(data_48k)
    ir_q15 = quantize_q15(ir)
    write_hex(ir_q15, "cab_ir_1x12_dd4wh.hex")
    print(f"  -> cab_ir_1x12_dd4wh.hex (256 samples, Q1.15)")
    analyze_freq_response(ir, TARGET_SR, "1x12_dd4wh (embedded)")
    processed_count += 1

    # Summary
    print("\n" + "=" * 60)
    print(f"TOTAL: {processed_count} cabinet IR hex files generated")
    print("=" * 60)

    hex_files = sorted(glob.glob("cab_ir_*.hex"))
    for f in hex_files:
        size = os.path.getsize(f)
        print(f"  {f:45s} {size:6d} bytes")


if __name__ == "__main__":
    main()
