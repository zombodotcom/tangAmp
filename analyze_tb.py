"""
analyze_tb.py
Analyze wdf_triode testbench output.
Plots input vs output waveforms and FFT to verify harmonic distortion.
Run after simulation: python analyze_tb.py tb_output.txt
"""

import numpy as np
import matplotlib.pyplot as plt
import sys

FP_FRAC  = 16
FP_SCALE = 1 << FP_FRAC  # 65536

def fp_to_float(x):
    return x / FP_SCALE

# ── Load data ─────────────────────────────────────────────────────────────────
fname = sys.argv[1] if len(sys.argv) > 1 else "tb_output.txt"
try:
    data = np.loadtxt(fname, dtype=int)
    n, inp, out = data[:,0], data[:,1], data[:,2]
    print(f"Loaded {len(n)} samples from {fname}")
except Exception as e:
    print(f"Could not load {fname}: {e}")
    print("Run simulation first with: iverilog -o sim wdf_triode_tb.v wdf_triode.v && vvp sim")
    sys.exit(1)

inp_v = fp_to_float(inp)
out_v = fp_to_float(out)

# ── Time domain plot ──────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 1, figsize=(12, 10))
fig.suptitle("WDF Triode Stage — 440Hz Input", fontsize=13)

t = n / 48000.0 * 1000  # milliseconds

# Show first 10ms (about 4 cycles of 440Hz)
mask = t < 10

ax = axes[0]
ax.plot(t[mask], inp_v[mask], 'b', linewidth=1.5, label='Input (guitar)')
ax.set_ylabel("Voltage (V)")
ax.set_title("Input Signal — 440Hz, 0.5V")
ax.legend()
ax.grid(True, alpha=0.3)

ax = axes[1]
ax.plot(t[mask], out_v[mask], 'r', linewidth=1.5, label='Output (plate)')
ax.set_ylabel("Voltage (V)")
ax.set_title("Output Signal — After 12AX7 Stage")
ax.legend()
ax.grid(True, alpha=0.3)

ax = axes[2]
ax.plot(t[mask], inp_v[mask] / (max(abs(inp_v[mask]))+1e-9), 
        'b', linewidth=1, alpha=0.6, label='Input (normalised)')
ax.plot(t[mask], out_v[mask] / (max(abs(out_v[mask]))+1e-9), 
        'r', linewidth=1, alpha=0.6, label='Output (normalised)')
ax.set_xlabel("Time (ms)")
ax.set_ylabel("Normalised amplitude")
ax.set_title("Overlay — Waveshaping visible here")
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("waveforms.png", dpi=150)
print("Saved: waveforms.png")

# ── FFT — harmonic content ────────────────────────────────────────────────────
fig2, axes2 = plt.subplots(1, 2, figsize=(14, 5))
fig2.suptitle("Harmonic Content — Tube Nonlinearity", fontsize=13)

def plot_fft(ax, signal, fs=48000, title="", color='b'):
    N = len(signal)
    window = np.hanning(N)
    spectrum = np.abs(np.fft.rfft(signal * window))
    spectrum_db = 20 * np.log10(spectrum / (max(spectrum) + 1e-12) + 1e-12)
    freqs = np.fft.rfftfreq(N, 1/fs)
    ax.plot(freqs, spectrum_db, color=color, linewidth=0.8)
    ax.set_xlim(0, 5000)
    ax.set_ylim(-80, 5)
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Magnitude (dB)")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    # Mark harmonics
    for h in range(1, 8):
        ax.axvline(440*h, color='gray', alpha=0.3, linestyle='--', linewidth=0.5)
    return freqs, spectrum_db

plot_fft(axes2[0], inp_v, title="Input Spectrum", color='blue')
plot_fft(axes2[1], out_v, title="Output Spectrum (harmonics = tube saturation)", color='red')

axes2[1].annotate("2nd harmonic\n(even = warm)", 
                  xy=(880, -20), fontsize=8, color='darkred')
axes2[1].annotate("3rd harmonic\n(odd = grit)", 
                  xy=(1320, -30), fontsize=8, color='darkred')

plt.tight_layout()
plt.savefig("harmonics.png", dpi=150)
print("Saved: harmonics.png")

# ── Stats ─────────────────────────────────────────────────────────────────────
print(f"\nInput:  min={inp_v.min():.3f}V  max={inp_v.max():.3f}V  rms={np.sqrt(np.mean(inp_v**2)):.3f}V")
print(f"Output: min={out_v.min():.3f}V  max={out_v.max():.3f}V  rms={np.sqrt(np.mean(out_v**2)):.3f}V")
if np.sqrt(np.mean(inp_v**2)) > 0:
    gain_db = 20 * np.log10(np.sqrt(np.mean(out_v**2)) / np.sqrt(np.mean(inp_v**2)) + 1e-12)
    print(f"Gain:   {gain_db:.1f} dB")
print("\nTo simulate: iverilog -o sim wdf_triode_tb.v wdf_triode.v && vvp sim")
print("Then run:    python analyze_tb.py tb_output.txt")
