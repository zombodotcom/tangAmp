"""
gen_cab_taps.py
Generate cabinet IR FIR taps as a hex file for FPGA loading.
Outputs cab_ir.hex with Q1.15 signed 16-bit coefficients.
"""

import numpy as np
import sys
import os

# Add parent dir so we can import from cabinet_ir.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cabinet_ir import make_cabinet_ir

def main():
    n_taps = 129  # odd number required for Type I FIR (firwin2)
    fs = 48000
    cab_type = "1x12_open"

    print(f"Generating {cab_type} cabinet IR, {n_taps} taps, {fs}Hz...")
    taps = make_cabinet_ir(cab_type, n_taps=n_taps, fs=fs)

    # Quantize to Q1.15 (signed 16-bit, range -1.0 to +0.99997)
    taps_q15 = np.clip(taps * 32768, -32768, 32767).astype(np.int16)

    out_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "cab_ir.hex")
    with open(out_path, "w") as f:
        for t in taps_q15:
            f.write(f"{int(t) & 0xFFFF:04X}\n")

    print(f"Wrote {len(taps_q15)} taps to {out_path}")
    print(f"  Peak tap value: {np.max(np.abs(taps_q15))}")
    print(f"  Sum of |taps|: {np.sum(np.abs(taps_q15))}")

    # Verify readback
    with open(out_path, "r") as f:
        lines = f.readlines()
    print(f"  Hex file lines: {len(lines)}")

if __name__ == "__main__":
    main()
