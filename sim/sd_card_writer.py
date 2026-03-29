#!/usr/bin/env python3
"""
sd_card_writer.py -- Write cabinet IR hex files to a raw SD card image.

Generates a binary image file with the tangAmp IR format:
  Sector 0: header (magic "TAIR", count, names)
  Sector 1+: IR data (256 x 16-bit LE per sector, padded to 512 bytes)

The image can be written to an SD card with:
  dd if=tangamp_irs.img of=/dev/sdX bs=512 conv=notrunc

Usage:
  python sd_card_writer.py --output tangamp_irs.img \
      --ir data/cab_ir.hex "V30 SM57" \
      --ir data/cab_ir_4x12_G12H75_SM57.hex "G12H75 SM57" \
      --default 0

  python sd_card_writer.py --output tangamp_irs.img --auto
      (auto mode: loads all data/cab_ir_*.hex files)
"""

import argparse
import struct
import sys
from pathlib import Path


SECTOR_SIZE = 512
MAGIC = b"TAIR"
VERSION = 1
TAPS_PER_IR = 256
BITS_PER_TAP = 16
MAX_IRS = 15  # 15 names fit in one header sector
NAME_SIZE = 32  # bytes per IR name slot


def read_hex_file(path: Path) -> list[int]:
    """Read a hex file of 16-bit tap values. Returns list of signed 16-bit ints."""
    taps = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("//"):
                continue
            val = int(line, 16) & 0xFFFF
            # Interpret as signed Q1.15
            if val >= 0x8000:
                val -= 0x10000
            taps.append(val)
    return taps


def taps_to_sector(taps: list[int], num_taps: int = TAPS_PER_IR) -> bytes:
    """Convert tap values to a 512-byte sector (16-bit little-endian)."""
    # Pad or truncate to num_taps
    if len(taps) < num_taps:
        taps = taps + [0] * (num_taps - len(taps))
    elif len(taps) > num_taps:
        taps = taps[:num_taps]

    data = b""
    for t in taps:
        # Pack as signed 16-bit little-endian
        data += struct.pack("<h", t)

    # Pad to sector size
    data += b"\x00" * (SECTOR_SIZE - len(data))
    return data


def build_header(num_irs: int, default_ir: int, names: list[str]) -> bytes:
    """Build the 512-byte header sector."""
    hdr = bytearray(SECTOR_SIZE)

    # Magic
    hdr[0:4] = MAGIC

    # Version (uint16 LE)
    struct.pack_into("<H", hdr, 4, VERSION)

    # Number of IRs (uint16 LE)
    struct.pack_into("<H", hdr, 6, num_irs)

    # Taps per IR (uint16 LE)
    struct.pack_into("<H", hdr, 8, TAPS_PER_IR)

    # Bits per tap (uint16 LE)
    struct.pack_into("<H", hdr, 10, BITS_PER_TAP)

    # Default IR index (uint16 LE)
    struct.pack_into("<H", hdr, 12, default_ir)

    # Reserved (uint16 LE)
    struct.pack_into("<H", hdr, 14, 0)

    # IR names (32 bytes each, starting at offset 0x10)
    for i, name in enumerate(names[:MAX_IRS]):
        offset = 0x10 + i * NAME_SIZE
        name_bytes = name.encode("ascii", errors="replace")[:NAME_SIZE - 1]
        hdr[offset:offset + len(name_bytes)] = name_bytes
        # Already null-terminated (bytearray is zero-filled)

    return bytes(hdr)


def auto_discover_irs(data_dir: Path) -> list[tuple[Path, str]]:
    """Find all cab_ir hex files and generate names from filenames."""
    irs = []
    for p in sorted(data_dir.glob("cab_ir*.hex")):
        # Generate name from filename: cab_ir_4x12_V30_SM57.hex -> "4x12 V30 SM57"
        stem = p.stem
        if stem.startswith("cab_ir_"):
            name = stem[7:].replace("_", " ")
        elif stem == "cab_ir":
            name = "Default"
        else:
            name = stem.replace("_", " ")
        irs.append((p, name))
    return irs


def main():
    parser = argparse.ArgumentParser(
        description="Write tangAmp cabinet IR data to a raw SD card image"
    )
    parser.add_argument("--output", "-o", required=True, help="Output image file path")
    parser.add_argument(
        "--ir",
        nargs=2,
        action="append",
        metavar=("HEX_FILE", "NAME"),
        help="Add an IR: hex file path and display name",
    )
    parser.add_argument("--default", "-d", type=int, default=0, help="Default IR index (0-based)")
    parser.add_argument(
        "--auto",
        action="store_true",
        help="Auto-discover all data/cab_ir_*.hex files",
    )

    args = parser.parse_args()

    # Collect IR entries
    ir_entries: list[tuple[Path, str]] = []

    if args.auto:
        # Auto-discover from data/ directory
        script_dir = Path(__file__).parent
        data_dir = script_dir.parent / "data"
        ir_entries = auto_discover_irs(data_dir)
        if not ir_entries:
            print(f"ERROR: No cab_ir*.hex files found in {data_dir}", file=sys.stderr)
            sys.exit(1)
        print(f"Auto-discovered {len(ir_entries)} IRs from {data_dir}")
    elif args.ir:
        for hex_path_str, name in args.ir:
            hex_path = Path(hex_path_str)
            if not hex_path.exists():
                # Try relative to script's parent (project root)
                alt = Path(__file__).parent.parent / hex_path_str
                if alt.exists():
                    hex_path = alt
                else:
                    print(f"ERROR: File not found: {hex_path_str}", file=sys.stderr)
                    sys.exit(1)
            ir_entries.append((hex_path, name))
    else:
        print("ERROR: Specify --ir entries or use --auto", file=sys.stderr)
        sys.exit(1)

    if len(ir_entries) > MAX_IRS:
        print(f"WARNING: Truncating to {MAX_IRS} IRs (got {len(ir_entries)})", file=sys.stderr)
        ir_entries = ir_entries[:MAX_IRS]

    default_idx = min(args.default, len(ir_entries) - 1)

    # Build image
    names = [name for _, name in ir_entries]
    header = build_header(len(ir_entries), default_idx, names)

    sectors = [header]
    for hex_path, name in ir_entries:
        taps = read_hex_file(hex_path)
        print(f"  IR {len(sectors):2d}: {name:30s} ({len(taps)} taps from {hex_path.name})")
        sectors.append(taps_to_sector(taps))

    # Write image
    output_path = Path(args.output)
    with open(output_path, "wb") as f:
        for sector in sectors:
            f.write(sector)

    total_bytes = len(sectors) * SECTOR_SIZE
    print(f"\nWrote {output_path} ({total_bytes} bytes, {len(sectors)} sectors)")
    print(f"  Header: sector 0")
    print(f"  IRs:    sectors 1-{len(ir_entries)} ({len(ir_entries)} IRs)")
    print(f"  Default IR: {default_idx} ({names[default_idx]})")
    print(f"\nTo write to SD card:")
    print(f"  dd if={output_path} of=/dev/sdX bs=512 conv=notrunc")


if __name__ == "__main__":
    main()
