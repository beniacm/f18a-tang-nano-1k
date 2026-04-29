#!/usr/bin/env python3
"""Build and run the native 16x16 `+*` multiply benchmark and report the
cycle count.

Usage:
    python3 run_muls_bench.py
"""
from __future__ import annotations

import contextlib
import io
import re
import subprocess
import sys
from pathlib import Path

import asm

ROOT = Path(__file__).resolve().parent
HEX = Path("/tmp/muls_bench.hex")
LST = Path("/tmp/muls_bench.lst")


def build_image() -> None:
    """Assemble muls_bench.f18a into a flat 1K hex image."""
    flat = [0] * 1024
    for src in ("muls_bench.f18a",):
        origin, words, *_ = asm.assemble_to_words(str(ROOT / src), size=1024)
        for i, w in enumerate(words):
            flat[origin + i] = w
    with HEX.open("w") as f:
        f.write("// muls_bench image\n")
        for w in flat:
            f.write(f"{w:05x}\n")


def run_variant(label: str) -> tuple[int, list[int]]:
    bin_path = Path(f"/tmp/tb_muls_bench_{label}")
    cmd = ["iverilog", "-g2012", "-o", str(bin_path),
           str(ROOT / "tb_muls_bench.v"), str(ROOT / "f18a_core.v")]
    subprocess.run(cmd, cwd=ROOT, check=True)
    proc = subprocess.run([str(bin_path)], capture_output=True, text=True, check=True)
    cycles = None
    out_bytes: list[int] = []
    for line in proc.stdout.splitlines():
        m = re.match(r"^UART_TX 0x([0-9a-fA-F]{2})\s*$", line)
        if m:
            out_bytes.append(int(m.group(1), 16))
            continue
        m = re.match(r"^CYCLES (\d+)\s*$", line)
        if m:
            cycles = int(m.group(1))
        if "TIMEOUT" in line:
            raise RuntimeError(f"{label}: timeout — {line}")
    if cycles is None:
        raise RuntimeError(f"{label}: no CYCLES line in output:\n{proc.stdout}")
    return cycles, out_bytes


def main() -> int:
    with contextlib.redirect_stdout(io.StringIO()):
        build_image()

    nat_cycles, nat_out = run_variant("native")

    print(f"  16-bit operands: 0xABCD * 0x1234 = 0x{0xABCD*0x1234:08x}")
    print(f"  emitted byte: {nat_out!r}")
    print(f"  native MULS:  {nat_cycles:6d} cycles")
    return 0


if __name__ == "__main__":
    sys.exit(main())
