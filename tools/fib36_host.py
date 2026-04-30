#!/usr/bin/env python3
"""Stream programs/fib36.f18a to the F18A SoC and read back the
36-bit Fibonacci sequence it computes via P9 add-with-carry.

The on-chip program lives at 0x200 (P9 page), runs 30 iterations of
   next = prev + curr
with prev/curr each split across a low + high 18-bit cell, and emits
2 UART bytes per step:
   byte 0 = curr & 0xFF              (low byte of bits  0..17)
   byte 1 = (curr >> 18) & 0xFF      (low byte of bits 18..35)

Two bytes is enough to verify the multi-precision add: once the
Fibonacci sequence passes 2^18 = 262 144 (Fib(28) = 317 811), the
high cell becomes non-zero, and the second byte starts diverging
from zero. The host-side reference computes the same 36-bit
sequence in Python and checks both bytes match at every step.

Usage:
    make flash-sram
    python3 tools/fib36_host.py
"""
from __future__ import annotations

import os
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TTY  = os.environ.get("TTY",  "/dev/ttyUSB0")
BAUD = int(os.environ.get("BAUD", "115200"))
N    = 30

BOOT_LOAD_A    = 0x11FE0
BOOT_STORE_INC = 0x10DE0
BOOT_JUMP      = 0x04000


def asm_to_hex(src: Path) -> tuple[int, list[int]]:
    """Run asm.py and return (origin, words[])."""
    hex_path = "/tmp/fib36.hex"
    lst_path = "/tmp/fib36.lst"
    subprocess.run(
        ["python3", str(ROOT / "tools" / "asm.py"), "-hex",
         str(src), hex_path, lst_path],
        check=True,
    )
    raw: list[int] = []
    origin = 0
    with open(lst_path) as f:
        for line in f:
            m = re.match(r";\s*origin=(0x[0-9A-Fa-f]+)", line)
            if m:
                origin = int(m.group(1), 0)
                break
    used = None
    with open(hex_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("//"):
                m = re.search(r"(\d+)\s+words used of\s+(\d+)", line)
                if m:
                    used = int(m.group(1))
                continue
            raw.append(int(line, 16))
    if used is None:
        used = len(raw)
        while used and raw[used - 1] == 0:
            used -= 1
    # asm.py emits a flat 1024-cell memory image; for non-zero origin
    # the meaningful cells live at raw[origin:origin+used].
    return origin, raw[origin:origin + used]


def word_bytes(w: int) -> bytes:
    return bytes([w & 0xFF, (w >> 8) & 0xFF, (w >> 16) & 0x03])


def stream(s, origin: int, prog: list[int]) -> int:
    """Stream loader + program (at origin) + jump:origin. Returns
    bytes sent."""
    payload = [BOOT_LOAD_A, origin]
    for w in prog:
        payload += [BOOT_STORE_INC, w]
    payload.append(BOOT_JUMP | (origin & 0x1FFF))
    blob = b"".join(word_bytes(w) for w in payload)
    s.write(blob)
    s.flush()
    return len(blob)


def main() -> int:
    import serial
    origin, prog = asm_to_hex(ROOT / "programs" / "fib36.f18a")
    print(f"compiled {len(prog)} words from programs/fib36.f18a "
          f"(origin = 0x{origin:03X})")

    # 36-bit Fibonacci reference (host-side).
    expected: list[int] = []
    a, b = 0, 1
    for _ in range(N):
        a, b = b, (a + b) & ((1 << 36) - 1)
        # the program emits curr AFTER computing next, so the i-th
        # emitted value is Fib(i+1), starting from Fib(1) = 1.
        expected.append(a)

    s = serial.Serial(TTY, BAUD, timeout=0)
    try:
        time.sleep(0.05)
        s.reset_input_buffer()

        t0 = time.monotonic()
        nbytes = stream(s, origin, prog)
        print(f"streamed {nbytes} bytes (loader + {len(prog)} prog cells)")

        want = 2 * N
        got = bytearray()
        deadline = t0 + 10.0
        while len(got) < want and time.monotonic() < deadline:
            n = s.in_waiting
            if n:
                chunk = bytes(c for c in s.read(n) if c != 0xFF)
                got += chunk
        elapsed_ms = 1000 * (time.monotonic() - t0)
    finally:
        s.close()

    if len(got) < want:
        print(f"FAIL — got {len(got)}/{want} bytes after {elapsed_ms:.1f} ms")
        return 1

    print(f"received {N} × 2 bytes in {elapsed_ms:.1f} ms")
    print()
    print(f"   {'i':>3}  {'expected':>14}  {'lo':>4}  {'hi':>4}  status")
    print(   "   " + "-" * 44)
    failures = 0
    for i in range(N):
        got_lo, got_hi = got[2*i], got[2*i+1]
        want = expected[i]
        want_lo = want & 0xFF
        want_hi = (want >> 18) & 0xFF
        ok = got_lo == want_lo and got_hi == want_hi
        mark = "OK" if ok else f"FAIL (want {want_lo:02X} {want_hi:02X})"
        if not ok:
            failures += 1
        print(f"   {i+1:>3}  {want:>14}  {got_lo:>4}  {got_hi:>4}  {mark}")

    print()
    if failures == 0:
        last = expected[-1]
        print(f"PASS — 36-bit Fib({N}) computed via P9 add-with-carry "
              f"on hardware: {last} = 0x{last:X}")
        return 0
    else:
        print(f"FAIL — {failures}/{N} mismatches")
        return 1


if __name__ == "__main__":
    sys.exit(main())
