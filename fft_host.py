#!/usr/bin/env python3
"""Stream a 128-word FFT/echo program to the F18A SoC, push 128 input
words, read 128 output words back, validate.

Phase 1: identity (echo) — proves the I/O loop is bit-perfect.
Phase 2 (later): the actual FFT. Same host script, different .f18a
program loaded with `--prog`.

Each output word arrives as 3 bytes from UART_TX:
    byte 0 = low 8 bits, byte 1 = next 8 bits, byte 2 = top 2 bits.

Usage:
    make flash-sram
    python3 fft_host.py                     # echo with random inputs
    python3 fft_host.py --prog fft.f18a     # FFT (when written)
    python3 fft_host.py --pattern ramp      # 0..127 instead of random
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent

TTY = os.environ.get("TTY", "/dev/ttyUSB0")
BAUD = int(os.environ.get("BAUD", "115200"))
N = 128


def asm_to_hex(src: Path) -> list[int]:
    """Run asm.py -hex and parse out the 18-bit words."""
    hex_path = "/tmp/fft.hex"
    lst_path = "/tmp/fft.lst"
    subprocess.run(
        ["python3", str(ROOT / "asm.py"), "-hex", str(src), hex_path, lst_path],
        check=True,
    )
    raw: list[int] = []
    with open(hex_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("//"):
                continue
            raw.append(int(line, 16))
    last = len(raw)
    while last and raw[last - 1] == 0:
        last -= 1
    return raw[:last]


# Streaming-port loader (same words as ack_host.py / ga_aforth.py).
BOOT_LOAD_A    = 0x11FE0   # @p a! NOP RET
BOOT_STORE_INC = 0x10DE0   # @p !+ NOP RET
BOOT_JUMP_0    = 0x04000   # jump:0


def word_bytes(w: int) -> bytes:
    return bytes([w & 0xFF, (w >> 8) & 0xFF, (w >> 16) & 0x03])


def stream_program(s, words: list[int]) -> None:
    """Loader + program + jump:0."""
    payload = [BOOT_LOAD_A, 0]
    for w in words:
        payload += [BOOT_STORE_INC, w]
    payload.append(BOOT_JUMP_0)
    s.write(b"".join(word_bytes(w) for w in payload))
    s.flush()


def make_inputs(pattern: str) -> list[int]:
    if pattern == "ramp":
        return [i for i in range(N)]
    if pattern == "alternating":
        return [(0x2AAAA if i & 1 else 0x15555) for i in range(N)]
    if pattern == "negramp":
        return [((-i) & 0x3FFFF) for i in range(N)]
    if pattern == "random":
        import random
        random.seed(42)
        return [random.randrange(0x40000) for _ in range(N)]
    raise SystemExit(f"unknown pattern: {pattern}")


def expected_outputs(inputs: list[int], algo: str) -> list[int]:
    if algo == "echo":
        return list(inputs)
    raise SystemExit(f"no expected-output rule for algo {algo!r}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--prog", default="fft_echo.f18a",
                    help="F18A source to stream (default: %(default)s)")
    ap.add_argument("--algo", default="echo",
                    choices=["echo"],
                    help="algorithm — drives the host-side validation")
    ap.add_argument("--pattern", default="random",
                    choices=["ramp", "alternating", "negramp", "random"],
                    help="input value pattern")
    ap.add_argument("--timeout", type=float, default=5.0)
    ap.add_argument("--show", type=int, default=4,
                    help="print first N input/output pairs on success")
    args = ap.parse_args()

    import serial
    program = asm_to_hex(ROOT / args.prog)
    print(f"compiled {len(program)} words from {args.prog}")
    inputs = make_inputs(args.pattern)
    expected = expected_outputs(inputs, args.algo)

    s = serial.Serial(TTY, BAUD, timeout=0.2)
    try:
        time.sleep(0.05)
        s.reset_input_buffer()

        t0 = time.monotonic()
        stream_program(s, program)
        # Push the 128 input words.
        for w in inputs:
            s.write(word_bytes(w))
        s.flush()
        print(f"streamed program + {N} inputs ({3 * (len(program) + N) + 6} bytes)")

        # Read 128 × 3 bytes back.
        want = 3 * N
        got = bytearray()
        deadline = t0 + args.timeout
        while len(got) < want and time.monotonic() < deadline:
            chunk = s.read(want - len(got))
            if chunk:
                got += chunk

        elapsed = time.monotonic() - t0
        if len(got) < want:
            print(f"FAIL — got {len(got)}/{want} bytes after {elapsed:.2f}s")
            return 1
    finally:
        s.close()

    # Reassemble 18-bit words from the 3-byte stream.
    outputs = [
        got[3 * i] | (got[3 * i + 1] << 8) | ((got[3 * i + 2] & 0x03) << 16)
        for i in range(N)
    ]

    mismatches = [i for i in range(N) if outputs[i] != expected[i]]
    if mismatches:
        print(f"FAIL — {len(mismatches)} mismatch(es) of {N}")
        for i in mismatches[:8]:
            print(f"   [{i:3d}] got 0x{outputs[i]:05X} expected 0x{expected[i]:05X}")
        return 1

    rate = N / elapsed
    print(f"PASS — {N} words round-tripped bit-perfect "
          f"({elapsed*1000:.1f} ms, {rate:.0f} words/s)")
    for i in range(min(args.show, N)):
        print(f"   [{i:3d}] in=0x{inputs[i]:05X} out=0x{outputs[i]:05X}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
