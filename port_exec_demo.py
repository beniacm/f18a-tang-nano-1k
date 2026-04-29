#!/usr/bin/env python3
"""Streamed boot demo for the F18A SoC.

The default Tang Nano 1K bitstream comes up with P=0x7F9 (the GA144-
style INST_PORT). The core stalls on its first fetch until 3 bytes
have been streamed in over UART; the SoC assembles them into one
18-bit instruction word and lets the fetch advance. P stays at the
port (DB001 §3.3.2) until something explicitly transfers control.

Demo: emit the byte 'X' (0x58) to UART_TX. Streamed instruction
sequence (12 bytes total):

    @p b! NOP RET    — pop the next streamed word into B
    [0x7F1]          — UART_TX address
    @p !b NOP RET    — pop the next streamed word, store at mem[B]
    [0x58]           — ASCII 'X'

After this, we keep streaming NOP-padding so the fetch pump stays
fed; the host stops as soon as it receives the 'X'.

Note: the FPGA's B register comes up at UART_TX (RESET_B in the soc),
which means the very first @p !b would already work without setting
B explicitly. We set it anyway here so the demo is self-contained.

Re-flash (`make flash-fullisa-sram`) between runs to get a fresh
power-on reset; without that the core may already be past the port
from the previous demo.
"""
import os, sys, time, serial

TTY  = os.environ.get("TTY", "/dev/ttyUSB0")
BAUD = int(os.environ.get("BAUD", "115200"))


def word_bytes(w):
    """Split an 18-bit instruction word into 3 UART bytes (lo, mid, hi[1:0])."""
    return bytes([w & 0xFF, (w >> 8) & 0xFF, (w >> 16) & 0x03])


def main():
    s = serial.Serial(TTY, BAUD, timeout=0.5)
    time.sleep(0.2)
    s.reset_input_buffer()

    program = [
        0x11EE0,   # @p b! NOP RET
        0x007F1,   #   [0x7F1]   — UART_TX
        0x10EE0,   # @p !b NOP RET
        0x00058,   #   [0x58]    — 'X'
    ]
    pad = 0x39CE0   # NOP NOP NOP RET — keeps fetch pumping while we wait

    print(f"streaming {len(program)} instruction words ({3 * len(program)} bytes)")
    s.write(b"".join(word_bytes(w) for w in program)); s.flush()

    # Pump pad NOPs until we see 'X' or time out.
    deadline = time.monotonic() + 1.0
    got = bytearray()
    while time.monotonic() < deadline:
        if s.in_waiting:
            r = s.read(s.in_waiting)
            got += bytes(c for c in r if c != 0xFF)
            if b"X" in got:
                break
        else:
            s.write(word_bytes(pad)); s.flush()
            time.sleep(0.005)

    if b"X" in got:
        print(f"got 'X' (0x58) — port-execution working")
        print(f"  full received: {bytes(got)!r}")
        sys.exit(0)
    else:
        print(f"FAIL — no 'X' received. got: {bytes(got)!r}")
        sys.exit(1)


if __name__ == "__main__":
    main()
