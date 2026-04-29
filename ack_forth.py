#!/usr/bin/env python3
"""Ackermann benchmark, compiled from high-level Forth and executed on
the F18A FPGA (or sim).

Drop-in spiritual successor to `ack_host.py`: same `-m` / `-n` arguments
and result format, but instead of W-loading a hand-written assembly
image (`ackermann.f18a`), every run compiles a fresh Forth source via
`forth_compile.py` and runs it through one of the back-ends in
`forth_run.py` (sim by default, hardware via `--hw`).

Forth source used (one `:` definition that encodes the three Ackermann
cases plus the immediate `M N ack emit` to push the result byte to
UART_TX):

    : ack
        over 0= if nip 1+
        else dup 0= if drop 1- 1 ack
        else over 1- >R 1- ack R> swap ack
        then then
    ;
    M N ack emit

The emitted byte is A(M, N). Inputs that overflow 8 bits are still
computed correctly — they just wrap when squeezed through `emit`. Use
A(3,3)=61 or below to stay inside one byte.
"""
from __future__ import annotations

import argparse
import sys
import time

import forth_compile
import forth_run


ACK_DEFINITION = (
    ": ack "
    "  over 0= if nip 1+ "
    "  else dup 0= if drop 1- 1 ack "
    "  else over 1- >R 1- ack R> swap ack "
    "  then then "
    "; "
)

EXPECTED = {
    (0, 0): 1, (0, 1): 2, (0, 5): 6,
    (1, 0): 2, (1, 1): 3, (1, 2): 4, (1, 3): 5,
    (2, 0): 3, (2, 1): 5, (2, 2): 7, (2, 3): 9,
    (3, 0): 5, (3, 1): 13, (3, 2): 29, (3, 3): 61,
}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("-m", type=int, default=3, help="m_init  (default 3)")
    ap.add_argument("-n", type=int, default=2, help="n_init  (default 2)")
    ap.add_argument("--hw", action="store_true",
                    help="run on Tang Nano 1K via UART instead of sim")
    ap.add_argument("--cycles", type=int, default=20_000_000,
                    help="sim cycle budget (sim only)")
    ap.add_argument("--idle", type=float, default=2.0,
                    help="max seconds to wait for the first result byte on hardware")
    args = ap.parse_args()

    src = ACK_DEFINITION + f" {args.m} {args.n} ack emit"
    state = forth_compile.CompilerState()
    program = forth_compile.compile_line(state, src)
    if program is None:
        print("internal error: compiler returned no program", file=sys.stderr)
        return 2

    t0 = time.monotonic()
    if args.hw:
        out = forth_run.run_hw(program, idle_timeout=args.idle, expected_bytes=1)
    else:
        out = forth_run.run_sim(program, cycle_budget=args.cycles)
    elapsed_ms = 1000.0 * (time.monotonic() - t0)

    if not out:
        print(f"FAIL — no result byte after {elapsed_ms:.0f} ms")
        return 1

    result = out[0] & 0xFF
    backend = "hw" if args.hw else "sim"
    print(f"A({args.m}, {args.n}) = {result}   ({elapsed_ms:.1f} ms, {backend})")

    expected = EXPECTED.get((args.m, args.n))
    if expected is not None and (result != (expected & 0xFF)):
        print(f"FAIL — expected {expected & 0xFF}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
