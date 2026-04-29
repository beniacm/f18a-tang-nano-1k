#!/usr/bin/env python3
"""Smoke tests for the Forth-on-FPGA host compiler + sim back-end.

Each case runs a single REPL line through `forth_compile.compile_line`
and `forth_run.run_sim`, then checks the captured UART_TX bytes against
an expected string. Run from the f18a directory:

    python3 test_forth_compile.py
"""
from __future__ import annotations

import sys

import forth_compile
import forth_run


_ACK_DEF = (
    ": ack "
    "  over 0= if nip 1+ "
    "  else dup 0= if drop 1- 1 ack "
    "  else over 1- >R 1- ack R> swap ack "
    "  then then ; "
)


CASES = [
    ("emit literal",          "65 emit 10 emit",                   b"A\n"),
    ("arithmetic + emit",     "60 5 + emit 10 emit",               b"A\n"),
    ("subtract",              "10 7 - emit",                       bytes([3])),
    ("or",                    "0xA0 0x0F or emit",                 bytes([0xAF])),
    ("direct +*",             "5 a! 10 0 +* emit",                bytes([5])),
    ("if/else/then",          ": iszero 0= if 65 emit else 66 emit then ; "
                               "0 iszero 5 iszero",                 b"AB"),
    ("swap",                  "65 66 swap emit emit",              b"AB"),
    ("colon + call",          ": hi 72 emit 105 emit 10 emit ; hi", b"Hi\n"),
    ("nested colons",         ": shout 33 emit ; "
                              ": greet 72 emit 105 emit shout 10 emit ; "
                              "greet greet",                       b"Hi!\nHi!\n"),
    ("dup / drop",            "65 dup emit drop 66 emit 10 emit",  b"AB\n"),
    ("ackermann 0,0",         _ACK_DEF + " 0 0 ack emit",          bytes([1])),
    ("ackermann 1,1",         _ACK_DEF + " 1 1 ack emit",          bytes([3])),
    ("ackermann 2,2",         _ACK_DEF + " 2 2 ack emit",          bytes([7])),
    ("ackermann 3,1",         _ACK_DEF + " 3 1 ack emit",          bytes([13])),
    # NB: the 16/32 FULLISA build reaches A(3,1), but A(3,2) still
    # overruns the practical recursive depth on hardware.
]


def main() -> int:
    fails = 0
    for name, src, expected in CASES:
        state = forth_compile.CompilerState()
        program = forth_compile.compile_line(state, src)
        if program is None:
            print(f"FAIL {name}: compile returned None")
            fails += 1
            continue
        try:
            out = forth_run.run_sim(program, cycle_budget=20_000_000)
        except Exception as e:
            print(f"FAIL {name}: run_sim raised: {e}")
            fails += 1
            continue
        if out != expected:
            print(f"FAIL {name}: expected {expected!r}, got {out!r}")
            fails += 1
            continue
        print(f"PASS {name}: {out!r}")
    if fails:
        print(f"{fails} failure(s)")
        return 1
    print("ALL PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
