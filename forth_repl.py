#!/usr/bin/env python3
"""Interactive Forth-on-FPGA REPL.

Each non-definition input line is compiled into a fresh F18A program
(definitions accumulated across lines + the line as `main` body), run
on the chosen back-end (sim by default), and the captured UART_TX
output is printed.

Usage:
    ./forth_repl.py                # sim mode
    ./forth_repl.py --hw           # hardware (NotImplementedError until
                                   # the UART back-end is wired in)
    ./forth_repl.py --once "5 emit"      # run a single line and exit

`:` definitions are stored host-side and don't trigger a download —
they're prepended to the program on every subsequent evaluatable line.
"""
from __future__ import annotations

import argparse
import sys
from typing import Callable

import forth_compile
import forth_run


def _print_dictionary(state: forth_compile.CompilerState) -> None:
    if not state.dictionary:
        print("(no user definitions)")
        return
    for name, body in state.dictionary.items():
        print(f": {name} {' '.join(body)} ;")


def _print_output(buf: bytes) -> None:
    if not buf:
        return
    try:
        sys.stdout.write(buf.decode("ascii"))
    except UnicodeDecodeError:
        sys.stdout.write(repr(buf))
        sys.stdout.write("\n")
    sys.stdout.flush()


def evaluate(state: forth_compile.CompilerState, line: str,
             runner: Callable[[str], bytes]) -> None:
    try:
        program = forth_compile.compile_line(state, line)
    except ValueError as e:
        print(f"compile error: {e}", file=sys.stderr)
        return
    if program is None:
        # Pure definition line — nothing to run.
        print("ok.")
        return
    try:
        out = runner(program)
    except Exception as e:
        print(f"run error: {e}", file=sys.stderr)
        return
    _print_output(out)
    print("ok.")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--hw", action="store_true", help="use hardware back-end")
    ap.add_argument("--once", help="evaluate one line and exit")
    args = ap.parse_args()

    runner = forth_run.run_hw if args.hw else forth_run.run_sim
    state = forth_compile.CompilerState()

    if args.once is not None:
        evaluate(state, args.once, runner)
        return 0

    print("forth-on-fpga (Ctrl-D to exit; `.words` lists definitions)")
    while True:
        try:
            line = input("> ")
        except (EOFError, KeyboardInterrupt):
            print()
            return 0
        line = line.strip()
        if not line:
            continue
        if line == ".words":
            _print_dictionary(state)
            continue
        evaluate(state, line, runner)
    return 0


if __name__ == "__main__":
    sys.exit(main())
