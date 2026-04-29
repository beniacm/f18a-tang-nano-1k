#!/usr/bin/env python3
"""Bridge ga-tools' aforth front-end to this repo's F18A SoC.

ga-tools (https://github.com/mschuldt/ga-tools) compiles GA144-flavoured
aforth source — `: name … ;` definitions, structured `if/then`,
`for/next`, `begin/until/while`, label-driven calls, the lot. It targets
the *real* F18A's instruction encoding, which XORs the slot bits to
make 3-bit slot-3 packing work in silicon. Our `asm.py` (and the
softcore in `f18a_core.v`) use a simpler plain-bits encoding, so the
two outputs aren't binary-compatible.

The bridge: let ga-tools parse and lay out the program (slot packing,
label resolution, branch placement), walk the resulting `Word` list,
emit each word as one line of `.f18a` source mnemonics, and re-feed
that through our `asm.py`. The mnemonic level is layout-stable across
the two encoders, so the only special case is slot-3 NOP — ga-tools
allows it (XOR encoding makes room), our asm.py rejects it. We coerce
NOP → RET there; semantically identical at top level, where rsp=0
turns RET into "fetch next word".

The CLI streams the compiled program straight at INST_PORT over UART
using the same 3-byte-per-instruction loader as `ack_host.py`.

Caveats:
  * ga-tools assumes the GA144 grid: `node N` plus 64-cell-per-node
    addresses. We just take the first node and run with it; addresses
    are fine for our 1024-cell BSRAM as long as you stay below 0x400.
  * Branches in slot 1 / slot 2 only reach 8 / 256 cells respectively.
    Use `..` to force-align onto slot 0 if your jump target needs the
    full 13-bit reach.
"""
from __future__ import annotations

import argparse
import os
import sys
import tempfile
import time

# ga-tools is installed via `pip install --user ga-tools`; pick up the
# user site-packages directory if Python isn't already importing from it.
USER_SITE = os.path.expanduser("~/.local/lib/python3.13/site-packages")
if os.path.isdir(USER_SITE) and USER_SITE not in sys.path:
    sys.path.insert(0, USER_SITE)

import ga_tools                                          # noqa: E402
from ga_tools.word import Word, INST, CONST, ADDR        # noqa: E402
from ga_tools.defs import ops                            # noqa: E402

# Our local assembler.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import asm                                               # noqa: E402


# Real F18A allows 8 ops in slot 3 (`last_slot_ops` in ga-tools' defs.py:
# `; unext @p !p +* + dup .`). Our softcore decodes slot 3 as plain
# bits 2:0 with top 2 bits zeroed, so only `; ex unext` (op indices
# 0, 1, 4) fit. We can't auto-spill the rest — a `@p` in slot 3 reads
# the next memory cell as its literal, so moving the @p to a fresh
# word would also need to relocate the corresponding literal cell.
# Likewise `.` in slot 3 isn't a free remap to `;`: at rsp>0 (e.g.,
# inside a `for/next` loop after `push`) `;` returns to R, while `.`
# is a no-op. The right fix is to refactor the aforth source: insert
# `..` (force-align) so the offending op lands in slot 0..2 of a
# fresh word.
OUR_SLOT3_OK = {";", "unext", "@p", "!p", "+*", "+", "dup", "."}


class Slot3UnsupportedError(Exception):
    pass


def _word_to_line(w: Word) -> str:
    """Render one ga-tools `Word` as one line of `.f18a` source. Both
    encoders now agree on the 8 valid slot-3 ops (`; unext @p !p +*
    + dup .`), so each ga-tools word maps 1:1 to one of our words."""
    if w.empty():
        return "    .  .  .  ;"
    if w.type == CONST:
        return f"    [0x{w.get_const(True) & 0x3FFFF:05X}]"
    if w.type == INST:
        slots = [ops[i] if i is not None else "." for i in w._slots]
        if slots[3] not in OUR_SLOT3_OK:
            raise Slot3UnsupportedError(
                f"slot 3 op {slots[3]!r} (in word {slots}) is allowed by "
                "real F18A but not by this softcore. Refactor the aforth "
                "source: insert `..` before the offending op so it lands "
                "in slot 0..2 of a fresh word."
            )
        return "    " + " ".join(slots)
    if w.type == ADDR:
        head = [ops[w._slots[i]] for i in range(w.op_index)]
        br   = ops[w._slots[w.op_index]]
        return f"    {' '.join(head + [br])}:0x{w._addr:X}".rstrip()
    raise AssertionError(f"unknown word type {w.type}")


def aforth_to_f18a_source(aforth_src: str) -> str:
    """Compile aforth via ga-tools, return the equivalent `.f18a`
    source for our asm.py."""
    ga_tools.clear_chips()
    ga_tools.include_string(aforth_src)
    ga_tools.do_compile()
    chip = list(ga_tools.get_chips().values())[0]
    node = next(iter(chip.nodes.values()))
    lines = ["#origin 0x000"]
    w = node.ram
    while w is not None:
        lines.append(_word_to_line(w))
        w = w.next
    # Append a tail jump to INST_PORT so the FPGA hands control back to
    # the host streamer when the program runs out — same idiom as
    # ackermann.f18a / forth_compile. Without this, ga-tools' aforth
    # programs walk off the end into zero-filled BSRAM and the FPGA
    # never re-enters port-execution, blocking the next streamed run.
    lines.append("    jump:0x7F9")
    return "\n".join(lines) + "\n"


def compile_aforth_to_words(aforth_src: str) -> tuple[int, list[int]]:
    """Compile aforth and return (origin, [18-bit words ready to stream])."""
    f18a_src = aforth_to_f18a_source(aforth_src)
    with tempfile.NamedTemporaryFile("w", suffix=".f18a", delete=False) as f:
        f.write(f18a_src)
        path = f.name
    try:
        origin, words, _listing, _symbols, _vars = asm.assemble_to_words(
            path, size=1024
        )
    finally:
        os.unlink(path)
    return origin, words


# ── Streaming-port loader (same as ack_host.py / forth_run.run_hw) ──
BOOT_LOAD_A    = 0x11FE0   # @p a! NOP RET   — A ← next streamed word
BOOT_STORE_INC = 0x10DE0   # @p !+ NOP RET   — mem[A] ← next, A++
BOOT_JUMP_0    = 0x04000   # jump:0          — leave port-exec, run from RAM


def _word_bytes(w: int) -> bytes:
    return bytes([w & 0xFF, (w >> 8) & 0xFF, (w >> 16) & 0x03])


def stream_to_hw(words: list[int], *, tty: str = "/dev/ttyUSB0",
                 baud: int = 115200, idle_timeout: float = 1.0,
                 expected_bytes: int | None = None,
                 max_bytes: int = 4096) -> bytes:
    """Stream loader + program + jump:0 to the FPGA over UART, then
    collect UART_TX bytes until idle / max_bytes / expected_bytes."""
    import serial  # type: ignore
    if not words:
        return b""

    payload = [BOOT_LOAD_A, 0]
    for w in words:
        payload += [BOOT_STORE_INC, w]
    payload.append(BOOT_JUMP_0)
    blob = b"".join(_word_bytes(w) for w in payload)

    s = serial.Serial(tty, baud, timeout=0.2)
    try:
        s.reset_input_buffer()
        s.write(blob)
        s.flush()
        out = bytearray()
        deadline = time.monotonic() + idle_timeout
        while time.monotonic() < deadline and len(out) < max_bytes:
            chunk = s.read(64)
            if chunk:
                deadline = time.monotonic() + idle_timeout
                for b in chunk:
                    if b == 0xFF:        # FTDI idle byte
                        continue
                    out.append(b)
                    if expected_bytes is not None and len(out) >= expected_bytes:
                        break
                if expected_bytes is not None and len(out) >= expected_bytes:
                    break
    finally:
        s.close()
    return bytes(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("source", nargs="?",
                    help="aforth source file (default: tiny `emit A` demo)")
    ap.add_argument("--hw", action="store_true",
                    help="stream to /dev/ttyUSB0 instead of just printing")
    ap.add_argument("--print-asm", action="store_true",
                    help="print the .f18a translation and exit")
    ap.add_argument("--idle", type=float, default=1.0,
                    help="seconds to wait for UART_TX bytes after streaming")
    ap.add_argument("--expect", type=int, default=None,
                    help="stop reading after this many bytes")
    ap.add_argument("--fire-and-forget", action="store_true",
                    help="just stream the program and exit (no UART read)")
    args = ap.parse_args()

    if args.source:
        with open(args.source) as f:
            aforth_src = f.read()
    else:
        # Default demo: emit "Hi\n" (3 bytes) via UART_TX.
        # Each byte is a separate `<addr> b! <val> !b ..` block. The
        # trailing `..` after each store forces ga-tools to start a
        # fresh word, so the next `b!`/`!b` lands in slot 0..2 and
        # never spills slot-3-only ops into our slot-3-restricted core.
        aforth_src = (
            "node 0\n"
            ": main\n"
            "    0x7F1 b!  72 !b ..\n"   # 'H'
            "    0x7F1 b!  105 !b ..\n"  # 'i'
            "    0x7F1 b!  10 !b\n"      # '\\n'
        )

    f18a_src = aforth_to_f18a_source(aforth_src)
    if args.print_asm:
        print(f18a_src)
        return 0

    _origin, words = compile_aforth_to_words(aforth_src)
    print(f"compiled {len(words)} words from aforth")

    if not args.hw:
        for i, w in enumerate(words):
            print(f"  {i:03X}: 0x{w:05X}")
        return 0

    if args.fire_and_forget:
        stream_to_hw(words, idle_timeout=0.0, expected_bytes=0)
        print(f"streamed {len(words)} words (fire-and-forget)")
        return 0

    out = stream_to_hw(words, idle_timeout=args.idle, expected_bytes=args.expect)
    if not out:
        print("FAIL — no bytes received")
        return 1
    print(f"received {len(out)} bytes: {out!r}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
