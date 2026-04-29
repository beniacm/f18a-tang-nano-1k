"""Run a compiled Forth program through one of two back-ends.

Sim back-end (default): assembles the program to /tmp/forth_run.hex,
invokes the cached `tb_forth_runtime` iverilog binary, parses lines like
"OUT 0xNN" out of stdout, and returns the captured bytes (with the EOT
sentinel stripped).

Hardware back-end: streams a 3-byte-per-instruction bootstrap loader,
the program payload, and a `jump:0` straight into the FPGA's INST_PORT
(DB001 §3.3.2 port-execution); then collects UART_TX bytes until the
line falls idle for `idle_timeout` seconds.
"""
from __future__ import annotations

import contextlib
import io
import os
import re
import subprocess
import tempfile
from pathlib import Path
from typing import List

import asm

ROOT = Path(__file__).resolve().parent
TB_BIN = Path(os.environ.get("TB_FORTH_RUNTIME_BIN", "/tmp/tb_forth_runtime"))
HEX_PATH = Path("/tmp/forth_run.hex")
LST_PATH = Path("/tmp/forth_run.lst")


class CompileError(RuntimeError):
    pass


def ensure_tb_built() -> None:
    """Build the iverilog testbench once and reuse the binary."""
    if TB_BIN.exists():
        # Rebuild if any source it depends on is newer than the binary.
        deps = [ROOT / "tb_forth_runtime.v", ROOT / "f18a_core.v"]
        if all(TB_BIN.stat().st_mtime >= dep.stat().st_mtime for dep in deps):
            return
    subprocess.run(
        ["iverilog", "-g2012", "-o", str(TB_BIN),
         str(ROOT / "tb_forth_runtime.v"),
         str(ROOT / "f18a_core.v")],
        cwd=ROOT, check=True,
    )


def assemble_to_hex(asm_source: str) -> None:
    """Write `asm_source` to a tempfile and run asm.py's hex emitter."""
    with tempfile.NamedTemporaryFile("w", suffix=".f18a", delete=False) as f:
        f.write(asm_source)
        src_path = f.name
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            asm.assemble_to_hex(src_path, str(HEX_PATH), str(LST_PATH), mem_size=2048)
    except SystemExit as e:
        raise CompileError(f"asm.py rejected the program: exit {e.code}")
    except Exception as e:
        raise CompileError(str(e))
    finally:
        try:
            os.unlink(src_path)
        except FileNotFoundError:
            pass


_OUT_RE = re.compile(r"^OUT 0x([0-9a-fA-F]{2})\s*$")


def run_sim(asm_source: str, cycle_budget: int = 200_000) -> bytes:
    """Build (if needed), assemble `asm_source`, run the sim, return the
    bytes the program emitted to UART_TX (EOT sentinel stripped)."""
    ensure_tb_built()
    assemble_to_hex(asm_source)
    proc = subprocess.run(
        [str(TB_BIN), f"+cycles={cycle_budget}"],
        capture_output=True, text=True, check=False,
    )
    out = bytearray()
    for line in proc.stdout.splitlines():
        m = _OUT_RE.match(line)
        if m:
            out.append(int(m.group(1), 16))
    if "TIMEOUT" in proc.stdout:
        raise RuntimeError(f"sim timed out after {cycle_budget} cycles")
    return bytes(out)


def _read_words_from_hex() -> list[tuple[int, int]]:
    """Parse /tmp/forth_run.hex into (addr, word) pairs for every
    non-zero cell. asm.assemble_to_hex emits one 5-hex-digit word per
    line, padded to mem_size=2048 — we drop trailing zeros after the
    last meaningful word to keep the W-stream short."""
    raw: list[int] = []
    with open(HEX_PATH) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("//"):
                continue
            raw.append(int(line, 16))
    last = len(raw)
    while last > 0 and raw[last - 1] == 0:
        last -= 1
    return [(addr, raw[addr]) for addr in range(last)]


# F18A instruction words used by the streamed loader. Each is 3 UART bytes
# (lo, mid, hi[1:0]) — a single instruction word the core executes off the
# port before P advances back to PORT_ADDR for the next streamed word.
BOOT_LOAD_A    = 0x11FE0   # @p a! NOP RET   — A ← next streamed word
BOOT_STORE_INC = 0x10DE0   # @p !+ NOP RET   — mem[A] ← next, A++
BOOT_JUMP_0    = 0x04000   # jump:0          — leave port-exec, run from RAM


def _word_bytes(w: int) -> bytes:
    return bytes([w & 0xFF, (w >> 8) & 0xFF, (w >> 16) & 0x03])


def run_hw(asm_source: str, *, tty: str | None = None, baud: int | None = None,
           idle_timeout: float = 0.4, max_bytes: int = 4096,
           expected_bytes: int | None = None) -> bytes:
    """Compile, stream the loader + program payload + `jump:0` into the
    FPGA's INST_PORT, then collect UART_TX bytes until the line falls
    idle for `idle_timeout` seconds (or `max_bytes` arrive, or
    `expected_bytes` worth of payload have been seen).

    The bitstream must already be flashed; re-flash (`make flash-sram`)
    between runs to start each program from a clean POR — without that
    the core is already past the loader from the previous run."""
    import serial  # type: ignore
    tty  = tty  or os.environ.get("TTY",  "/dev/ttyUSB0")
    baud = baud or int(os.environ.get("BAUD", "115200"))

    assemble_to_hex(asm_source)
    words = [w for _addr, w in _read_words_from_hex()]
    if not words:
        return b""

    target = 0
    payload = [BOOT_LOAD_A, target]
    for w in words:
        payload += [BOOT_STORE_INC, w]
    payload.append(BOOT_JUMP_0 | (target & 0x1FFF))
    blob = b"".join(_word_bytes(w) for w in payload)

    s = serial.Serial(tty, baud, timeout=0.2)
    try:
        s.reset_input_buffer()
        s.write(blob)
        s.flush()

        out = bytearray()
        deadline = _monotonic() + idle_timeout
        while _monotonic() < deadline and len(out) < max_bytes:
            chunk = s.read(64)
            if chunk:
                deadline = _monotonic() + idle_timeout
                for b in chunk:
                    # FTDI idles 0xFF on dropped bytes — filter them.
                    if b == 0xFF:
                        continue
                    out.append(b)
                    if expected_bytes is not None and len(out) >= expected_bytes:
                        break
                if expected_bytes is not None and len(out) >= expected_bytes:
                    break
    finally:
        s.close()

    return bytes(out)


def _monotonic() -> float:
    import time
    return time.monotonic()
