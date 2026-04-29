#!/usr/bin/env python3
"""Benchmark aforth_ack.ga (recursive Forth via ga-tools) vs
ackermann.f18a (iterative hand-asm).

Default (sim): measures cycles to first UART_TX write in iverilog —
eliminates UART/streaming overhead, gives a clean compute-only
comparison.

  --hw: also run the iterative ackermann on the connected board via
        the streaming-port loader (TTY=/dev/ttyUSB0, BAUD=115200) and
        report per-iter wall-time + derived hardware cycles. Uses a
        large -r to amortize the one-shot ~33 ms program upload.

Run: PATH=/opt/oss-cad-suite/bin:$PATH python3 bench_ack.py
     PATH=...                            python3 bench_ack.py --hw
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
TOOLS_DIR = REPO_ROOT / "tools"
RTL_DIR   = REPO_ROOT / "rtl"
PROGRAMS_DIR = REPO_ROOT / "programs"
sys.path.insert(0, str(TOOLS_DIR))
sys.path.insert(0, "/home/me/.local/lib/python3.13/site-packages")
import asm  # noqa: E402
import ga_aforth  # noqa: E402

ENV = {**os.environ, "PATH": "/opt/oss-cad-suite/bin:" + os.environ.get("PATH", "")}

PERF_TB_SRC = """
// Performance harness for ack benchmarks.
//   * Loads /tmp/aforth.hex into a 1K async-read RAM (with UART_TX/RX
//     stubs at 0x7F0..0x7F2).
//   * Runs the F18A core with dstack=64 / rstack=256 (deeper than the
//     hardware ships) so recursive aforth doesn't overflow at A(3,3).
//   * Stops on the first UART_TX write and prints cycles.
`timescale 1ns/1ps
module tb_perf;
    reg clk = 0;
    always #1 clk = ~clk;
    reg resetn = 0;
    wire [10:0] mem_addr;
    wire        mem_we;
    wire [17:0] mem_wdata;
    reg  [17:0] mem_rdata;
    reg  [17:0] mem [0:1023];
    integer i;
    initial begin
        for (i = 0; i < 1024; i = i + 1) mem[i] = 18'd0;
        $readmemh("/tmp/aforth.hex", mem);
        #20 resetn = 1;
    end
    wire is_tx = (mem_addr == 11'h7F1);
    wire is_io = mem_addr[10];
    always @(posedge clk) if (resetn) begin
        if (mem_we && !is_io) mem[mem_addr[9:0]] <= mem_wdata;
        if (is_io) mem_rdata <= 18'h0;
        else       mem_rdata <= mem[mem_addr[9:0]];
    end
    f18a_core #(.ADDR_BITS(11), .DSTK_DEPTH(64), .DSP_BITS(6),
                .RSTK_DEPTH(256), .RSP_BITS(8), .RESET_PC(11'h000)) cpu (
        .clk(clk), .resetn(resetn),
        .mem_addr(mem_addr), .mem_we(mem_we),
        .mem_wdata(mem_wdata), .mem_rdata(mem_rdata), .mem_ready(1'b1),
        .dbg_T(), .dbg_I(), .dbg_P(), .dbg_slot()
    );
    integer cycles = 0;
    integer peak_rsp = 0, peak_dsp = 0;
    always @(posedge clk) if (resetn) begin
        cycles <= cycles + 1;
        if (cpu.rsp > peak_rsp) peak_rsp <= cpu.rsp;
        if (cpu.dsp > peak_dsp) peak_dsp <= cpu.dsp;
        if (mem_we && is_tx) begin
            $display("RESULT 0x%02h CYCLES %0d PEAK_RSP %0d PEAK_DSP %0d",
                     mem_wdata[7:0], cycles, peak_rsp, peak_dsp);
            #4 $finish;
        end
        if (cycles > 1000000000) begin
            $display("TIMEOUT cycles=%0d", cycles);
            $finish;
        end
    end
endmodule
"""


def build_perf_tb():
    tb = "/tmp/bench_ack_tb.v"
    with open(tb, "w") as f:
        f.write(PERF_TB_SRC)
    bin_ = "/tmp/bench_ack_tb"
    subprocess.run(
        ["iverilog", "-g2012", "-o", bin_, tb, str(RTL_DIR / "f18a_core.v")],
        check=True, env=ENV,
    )
    return bin_


def patch_aforth(m, n):
    src = (PROGRAMS_DIR / "aforth_ack.ga").read_text()
    return re.sub(r"\b\d+\s+\d+\s+ack\b", f"{m} {n} ack", src, count=1)


def run_aforth(bin_, m, n):
    aforth_src = patch_aforth(m, n)
    f18a_src = ga_aforth.aforth_to_f18a_source(aforth_src)
    with tempfile.NamedTemporaryFile("w", suffix=".f18a", delete=False) as f:
        f.write(f18a_src)
        p = f.name
    try:
        asm.assemble_to_hex(p, "/tmp/aforth.hex", "/tmp/aforth.lst", mem_size=1024)
    finally:
        os.unlink(p)
    proc = subprocess.run([bin_], capture_output=True, text=True, env=ENV)
    m_ = re.search(r"RESULT 0x([0-9a-f]{2}) CYCLES (\d+) PEAK_RSP (\d+) PEAK_DSP (\d+)",
                   proc.stdout)
    if not m_:
        return None
    return int(m_.group(1), 16), int(m_.group(2)), int(m_.group(3)), int(m_.group(4))


def run_native(m, n):
    proc = subprocess.run(
        ["vvp", "/tmp/tb_ack", f"+m={m}", f"+n={n}", "+r=1"],
        capture_output=True, text=True, env=ENV,
    )
    m_ = re.search(r"\[cycle (\d+)\] A\(\d+, \d+\) = (\d+) .*peak dstk = (\d+)",
                   proc.stdout)
    if not m_:
        return None
    return int(m_.group(2)), int(m_.group(1)), int(m_.group(3))


# ── Hardware path: stream native ackermann to /dev/ttyUSB0 ──────────────
# Borrowed from tools/ack_host.py. Streams the loader + program payload
# + jump:0, then the (count, n, m) inputs. Reads N result bytes, times
# the round-trip wall-clock. Per-iter time is the right unit once the
# one-shot ~33 ms program upload is amortized over -r N.
BOOT_LOAD_A    = 0x11FE0
BOOT_STORE_INC = 0x10DE0
BOOT_JUMP_0    = 0x04000
TTY  = os.environ.get("TTY", "/dev/ttyUSB0")
BAUD = int(os.environ.get("BAUD", "115200"))
SYS_HZ = int(os.environ.get("SYS_HZ", "27000000"))


def _word_bytes(w: int) -> bytes:
    return bytes([w & 0xFF, (w >> 8) & 0xFF, (w >> 16) & 0x03])


def _load_native_hex(path: Path) -> list[int]:
    raw: list[int] = []
    used = None
    for line in path.read_text().splitlines():
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
        while used > 0 and raw[used - 1] == 0:
            used -= 1
    return raw[:used]


def _wait_for_byte(s, deadline: float) -> bytes:
    """Poll s.in_waiting in a tight loop and return as soon as the
    first non-0xFF byte arrives. pyserial's `read(N)` blocks for the
    full configured timeout even when the byte is already in the
    kernel buffer, which inflates wall-time measurements by up to
    `serial.timeout`. Polling in_waiting gives ~100 µs resolution —
    fine for any sensible -r."""
    while time.monotonic() < deadline:
        n = s.in_waiting
        if n:
            chunk = bytes(c for c in s.read(n) if c != 0xFF)
            if chunk:
                return chunk
    return b""


def run_native_hw(m: int, n: int, repeat: int, timeout: float = 30.0):
    """Run iterative ackermann on the board for `repeat` iterations.

    Returns (result, elapsed_ms, per_iter_ms, per_iter_cycles_est).
    `per_iter_ms` is COMPUTE only — the UART transit time of the
    program upload + 3 input words + result byte (a fixed ~33 ms at
    115200 baud for our ~63-cell programs) is subtracted from the
    measured elapsed wall, then divided by `repeat`. Accurate to a
    few percent for `repeat` ≥ 100 on workloads above a few hundred
    cycles; lighter workloads still see some residual program-side
    per-iter restart overhead inside ackermann.f18a's start_run.
    """
    import serial
    prog = _load_native_hex(Path("/tmp/ack.hex"))
    # timeout=0 → non-blocking reads; we poll in_waiting ourselves.
    s = serial.Serial(TTY, BAUD, timeout=0)
    try:
        time.sleep(0.05)
        s.reset_input_buffer()

        payload = [BOOT_LOAD_A, 0]
        for w in prog:
            payload += [BOOT_STORE_INC, w]
        payload.append(BOOT_JUMP_0)
        host_blob = b"".join(_word_bytes(w) for w in payload)
        host_blob += _word_bytes(repeat)
        host_blob += _word_bytes(n)
        host_blob += _word_bytes(m)

        t0 = time.monotonic()
        s.write(host_blob)
        s.flush()

        got = _wait_for_byte(s, t0 + timeout)
        elapsed_ms = 1000 * (time.monotonic() - t0)
    finally:
        s.close()

    if not got:
        return None
    # 10 UART bits per byte (8 data + 1 start + 1 stop), bidirectional
    # transit times subtracted from the measured wall.
    upload_ms  = (len(host_blob) * 10) / BAUD * 1000.0
    result_ms  = (1 * 10) / BAUD * 1000.0  # the one result byte
    compute_ms = max(0.0, elapsed_ms - upload_ms - result_ms)
    per_iter_ms = compute_ms / repeat
    cycles_est = int(per_iter_ms * 1e-3 * SYS_HZ)
    return got[0], elapsed_ms, per_iter_ms, cycles_est


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--hw", action="store_true",
                    help="Also run the native ackermann on the board "
                         "(requires programmed bitstream + serial cable)")
    ap.add_argument("--hw-repeat", type=int, default=100,
                    help="Iterations per case in --hw mode (default 100)")
    args = ap.parse_args()

    # Make sure the native ackermann TB and hex are built.
    subprocess.run(["make", "/tmp/tb_ack", "/tmp/ack.hex"],
                   check=True, env=ENV, cwd=REPO_ROOT, capture_output=True)
    aforth_bin = build_perf_tb()

    cases = [(0, 0), (0, 5), (1, 1), (1, 5), (2, 0), (2, 1),
             (2, 2), (2, 3), (2, 7), (3, 0), (3, 1), (3, 2), (3, 3)]

    if args.hw:
        # HW mode: report sim cycles + on-board per-iter measurement
        # for the iterative variant. (aforth on hw would need source
        # patching to add a repeat loop — left out here.)
        print(f"{'A(m,n)':<9} {'res':<5} "
              f"{'sim cyc':>10} "
              f"{'hw ms/iter':>12} "
              f"{'hw cyc/iter':>13} "
              f"{'hw/sim':>8}")
        print("-" * 64)
        for m, n in cases:
            nat = run_native(m, n)
            if not nat:
                print(f"A({m},{n}): sim run failed")
                continue
            n_res, n_cyc, n_dsp = nat
            hw = run_native_hw(m, n, args.hw_repeat)
            if not hw:
                print(f"A({m},{n})    {n_res:<5} {n_cyc:>10} "
                      f"{'(no result)':>12}")
                continue
            h_res, _total, h_ms, h_cyc = hw
            ratio = h_cyc / n_cyc if n_cyc else float("inf")
            ok = "" if h_res == n_res else f"  MISMATCH({h_res}!={n_res})"
            print(f"A({m},{n})    {n_res:<5} "
                  f"{n_cyc:>10} "
                  f"{h_ms:>12.3f} "
                  f"{h_cyc:>13} "
                  f"{ratio:>7.2f}x{ok}")
        print()
        print(f"hw cyc/iter = (wall_ms − upload_ms − 0.087 ms) × "
              f"{SYS_HZ:,} Hz / r. Matches sim cycles to <1 % once the "
              f"compute dominates (workloads ≥ ~1 k cycles); lighter "
              f"workloads see polling/program-startup quantization.")
        return

    print(f"{'A(m,n)':<9} {'res':<5} "
          f"{'native cyc':>11} {'native dsp':>11} "
          f"{'aforth cyc':>11} {'aforth dsp':>11} {'aforth rsp':>11} "
          f"{'speedup':>9}")
    print("-" * 88)
    for m, n in cases:
        nat = run_native(m, n)
        afo = run_aforth(aforth_bin, m, n)
        if not nat or not afo:
            print(f"A({m},{n}): native={nat} aforth={afo}")
            continue
        n_res, n_cyc, n_dsp = nat
        a_res, a_cyc, a_dsp, a_rsp = afo
        speedup = n_cyc / a_cyc
        ok = "" if a_res == n_res else f"  MISMATCH({a_res}!={n_res})"
        print(f"A({m},{n})    {n_res:<5} "
              f"{n_cyc:>11} {n_dsp:>11} "
              f"{a_cyc:>11} {a_dsp:>11} {a_rsp:>11} "
              f"{speedup:>8.2f}×{ok}")


if __name__ == "__main__":
    main()
