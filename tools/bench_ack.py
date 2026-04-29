#!/usr/bin/env python3
"""Benchmark aforth_ack.ga (recursive Forth via ga-tools) vs
ackermann.f18a (iterative hand-asm). Measures cycles to first
UART_TX write in iverilog sim — eliminates UART/streaming overhead
and gives a clean compute-only comparison.

Run: PATH=/opt/oss-cad-suite/bin:$PATH python3 bench_ack.py
"""
from __future__ import annotations

import os
import re
import subprocess
import sys
import tempfile
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


def main():
    # Make sure the native ackermann TB and hex are built.
    subprocess.run(["make", "/tmp/tb_ack", "/tmp/ack.hex"],
                   check=True, env=ENV, cwd=REPO_ROOT, capture_output=True)
    aforth_bin = build_perf_tb()

    cases = [(0, 0), (0, 5), (1, 1), (1, 5), (2, 0), (2, 1),
             (2, 2), (2, 3), (2, 7), (3, 0), (3, 1), (3, 2), (3, 3)]

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
