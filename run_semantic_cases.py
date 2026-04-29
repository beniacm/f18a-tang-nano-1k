#!/usr/bin/env python3
"""ArrayForth-style semantic regression cases for the local F18A core.

The existing tb_ops.v bench checks individual opcodes and a few control-flow
edges. This runner complements that with small end-to-end programs, closer in
spirit to the reference ArrayForth interpreter tests: each case runs a snippet
of F18A source and checks the resulting architectural state.
"""

from __future__ import annotations

import subprocess
import textwrap
from dataclasses import dataclass, field
from pathlib import Path

import asm


ROOT = Path(__file__).resolve().parent
CASE_HEX = Path("/tmp/f18a_semantic_case.hex")
CASE_TB = Path("/tmp/tb_semantic_case.v")
CASE_BIN = Path("/tmp/tb_semantic_case")
MEM_SIZE = 512


@dataclass(frozen=True)
class Case:
    name: str
    source: str
    stop_pc: int
    expect_regs: dict[str, int]
    expect_mem: dict[int, int] = field(default_factory=dict)
    init_mem: dict[int, int] = field(default_factory=dict)
    min_cycles: int = 8
    max_cycles: int = 240


def src(body: str) -> str:
    return "#origin 0x000\n\n" + textwrap.dedent(body).strip() + "\n"


CASES = [
    Case(
        name="case_1",
        source=src(
            """
            start:
                @p @p +
                [2]
                [3]
            halt:
                jump:halt
            """
        ),
        stop_pc=0x003,
        expect_regs={"p": 0x003, "t": 5, "a": 0, "b": 0, "r": 0, "s": 0},
    ),
    Case(
        name="case_2",
        source=src(
            """
            start:
                @p -
                [0]
            halt:
                jump:halt
            """
        ),
        stop_pc=0x002,
        expect_regs={"p": 0x002, "t": 0x3FFFF, "a": 0, "b": 0, "r": 0, "s": 0},
    ),
    Case(
        name="case_3",
        source=src(
            """
            start:
                @p b!
                [4]
                @p
                [42]
                !b
            halt:
                jump:halt
            """
        ),
        stop_pc=0x005,
        expect_regs={"p": 0x005, "b": 4, "a": 0, "r": 0, "s": 0, "t": 0},
        expect_mem={4: 42},
    ),
    Case(
        name="case_fetchP",
        source=src(
            """
            start:
                @p
                [42]
            halt:
                jump:halt
            """
        ),
        stop_pc=0x002,
        expect_regs={"p": 0x002, "t": 42, "a": 0, "b": 0, "r": 0, "s": 0},
    ),
    Case(
        name="case_fetchPlus",
        source=src(
            """
            start:
                @p a!
                [10]
                @+
            halt:
                jump:halt
            """
        ),
        stop_pc=0x003,
        expect_regs={"p": 0x003, "t": 0x0CAFE, "a": 11, "b": 0, "r": 0, "s": 0},
        init_mem={10: 0x0CAFE},
    ),
    Case(
        name="case_fetchB",
        source=src(
            """
            start:
                @p b!
                [9]
                @b
            halt:
                jump:halt
            """
        ),
        stop_pc=0x003,
        expect_regs={"p": 0x003, "t": 0x0BEEF, "a": 0, "b": 9, "r": 0, "s": 0},
        init_mem={9: 0x0BEEF},
    ),
    Case(
        name="case_fetch",
        source=src(
            """
            start:
                @p a!
                [7]
                @
            halt:
                jump:halt
            """
        ),
        stop_pc=0x003,
        expect_regs={"p": 0x003, "t": 0x0F00D, "a": 7, "b": 0, "r": 0, "s": 0},
        init_mem={7: 0x0F00D},
    ),
    Case(
        name="case_storeP",
        source=src(
            """
            start:
                @p !p
                [42]
                .
            halt:
                jump:halt
            """
        ),
        stop_pc=0x003,
        expect_regs={"p": 0x003, "a": 0, "b": 0, "r": 0, "s": 0, "t": 0},
        expect_mem={2: 42},
    ),
    Case(
        name="case_storePlus",
        source=src(
            """
            start:
                @p a!
                [10]
                @p
                [42]
                !+
            halt:
                jump:halt
            """
        ),
        stop_pc=0x005,
        expect_regs={"p": 0x005, "a": 11, "b": 0, "r": 0, "s": 0, "t": 0},
        expect_mem={10: 42},
    ),
    Case(
        name="case_storeB",
        source=src(
            """
            start:
                @p b!
                [10]
                @p
                [42]
                !b
            halt:
                jump:halt
            """
        ),
        stop_pc=0x005,
        expect_regs={"p": 0x005, "a": 0, "b": 10, "r": 0, "s": 0, "t": 0},
        expect_mem={10: 42},
    ),
    Case(
        name="case_store",
        source=src(
            """
            start:
                @p a!
                [10]
                @p
                [42]
                !
            halt:
                jump:halt
            """
        ),
        stop_pc=0x005,
        expect_regs={"p": 0x005, "a": 10, "b": 0, "r": 0, "s": 0, "t": 0},
        expect_mem={10: 42},
    ),
    Case(
        name="case_pushPop",
        source=src(
            """
            start:
                @p push pop
                [42]
            halt:
                jump:halt
            """
        ),
        stop_pc=0x002,
        expect_regs={"p": 0x002, "t": 42, "a": 0, "b": 0, "r": 0, "s": 0},
    ),
    Case(
        name="case_over",
        source=src(
            """
            start:
                @p @p over
                [1]
                [2]
            halt:
                jump:halt
            """
        ),
        stop_pc=0x003,
        expect_regs={"p": 0x003, "t": 1, "s": 2, "dstk_top": 1, "a": 0, "b": 0, "r": 0},
    ),
    Case(
        name="case_a",
        source=src(
            """
            start:
                @p a! a
                [42]
            halt:
                jump:halt
            """
        ),
        stop_pc=0x002,
        expect_regs={"p": 0x002, "a": 42, "t": 42, "b": 0, "r": 0, "s": 0},
    ),
    Case(
        name="case_call",
        source=src(
            """
            start:
                call:target
                .
                .
                .
            target:
                jump:target
            """
        ),
        stop_pc=0x004,
        expect_regs={"p": 0x004, "r": 1, "a": 0, "b": 0, "s": 0, "t": 0},
    ),
    Case(
        name="case_unextCounter",
        source=src(
            """
            start:
                @p a!
                [0]
                @p push jump:loop
                [3]
            loop:
                @+ . . unext
            halt:
                jump:halt
            """
        ),
        stop_pc=0x005,
        expect_regs={"p": 0x005, "a": 4, "b": 0},
        max_cycles=400,
    ),
    Case(
        name="case_nextCounter",
        source=src(
            """
            start:
                @p push
                [2]
            loop:
                next:loop
            halt:
                jump:halt
            """
        ),
        stop_pc=0x003,
        expect_regs={"p": 0x003},
        max_cycles=300,
    ),
    Case(
        name="case_ifTaken",
        source=src(
            """
            start:
                if:halt
                .
            halt:
                jump:halt
            """
        ),
        stop_pc=0x002,
        expect_regs={"p": 0x002, "a": 0, "b": 0, "r": 0, "s": 0, "t": 0},
    ),
    Case(
        name="case_ifFallthrough",
        source=src(
            """
            start:
                @p
                [1]
                if:dead
            halt:
                jump:halt
            dead:
                jump:dead
            """
        ),
        stop_pc=0x003,
        expect_regs={"p": 0x003, "a": 0, "b": 0, "r": 0, "s": 0, "t": 1},
    ),
    Case(
        name="case_minusIfTaken",
        source=src(
            """
            start:
                -if:halt
                .
            halt:
                jump:halt
            """
        ),
        stop_pc=0x002,
        expect_regs={"p": 0x002, "a": 0, "b": 0, "r": 0, "s": 0, "t": 0},
    ),
    Case(
        name="case_minusIfFallthrough",
        source=src(
            """
            start:
                @p
                [0x20000]
                -if:dead
            halt:
                jump:halt
            dead:
                jump:dead
            """
        ),
        stop_pc=0x003,
        expect_regs={"p": 0x003, "a": 0, "b": 0, "r": 0, "s": 0, "t": 0x20000},
    ),
    Case(
        name="case_times2",
        source=src(
            """
            start:
                @p 2*
                [2]
            halt:
                jump:halt
            """
        ),
        stop_pc=0x002,
        expect_regs={"p": 0x002, "t": 4, "a": 0, "b": 0, "r": 0, "s": 0},
    ),
    Case(
        name="case_div2",
        source=src(
            """
            start:
                @p 2/
                [4]
            halt:
                jump:halt
            """
        ),
        stop_pc=0x002,
        expect_regs={"p": 0x002, "t": 2, "a": 0, "b": 0, "r": 0, "s": 0},
    ),
    Case(
        name="case_setB",
        source=src(
            """
            start:
                @p b!
                [42]
            halt:
                jump:halt
            """
        ),
        stop_pc=0x002,
        expect_regs={"p": 0x002, "b": 42, "a": 0, "r": 0, "s": 0, "t": 0},
    ),
    Case(
        name="case_setA",
        source=src(
            """
            start:
                @p a!
                [42]
            halt:
                jump:halt
            """
        ),
        stop_pc=0x002,
        expect_regs={"p": 0x002, "a": 42, "b": 0, "r": 0, "s": 0, "t": 0},
    ),
]


REG_EXPR = {
    "a": "cpu.A",
    "b": "cpu.B",
    "p": "dbg_P",
    "r": "cpu.R",
    "s": "cpu.S",
    "t": "dbg_T",
    "dstk_top": "cpu.dstk[cpu.dsp - 6'd1]",
    "rstk_top": "cpu.rstk[cpu.rsp - 4'd1]",
}


TB_TEMPLATE = """\
`timescale 1ns/1ps
`default_nettype none

module tb_semantic_case;
    reg clk = 0;
    always #10 clk = ~clk;

    reg [17:0] mem [0:511];
    reg resetn = 1'b0;
    initial begin
        $readmemh("{hex_path}", mem);
        repeat (2) @(posedge clk);
        resetn = 1'b1;
    end

    wire [8:0]  mem_addr;
    wire        mem_we;
    wire [17:0] mem_wdata;
    reg  [17:0] mem_rdata;

    wire [17:0] dbg_T, dbg_I;
    wire [8:0]  dbg_P;
    wire [1:0]  dbg_slot;

    f18a_core #(.ADDR_BITS(9), .RESET_PC(9'h000)) cpu (
        .clk       (clk),
        .resetn    (resetn),
        .mem_addr  (mem_addr),
        .mem_we    (mem_we),
        .mem_wdata (mem_wdata),
        .mem_rdata (mem_rdata),
        .mem_ready (1'b1),
        .dbg_T     (dbg_T),
        .dbg_I     (dbg_I),
        .dbg_P     (dbg_P),
        .dbg_slot  (dbg_slot)
    );

    always @(posedge clk)
        if (mem_we) mem[mem_addr] <= mem_wdata;
    always @(*) mem_rdata = mem[mem_addr];

    integer cycles = 0;
    integer failed = 0;

    task check18;
        input [255:0] name;
        input [17:0] got;
        input [17:0] want;
        begin
            if (got !== want) begin
                $display("FAIL {case_name} %0s: got 0x%05h want 0x%05h", name, got, want);
                failed = failed + 1;
            end
        end
    endtask

    task check9;
        input [255:0] name;
        input [8:0] got;
        input [8:0] want;
        begin
            if (got !== want) begin
                $display("FAIL {case_name} %0s: got 0x%03h want 0x%03h", name, got, want);
                failed = failed + 1;
            end
        end
    endtask

    task finish_case;
        begin
{checks}
            if (failed == 0) begin
                $display("PASS {case_name}");
                $finish;
            end else begin
                $finish_and_return(1);
            end
        end
    endtask

    always @(posedge clk) begin
        if (!resetn) begin
            cycles <= 0;
        end else begin
            cycles <= cycles + 1;
            if (cycles >= {min_cycles} && dbg_P === 9'h{stop_pc:03x} && cpu.st == 3'd0)
                finish_case();
            if (cycles > {max_cycles}) begin
                $display("FAIL {case_name}: timeout at cycle %0d P=0x%03h", cycles, dbg_P);
                $finish_and_return(1);
            end
        end
    end
endmodule
"""


def assemble_case(case: Case) -> None:
    origin, words, *_ = asm._assemble_raw(f"<{case.name}>", case.source.splitlines(), MEM_SIZE)
    flat = [0] * MEM_SIZE
    for i, word in enumerate(words):
        flat[origin + i] = word
    for addr, value in case.init_mem.items():
        flat[addr] = value & 0x3FFFF
    CASE_HEX.write_text("// auto-generated semantic case image\n" + "".join(f"{word:05x}\n" for word in flat))


def render_checks(case: Case) -> str:
    lines: list[str] = []
    for key, value in case.expect_regs.items():
        expr = REG_EXPR[key]
        if key == "p":
            lines.append(f'            check9("{key}", {expr}, 9\'h{value:03x});')
        else:
            lines.append(f'            check18("{key}", {expr}, 18\'h{value:05x});')
    for addr, value in sorted(case.expect_mem.items()):
        lines.append(f'            check18("mem[{addr}]", mem[{addr}], 18\'h{value:05x});')
    return "\n".join(lines)


def build_tb(case: Case) -> None:
    CASE_TB.write_text(
        TB_TEMPLATE.format(
            hex_path=CASE_HEX,
            case_name=case.name,
            checks=render_checks(case),
            min_cycles=case.min_cycles,
            max_cycles=case.max_cycles,
            stop_pc=case.stop_pc,
        )
    )
    subprocess.run(
        ["iverilog", "-g2012", "-o", str(CASE_BIN), str(CASE_TB), "f18a_core.v"],
        cwd=ROOT,
        check=True,
    )


def run_case(case: Case) -> None:
    assemble_case(case)
    build_tb(case)
    proc = subprocess.run([str(CASE_BIN)], cwd=ROOT, text=True, capture_output=True)
    if proc.returncode != 0:
        raise RuntimeError(proc.stdout + proc.stderr)
    if f"PASS {case.name}" not in proc.stdout:
        raise RuntimeError(f"{case.name}: missing PASS line\n{proc.stdout}")
    print(f"PASS {case.name}")


def main() -> None:
    for case in CASES:
        run_case(case)


if __name__ == "__main__":
    main()
