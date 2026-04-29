#!/usr/bin/env python3
import re
import subprocess
from pathlib import Path

import asm

ROOT = Path(__file__).resolve().parent
MEM_SIZE = 1024
TB_BIN = Path("/tmp/tb_fullisa_case")
CASE_HEX = Path("/tmp/current_case.hex")

CASES = [
    ("tests_array_forth/case_1.f18a", 0x50),
    ("tests_array_forth/case_pop.f18a", 0x50),
    ("tests_array_forth/case_unext.f18a", 0x50),
    ("tests_array_forth/case_multiplyStepOdd.f18a", 0x50),
]


def combine_image(case_path: Path) -> None:
    flat = [0] * MEM_SIZE
    case_origin, case_words, *_ = asm.assemble_to_words(str(case_path), size=MEM_SIZE)
    for i, word in enumerate(case_words):
        flat[case_origin + i] = word

    CASE_HEX.write_text("// auto-generated case image\n" + "".join(f"{word:05x}\n" for word in flat))


def build_tb() -> None:
    subprocess.run(
        [
            "iverilog",
            "-g2012",
            "-DFULLISA",
            "-o",
            str(TB_BIN),
            "tb_fullisa_case.v",
            "c1_bram.v",
            "f18a_core.v",
        ],
        cwd=ROOT,
        check=True,
    )


def run_case(case_rel: str, expect_byte: int) -> None:
    case_path = ROOT / case_rel
    combine_image(case_path)
    proc = subprocess.run([str(TB_BIN)], capture_output=True, text=True, check=True)
    match = re.search(r"EMIT 0x([0-9a-fA-F]{2})", proc.stdout)
    if not match:
        raise RuntimeError(f"{case_rel}: no EMIT line\n{proc.stdout}")
    got = int(match.group(1), 16)
    if got != expect_byte:
        raise RuntimeError(f"{case_rel}: expected 0x{expect_byte:02x}, got 0x{got:02x}\n{proc.stdout}")
    print(f"PASS {case_rel}: 0x{got:02x}")


def main() -> None:
    build_tb()
    for case_rel, expect in CASES:
        run_case(case_rel, expect)


if __name__ == "__main__":
    main()
