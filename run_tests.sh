#!/usr/bin/env bash
# Simulation unit tests for the F18A core, SoC, and host-side Forth compiler.
set -euo pipefail
cd "$(dirname "$0")"
PATH="/opt/oss-cad-suite/bin:$PATH"

iverilog -g2012 -o /tmp/tb_ops rtl/tb_ops.v rtl/f18a_core.v
vvp /tmp/tb_ops

iverilog -g2012 -o /tmp/tb_extarith rtl/tb_extarith.v rtl/f18a_core.v
vvp /tmp/tb_extarith

iverilog -g2012 -o /tmp/tb_muls rtl/tb_muls.v rtl/c1_bram.v rtl/f18a_core.v
vvp /tmp/tb_muls

python3 tools/run_fullisa_cases.py
python3 tools/run_semantic_cases.py

# tb_ack — iterative Ackermann sim (smoke test that the streaming
# INST_PORT path still computes the right answer).
iverilog -g2012 -o /tmp/tb_ack rtl/tb_ack.v rtl/f18a_core.v
python3 tools/asm.py -hex programs/ackermann.f18a /tmp/ack.hex /tmp/ack.lst
vvp /tmp/tb_ack +m=3 +n=3 | grep 'A(3, 3) = 61' >/dev/null \
    && echo "PASS tb_ack: A(3, 3) = 61"

python3 tools/test_forth_compile.py

# ga-tools aforth bridge — only runs if ga_tools is importable. Compiles
# aforth_ack.ga, runs it through the perf TB used by bench_ack.py, and
# checks A(2, 2) == 7. Skips silently if ga-tools isn't installed.
python3 - <<'PY'
import sys, importlib.util
if importlib.util.find_spec("ga_tools") is None:
    # ga-tools may live in the user site-packages (pip install --user)
    import os
    user_site = os.path.expanduser("~/.local/lib/python3.13/site-packages")
    if os.path.isdir(user_site):
        sys.path.insert(0, user_site)
if importlib.util.find_spec("ga_tools") is None:
    print("SKIP ga_aforth (install ga-tools: pip install --user --break-system-packages ga-tools)")
    sys.exit(0)
import os, re, subprocess, tempfile
from pathlib import Path
ROOT = Path(__file__).resolve().parent if "__file__" in dir() else Path.cwd()
sys.path.insert(0, str(ROOT / "tools"))
import asm, ga_aforth
src = (ROOT / "programs" / "aforth_ack.ga").read_text()
src = re.sub(r"\b\d+\s+\d+\s+ack\b", "2 2 ack", src, count=1)
f18a_src = ga_aforth.aforth_to_f18a_source(src)
with tempfile.NamedTemporaryFile("w", suffix=".f18a", delete=False) as f:
    f.write(f18a_src); p = f.name
asm.assemble_to_hex(p, "/tmp/aforth.hex", "/tmp/aforth.lst", mem_size=1024)
os.unlink(p)
import bench_ack
tb = "/tmp/run_tests_aforth_tb.v"; open(tb, "w").write(bench_ack.PERF_TB_SRC)
subprocess.run(["iverilog", "-g2012", "-o", "/tmp/run_tests_aforth", tb,
                str(ROOT / "rtl" / "f18a_core.v")], check=True)
out = subprocess.run(["vvp", "/tmp/run_tests_aforth"], capture_output=True, text=True).stdout
m = re.search(r"RESULT 0x([0-9a-f]{2})", out)
result = int(m.group(1), 16) if m else None
assert result == 7, f"FAIL ga_aforth: A(2,2) = {result} (expected 7); output:\n{out}"
print("PASS ga_aforth: A(2, 2) = 7")
PY
