DEVICE  = GW1NZ-LV1QN48C6/I5
FAMILY  = GW1NZ-1
TOP     = top
PATH    := /opt/oss-cad-suite/bin:$(PATH)

# Tang Nano 1K F18A SoC. Default sys_clk = 27 MHz board oscillator. The
# bitstream contains no boot ROM: the core comes up with P pointing at
# INST_PORT (DB001 §3.3.2 port-execution) and stalls there until the
# host streams a 3-byte-per-instruction bootstrap loader over UART.
#
# Override the clock with `make f18a.fs PLL_FREQ=54` (allowed values:
# 27, 54, 81, 108). PLL_FREQ=27 keeps the board clock straight through;
# anything else routes through the on-chip rPLL (rtl/gowin_rpll.v).
# UART_DIV auto-scales from SYS_HZ inside rtl/f18a_soc.v.
PLL_FREQ ?= 27

# Stack depths default to 64-deep dstack / 128-deep rstack (BSRAM-backed),
# set inside rtl/f18a_soc.v. Override here for experiments — e.g.
# `make f18a.fs DSTK_DEPTH=8 RSTK_DEPTH=8` for the GA144-faithful 8/8
# build (passes everything except recursive Forth, which overruns
# rstk=8).
DSTK_DEPTH ?=
RSTK_DEPTH ?=

ifeq ($(PLL_FREQ),27)
SRC     = rtl/f18a_core.v rtl/f18a_soc.v rtl/c1_bram.v
DEFS    =
else
SRC     = rtl/f18a_core.v rtl/f18a_soc.v rtl/c1_bram.v rtl/gowin_rpll.v
DEFS    = -DPLL_FREQ=$(PLL_FREQ)
endif

ifneq ($(DSTK_DEPTH),)
DEFS += -DDSTK_DEPTH=$(DSTK_DEPTH)
endif
ifneq ($(RSTK_DEPTH),)
DEFS += -DRSTK_DEPTH=$(RSTK_DEPTH)
endif

all: f18a.fs

f18a.json: $(SRC)
	yosys -p "read_verilog $(DEFS) $(SRC); synth_gowin -top $(TOP) -json f18a.json"

f18a.pnr: f18a.json rtl/tang_nano_1k.cst
	nextpnr-himbaechel --json f18a.json --write f18a.pnr \
		--device $(DEVICE) \
		--freq $(PLL_FREQ) \
		--vopt family=$(FAMILY) \
		--vopt cst=rtl/tang_nano_1k.cst

f18a.fs: f18a.pnr
	gowin_pack -d $(FAMILY) -o f18a.fs f18a.pnr

# Volatile load to SRAM (lost on power cycle, faster iteration).
flash-sram: f18a.fs
	openFPGALoader -b tangnano1k -m f18a.fs

# Persistent SPI-flash load.
flash: f18a.fs
	openFPGALoader -b tangnano1k f18a.fs

clean:
	rm -f f18a.json f18a.pnr f18a.fs

.PHONY: all flash flash-sram clean ack-sim ack ack-hw test

test:
	bash run_tests.sh

# ── Ackermann on the simulated F18A core ──────────────────────────────
# Build with `make ack-sim`, run with `make ack M=3 N=3 R=1`.
M ?= 3
N ?= 3
R ?= 1

/tmp/ack.hex: programs/ackermann.f18a tools/asm.py
	python3 tools/asm.py -hex programs/ackermann.f18a /tmp/ack.hex /tmp/ack.lst

/tmp/tb_ack: rtl/tb_ack.v rtl/f18a_core.v
	iverilog -g2012 -o /tmp/tb_ack rtl/tb_ack.v rtl/f18a_core.v

ack-sim: /tmp/ack.hex /tmp/tb_ack

ack: ack-sim
	vvp /tmp/tb_ack +m=$(M) +n=$(N) +r=$(R) | grep 'A('

# ── Run on hardware ───────────────────────────────────────────────────
# Stream bootstrap loader + ackermann + inputs to the FPGA over UART.
# Re-flash (`make flash-sram`) between runs to start from a clean POR.
ack-hw: /tmp/ack.hex
	python3 tools/ack_host.py -m $(M) -n $(N) -r $(R)
