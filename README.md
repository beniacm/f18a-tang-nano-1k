# F18A softcore on Tang Nano 1K

An 18-bit, 4-slots-per-word [F18A](https://www.greenarraychips.com/) softcore
that fits on the Sipeed Tang Nano 1K (Gowin GW1NZ-1, 1152 LUT4 / 4 BSRAM),
with a host-side toolchain that streams programs straight into the core
over UART — no boot ROM, no in-FPGA monitor.

The bitstream comes up with `P` pointing at **INST_PORT** (DB001 §3.3.2
port-execution). The core stalls on its first fetch until 3 bytes have
been streamed in over UART; the SoC assembles them into one 18-bit
instruction word and lets the fetch advance. `P` stays at the port
across that fetch, so a single streamed instruction word can be executed
before the program is ever resident in RAM. The host uses that window to
stream a tiny self-loader, the program payload, and a `jump:0` to leave
port-execution and run from RAM.

A program signals "I'm done, send me another" by jumping back to
`INST_PORT` — that hands control straight back to the streaming-fetch
state, which from the F18A's perspective is identical to power-on, so
the next streamed program runs exactly like the first one. The on-board
**B button** (pin 44, nRST) drives the SoC's `resetn` low while held,
giving you a hardware soft-reset back to INST_PORT without re-flashing.

## Quickstart

```sh
# 1. install oss-cad-suite under /opt/oss-cad-suite, then:
make f18a.fs                                # build bitstream
make flash-sram                             # volatile SRAM load

# 2. run something
python3 port_exec_demo.py                   # streams 12 bytes → emits 'X'
python3 ack_host.py -m 3 -n 3               # iterative Ackermann
python3 ga_aforth.py --hw aforth_demo.ga    # ga-tools aforth → "01234\n"
python3 ga_aforth.py --hw aforth_hue.ga --fire-and-forget   # RGB hue
python3 fft_host.py                         # 128-word echo round-trip
```

## Layout

```
.
├── f18a_core.v          # F18A softcore (18-bit, 4-slot packed ISA, 11-bit addrs)
├── f18a_soc.v           # SoC: core + UART + INST_PORT byte-assembler + LED/BUTTON
├── c1_bram.v            # Single-port 18-bit BSRAM
├── gowin_rpll.v         # Optional rPLL wrapper (build with PLL_FREQ=…)
├── tang_nano_1k.cst     # Pin constraints
├── Makefile             # `make f18a.fs`, `make flash-sram`, `make ack`, `make test`
│
├── asm.py               # F18A assembler (#define / #variable, slot-3 NOP `.`)
├── ackermann.f18a       # Iterative Ackermann demo program
├── lib.f18a             # Helper words / examples
├── hello.f18a           # "hi\n" UART loop
├── core1.f18a           # 'B'-spammer
├── swap_test.f18a       # SWAP via memory scratch
├── muls_bench.f18a      # MULS microbench
├── fft_echo.f18a        # Phase-1 FFT crunch test (128-word echo)
├── examples/            # More small demos (count.f18a)
│
├── tb_ack.v             # Ackermann simulation testbench
├── tb_ops.v             # Per-opcode regression
├── tb_extarith.v        # Extended-arith (carry / +* edge cases)
├── tb_muls.v            # Native MULS through the BSRAM path
├── tb_muls_bench.v      # MULS microbench TB
├── tb_fullisa_case.v    # Per-instruction semantic harness
├── tb_forth_runtime.v   # TB that backs forth_run.run_sim
│
├── run_tests.sh         # Whole regression in one shot
├── run_fullisa_cases.py # Array-Forth opcode case images
├── run_semantic_cases.py# 23 per-instruction semantic checks
├── run_muls_bench.py    # MULS-bench driver
├── test_forth_compile.py# Forth → F18A compiler smoke tests
│
├── forth_compile.py     # Forth → F18A asm compiler (host side)
├── forth_run.py         # Sim and hardware back-ends (iverilog + UART)
├── forth_repl.py        # Interactive Forth REPL
├── ack_forth.py         # Ackermann benchmark in pure Forth
│
├── ga_aforth.py         # Bridge to mschuldt/ga-tools' aforth compiler
├── aforth_demo.ga       # for/next loop emitting "01234\n"
├── aforth_ack.ga        # Recursive Ackermann in aforth
├── aforth_buttons.ga    # Read BUTTON GPIO, mirror to RGB LED
├── aforth_hue.ga        # RGB-LED hue cycler
├── bench_ack.py         # Cycle-count comparison: native iter vs aforth
│
├── ack_host.py          # Host streamer for the Ackermann demo
├── port_exec_demo.py    # Tiny "emit X over UART" port-execution demo
├── fft_host.py          # Streams + validates the fft_echo program
│
└── NEXT.md              # Open work / queued ideas
```

## Memory map

11-bit address space. Bit 10 selects the I/O band; the low 1024 cells
are general-purpose BSRAM. The full 1K is yours for code + data.

```
0x000..0x3FF   core RAM (host streams the loader here, then jumps in)
0x7F0..0x7F4   memory-mapped peripherals:
                 0x7F0 UART_RX     read pops one byte (blocks if none)
                 0x7F1 UART_TX     write enqueues a byte (blocks if full)
                 0x7F2 UART_STAT   bit0 rx_avail, bit1 tx_room
                 0x7F3 LED         bits 2:0 = R, G, B (common-anode at the pad)
                 0x7F4 BUTTON      bit0 = button A pressed (active high)
0x7F9          INST_PORT — 3 UART bytes → one 18-bit word
                 (read blocks until 3 bytes have been assembled)
```

`UART_TX`, `UART_RX`, and `INST_PORT` are GA144-style **blocking** neighbor
ports: the SoC drives `mem_ready=0` until the operation can complete.
Every other I/O register stays non-blocking like BRAM. The 1024-cell BSRAM
cap is forced by an apycula `gowin_pack` quirk on GW1NZ-1 — 2K×18
cascades into two BSRAM cells and only the primary gets init data.

Button **B** (pin 44, nRST) is wired into the SoC's reset chain. Pressing
it drives `resetn` low and the F18A returns to `RESET_PC = 0x7F9` —
identical to power-on, so the next streamed program runs without a
re-flash.

## Build & flash

```sh
make f18a.fs                       # default: 27 MHz, 75 % LUT4 / 3 BSRAM
make f18a.fs PLL_FREQ=54           # rPLL build (synthesises but see NEXT.md)
make flash-sram                    # volatile SRAM load (lost on power cycle)
make flash                         # SPI flash (persists across power cycles)
make test                          # full regression
```

The default build packs the F18A core, 1 K main BSRAM, 64-deep dstk
BSRAM, 128-deep rstk BSRAM, and the SoC plumbing (UART, INST_PORT, LED,
button) at 75 % LUT4, 105 MHz fmax post-route. dstk/rstk live in BSRAM
rather than distributed LUT RAM (one block each), so deep recursive
aforth — including A(3,3), peak rsp = 121 — runs end-to-end on chip.

UART runs at 115200 baud on `/dev/ttyUSB0`. The Tang Nano 1K's BL702 USB
bridge **isn't wired to the FPGA**, so you'll need an external FTDI
adapter: FTDI TX → FPGA pin 27, FTDI RX ← FPGA pin 28, GND tied.

## Tests

```sh
$ bash run_tests.sh
```

Covers:

- `tb_ops.v` — 56-case opcode regression
- `tb_extarith.v` — carry / `+*` edge cases
- `tb_muls.v` — native MULS through the BSRAM path
- `run_fullisa_cases.py` — Array-Forth opcode case images
- `run_semantic_cases.py` — 23 per-instruction semantic checks
- `tb_ack` — iterative Ackermann (smoke-tests the streaming-port path
  in sim)
- `test_forth_compile.py` — Forth compiler regression
  (emit / arith / colons / dup-drop / if-then / swap, plus Ackermann
  through A(3, 1))

## Forth REPL (host-compiled)

```sh
$ python3 forth_repl.py            # sim by default
forth-on-fpga (Ctrl-D to exit; `.words` lists definitions)
> : hi 72 emit 105 emit 33 emit 10 emit ;
ok.
> hi
Hi!
ok.
> 60 5 + emit 10 emit
A
ok.

$ python3 forth_repl.py --hw       # talk to /dev/ttyUSB0
```

Each non-definition line is recompiled from scratch (including all `:`
definitions accumulated so far in the session), passed through the F18A
assembler, and either fed into a sim BSRAM via `$readmemh` or streamed
straight at INST_PORT over UART. Captured `UART_TX` bytes are echoed
back to the host. There is **no on-target dictionary**: every line is a
fresh image compiled on the host and shipped to RAM.

### Forth subset

- Decimal / hex (`0x…`) literals, signed 18-bit
- `+`, `-`, `and`, `or`, `xor`, `inv`, `2*`, `2/`
- `dup`, `drop`, `over`, `swap`, `nip`
- `>R` / `R>` (return-stack stash)
- `1+`, `1-`, `negate`
- `0=`, `if … else … then`
- `:` *name* `body` `;`
- `emit` (write low byte of T to UART_TX, with TX-room polling)
- `@`, `!`, `@b`, `!b`, `a`, `a!`, `b!` (raw F18A memory & pointer ops)

It also accepts a few **raw F18A words directly** when you want to drop
closer to the metal: `+*`, `@p`, `!p`, `@+`, `!+`, and `ex`.

## ga-tools aforth (third-party front-end)

Optional: drive the softcore from
[mschuldt/ga-tools](https://github.com/mschuldt/ga-tools)' aforth
compiler. Gives you GA144-flavoured Forth with `: ... ;`, structured
`if/then`, `for/next`, `begin/until`, label-driven calls, and the
real-F18A 8-op slot 3 (`; unext @p !p +* + dup .`).

```sh
pip install --user --break-system-packages ga-tools
make flash-sram
python3 ga_aforth.py --hw aforth_demo.ga    # → b'01234\n'
python3 ga_aforth.py --hw aforth_ack.ga     # → b'\x07'  (A(2,2) = 7)
python3 ga_aforth.py --hw aforth_hue.ga --fire-and-forget    # RGB hue cycle
python3 ga_aforth.py --hw aforth_buttons.ga --fire-and-forget # mirror BUTTON to LED
```

`ga_aforth.py` is the bridge: it lets ga-tools parse + slot-pack +
resolve labels, walks the resulting `Word` list, and re-emits one
`.f18a` mnemonic line per word so this repo's `asm.py` re-packs it
with the bit layout the softcore decodes (the two assemblers use
different slot encodings — ga-tools targets the real F18A's XOR
encoding, we use plain bits). A `jump:0x7F9` epilogue is appended so
each program returns to INST_PORT after its main word, matching how
`ackermann.f18a` behaves.

`bench_ack.py` is a cycle-count comparison between
`ackermann.f18a` (iterative, hand-asm) and `aforth_ack.ga`
(recursive, via ga-tools). Recursive aforth is consistently
1.7×–2.3× faster, at the cost of much deeper hardware stacks.

## Streaming-port protocol (boot loader)

Each instruction word is sent as 3 little-endian bytes:
`[w&0xFF, (w>>8)&0xFF, (w>>16)&0x03]`. After power-on / soft-reset, the
F18A's first fetch is at INST_PORT — so the host can stream in any
sequence of standalone instruction words and have them executed one at
a time. The standard self-loader is just three packed words:

```
@p a! . .   [target_addr]    # set A to first cell of the loader's payload
@p !+ . .   [data_word]      # store a word, A++   ← repeated N times
jump:0      .  .  .          # leave port-execution, run from RAM
```

`ack_host.py` is the reference example: it bundles loader + program +
input data into one UART blob, reads back the result, and prints it.
`port_exec_demo.py` is a 12-byte "emit X over UART" program that
doesn't even use a loader — it just streams two instruction words
straight at the port and watches for the `'X'`. Useful for sanity-
checking the streaming path on a fresh bitstream.

## Assembler

`asm.py` packs 4-slot F18A instruction words. Two directives let you
define addresses by name:

```f18a
#define PORT     0x7F9       # named constant
#define UART_TX  0x7F1
#define MINUS1   0x3FFFF

#variable ctr_cell           # auto-allocated past last code cell
#variable n_addr
#variable mstack_base
```

`#variable` cells are placed at the next free word past the last code
cell (in declaration order). They never collide with the program, no
matter how the program grows or shrinks. `#define` constants can be used
anywhere a literal would go — `[PORT]`, `jump:PORT`, `[-mstack_base]`
all work. The listing emitted by `python3 asm.py -hex …` surfaces the
allocated variable addresses so the layout stays auditable:

```
; var ctr_cell = 0x040
; var n_addr   = 0x041
; var mstack_base = 0x042
```

Slot-3 NOPs are written `.` (the assembler rejects anything else that
isn't `;`, `ex`, or `unext`).

## F18A control-flow notes (so you don't get bitten)

The 4-slot packing model is unforgiving in two ways the assembler can't
fully shield you from:

- **Slot 3 only allows `;`, `ex`, `unext`, or one of `@p !p +* + dup .`.**
  A *missing* slot 3 still defaults to an implicit `;` — fine at top
  level (rsp=0 turns ret into "fetch next word") but fatal anywhere `R`
  is non-zero (the implicit ret pops `R` into `P` and you jump wherever
  `R` happens to point). Inside `for ... next` / `for ... unext` and
  inside any called subroutine, every word that isn't the final RET
  needs an explicit `.` (slot-3 NOP).
- **Branches reach by slot.** Slot-0 branches use the full 13-bit
  address (masked to `ADDR_BITS`). Slot-1 branches are page-local
  (256 words). Slot-2 branches are block-local (8 words).

`ackermann.f18a` and `fft_echo.f18a` show idiomatic ways to lay code
out around these rules.

## What's next

See [NEXT.md](NEXT.md) for the queue: 54 MHz PLL build, dual-core
hyperthreaded SoC (sim-verified, doesn't fit GW1NZ-1), asymmetric
stacks for deeper recursion, rstk-spill-to-BSRAM for A(3,3) recursive.

## References

- [GreenArrays DB001](https://www.greenarraychips.com/home/documents/greg/DB001-221113-F18A.pdf)
  — the F18A reference manual
- [mschuldt/ga-tools](https://github.com/mschuldt/ga-tools) — aforth
  compiler used by `ga_aforth.py`
- [Sipeed Tang Nano 1K wiki](https://wiki.sipeed.com/hardware/en/tang/Tang-Nano-1K/Nano-1K.html)
- [oss-cad-suite](https://github.com/YosysHQ/oss-cad-suite-build) —
  yosys, nextpnr-himbaechel, apycula, iverilog

## License

MIT. F18A and arrayForth are trademarks of GreenArrays, Inc.; this is
an independent reimplementation for educational and research purposes,
not endorsed by or affiliated with GreenArrays.
