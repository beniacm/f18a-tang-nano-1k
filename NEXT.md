# Next steps

## Done

- **Slot-3 encoding matches DB001 §2.3.1.** Eight ops valid in slot 3
  (`; unext @p !p +* + dup .`); slot-3 bits store `op[4:2]`, the core
  re-extends with two zero LSBs at decode time. Lifted the for/next /
  if-then restriction in `ga_aforth.py`.
- **ga-tools bridge.** `ga_aforth.py` accepts aforth source, hands it
  to ga-tools for parsing + slot packing + label resolution, then
  re-emits one `.f18a` mnemonic line per word so `asm.py` can pack it
  with our bit layout. `aforth_demo.ga` (for/next loop emitting
  "01234\n") and `aforth_ack.ga` (recursive Ackermann) both run on
  hardware.
- **Performance baseline.** `bench_ack.py` compares iterative
  `ackermann.f18a` vs recursive `aforth_ack.ga`. Recursive aforth is
  1.7×–2.3× faster in cycles (BSRAM-stack ops cost more than in-core
  rstk recursion), at the cost of much deeper hardware stacks
  (peak rsp = 121 for A(3,3) vs 0 for native iterative).

## Open / queued

### 54 MHz on hardware
`make f18a.fs PLL_FREQ=54` synthesises clean (106 MHz fmax post-route,
2× margin) but the bitstream doesn't tick on the FPGA — `ack_host`
gets no UART output. Apycula 0.33 packs the rPLL primitive (1/1) and
reports `pll_inst.clkin net was routed using global resources only`,
yet the design is dead. Same symptom as the original rPLL retirement
(commit `bcb5958`). Open question: is apycula's GW1NZ-1 rPLL bitstream
support missing a CLKDIV/CLKBUF wiring on the PLL output? Until
someone digs into the bitstream diff, run at the default `PLL_FREQ=27`.

### Dual-core hyperthreaded SoC
Implemented and sim-verified on the `dual-core` branch (last commit
`c23a901`). Two `f18a_core` instances on shared BSRAM, alternating
slots via a free-running `tick`, two cross-core mailbox FIFOs. Doesn't
fit GW1NZ-1: 1316/1152 LUT4 (114 %) at NO_BIG, 1093/1152 (94 %) at
MINIMAL — even MINIMAL fails legal placement. A larger Gowin part
(GW1NR-9 on Tang Nano 9K) would likely fit it. The sim path is
exercised by `tb_dual.v` + `dual_ping.f18a` if you check out that
branch.

### Asymmetric stacks for deeper recursion (measured)
Native iterative ackermann fits any depth at peak dsp = 5; recursive
versions need much more rstk. Tried several depths on hardware:

| Build                              | LUT4   | A(3,1) | A(3,2) | A(2,7) | A(3,3) |
|------------------------------------|--------|:------:|:------:|:------:|:------:|
| `f18a.fs` (default 16/32)          | 77 %   |  ✓     |  ✗     |  ✗     |  ✗     |
| `f18a.fs RSTK_DEPTH=64`            | 87 %   |  ✓     |  ✓     |  ✓     |  ✗     |
| `f18a.fs RSTK_DEPTH=128`           | n/a    |  -     |  -     |  -     |  -     |

`RSTK_DEPTH=128` overflows the LUT4 budget — rstk infers as LUTRAM
once it can't fit BSRAM (apycula already gives the 1 BSRAM to
`c1_mem`), and 128 × 18 bits is too many LUT4 cells.

`RSTK_DEPTH=64` is the sweet spot — fits at 87 %, runs A(3,2) and
A(2,7) recursive aforth/Forth on hardware. A(3,3) still doesn't fit
(peak rsp = 121 in sim).

Open path for A(3,3) recursive on this chip: spill rstk to BSRAM
when it overflows in-core capacity. The core would push to memory at
some agreed scratch range (e.g. 0x300..0x37F) on rstk overflow and
pop back transparently. Costs: a few LUT4 for the overflow logic
plus per-spill cycles.
