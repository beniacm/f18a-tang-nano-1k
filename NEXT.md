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
- **P9 enhances shifts too.** P9 was previously only an arithmetic-
  carry-chain switch (turns `+` and `+*` into add-with-carry). Now
  also turns `SHL`/`SHR` into rotate-through-carry: code running at
  P9 (page 0x200+) sees the bit dropping off the end of the shift
  land in the carry latch, and the bit shifting in on the other end
  taken from carry. Pairs with the existing P9 add-with-carry to
  give multi-precision shifts (e.g. a 36-bit `<<1` becomes two
  P9-mode SHLs across a lo/hi cell pair). Non-P9 shifts are
  unchanged: `SHL` drops MSB and shifts in 0; `SHR` is arithmetic
  (sign-extends MSB). Cost: +13 LUT4. Verified by `tb_extarith.v`.
- **dstk + rstk in BSRAM.** Both stacks moved out of distributed LUT
  RAM into block RAM (one BSRAM each). Reads use a one-cycle-ahead
  combinational `next_dsp` / `next_rsp` lookahead so the registered
  read addr arrives at the BSRAM port one edge before it's needed;
  push collisions are handled by an explicit write-through forwarding
  mux in the same always block. New defaults: `DSTK_DEPTH = 64`,
  `RSTK_DEPTH = 128` (fits A(3,3) recursive — peak rsp = 121, runs
  end-to-end on hardware via `ga_aforth.py --hw aforth_ack.ga`).
  73 % LUT4 / 75 % BSRAM (3/4 blocks) at 92 MHz post-route — down from
  80 % LUT4 with the old distributed-RAM stacks.

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

### Stack depth on hardware (post-BSRAM-move)

With dstk and rstk in BSRAM, depth no longer competes with the LUT4
budget — each block holds 256+ entries, so we just pick a deep
default and forget about it. Confirmed on hardware:

| Build                              | LUT4 | BSRAM | A(3,1) | A(3,2) | A(2,7) | A(3,3) |
|------------------------------------|------|-------|:------:|:------:|:------:|:------:|
| `f18a.fs` (default 64/128)         | 75 % | 3/4   |  ✓     |  ✓     |  ✓     |  ✓     |

A(3,3) recursive runs end-to-end via `ga_aforth.py --hw aforth_ack.ga`
(peak rsp = 121, fits in the 128-deep BSRAM rstk).
