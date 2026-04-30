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

### GA144-style cooperative multitasking (Phase 1 done, 2–4 in flight)

Goal: replicate GA144 parallelism on the single F18A core. Multiple
Forth tasks share the chip; each blocks transparently when it hits a
memory-mapped neighbour port that isn't ready (UART_TX full, UART_RX
empty, INST_PORT not assembled), and the scheduler resumes whichever
blocked task has its port ready next, in FIFO order.

Architecture (4 task contexts on GW1NZ-1):

* **One active register set** stays in flip-flops (T, S, R, A, B, P,
  carry, dsp, rsp, mem_addr, mem_wdata, mem_we, st, slot — same as
  today). The other 3 task contexts sit in a small BSRAM-backed
  context table (4 tasks × 9 packed cells each = 36 cells, fits in
  the spare half of an existing stack BSRAM). RTOS-style save/restore
  rather than per-task FF banks — the FF cost of 4-way FF
  duplication (~648 FFs added) blows the 864-DFF budget; BSRAM-backed
  contexts add only ~80 FFs.
* **Per-task stack BSRAM slicing.** Each task gets its own
  DSTK_DEPTH-deep dstk slice and RSTK_DEPTH-deep rstk slice;
  `dstk_addr = {running_task, dsp_local}` (resp. rstk). For
  NTASKS=4 + DSTK_DEPTH=64 the array is 256 cells in one BSRAM (50 %
  used); RSTK_DEPTH=128 fills one BSRAM (4 × 128 = 512 cells).
* **Switch trigger:** `mem_ready=0` in any of `ST_FWAIT` /
  `ST_MWAIT` / `ST_MEMWR` *and* another task is runnable. Save the
  full active state to `ctx_ram[running_task * 9 + idx]` (9 cycles),
  pick the next runnable task off the FIFO, load its context (10
  cycles, BSRAM 1-cycle latency), resume in whatever state the
  reloaded `st` says. Switch overhead ≈ 19 cycles, fine because the
  port-blocked task would have been stalled waiting anyway.
* **Per-task block tracking.** When task k blocks, set
  `task_blocked[k]=1`, `task_block_addr[k]=mem_addr`. Scheduler
  picks the first task whose block-port is ready: UART_RX → rx_avail,
  UART_TX → !mbox_full, INST_PORT → rx_inst_count==3, anything else
  → always ready. SoC exposes the per-port-ready bits as new core
  inputs; core internally maps `block_addr` → which port-ready bit
  to consult.
* **Spawn.** New SoC register `TASK_CTRL` at 0x7F7. Writing
  `{task_id[1:0], pc[10:0]}` updates `ctx_ram[task_id, P_idx]` and
  clears `task_blocked[task_id]`. At reset only task 0 is runnable
  (`task_blocked = 4'b1110`); other tasks need explicit spawning.

Phase 1 — **per-task stack BSRAM slicing** (in this branch).
`running_task` register added, `dstk[]`/`rstk[]` widened to NTASKS ×
depth, addresses prepended with `running_task`. `running_task` is
hard-pinned to 0 until phases 2–4 land, so all single-task tests pass
unchanged. Resource cost: +1 percentage point LUT4 (76 → 77 %), same
3 BSRAM blocks (the larger stack arrays still fit in one block each
at 18-bit width). Verified on real hardware: A(3,3) iterative
146,219 hw cycles vs 146,271 sim — unchanged. Foundation in place
for phases 2–4 to add the scheduler without further BSRAM growth.

Phase 2 — **context save/restore + manual yield** (in this branch).
Working in iverilog sim: `programs/mt_abab.f18a` on `rtl/tb_multitask.v`
emits a clean `ABABABAB…` round-robin sequence at 70 cycles/character
(both tasks pre-spawned at reset; each emits its char and writes
`TASK_CTRL` (0x7F7) to yield). Switch overhead ≈ 14 cycles
(6-cell save + 1-cycle bridge + 7-cycle load). The trimmed context
saves T, S, R, A, P, dsp, rsp, carry only — B is not saved (each
task's loop sets B fresh from `@p b!` before each emit). Mid-FSM
state (mem_addr, mem_we, mem_wdata, slot, saved_st) isn't saved
either; switches always trigger at ST_FETCH and resume at ST_FETCH.
NTASKS=2 in this build to keep the save mux narrow.

**Hardware fit on GW1NZ-1: still tight.** The latest design moves
the per-task save/restore through the existing main-RAM port:
contexts live in `c1_mem` at `CTX_BASE..CTX_BASE+NTASKS*16-1`
(default `0x3E0..0x3FF` for NTASKS=2 — top 32 cells of the 1 K
RAM, reserved by the core, off-limits to programs). Walking those
cells through `mem_addr`/`mem_we`/`mem_wdata` instead of a
dedicated `ctx_ram` BSRAM drops one BSRAM (3/4 used, vs 4/4 with
the dedicated array) and trims the duplicated `ctx_*` mux logic.
A new `ST_INIT` boot state seeds task 1's P cell with `TASK1_PC`
once at reset.

Resource budget after the refactor: **1 119 LUT4 / 1 152 (97 %)**,
3/4 BSRAM, 350/864 DFF. Synthesis succeeds but `nextpnr-himbaechel`
placement still fails — "Unable to find legal placement for all
cells, design is probably at utilisation limit". The 5-way save
mux + 6-way load demux are intrinsically wide (each register has
multiple input sources because of the existing op decode), and the
chip's 4 BSRAMs are in fixed positions, leaving routing congested
around them. Sim is fully working; further fitting would need to
either drop more saved registers (e.g. share dstk/rstk pointers
across tasks — abandoning per-task stack state) or use a bigger
part (Tang Nano 9K with a GW1NR-9 has ~6× the LUT4).

Phase 3 — auto-switch on `mem_ready=0` + per-task block-port
tracking. Restore the FSM-state save (mem_addr, mem_we, mem_wdata,
slot, saved_st) so the resumed task picks up mid-instruction. Add
SoC `uart_rx_ready` / `uart_tx_ready` / `inst_port_ready` outputs
into the core for the unblock check.

Phase 4 — FIFO scheduler + SoC TASK_CTRL spawn-inject + multi-task
demos (4-task with port-blocked tasks waking on UART RX, etc.).

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
