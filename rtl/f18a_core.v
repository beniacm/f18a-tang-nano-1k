// F18A ISA softcore (synchronous; simplified instruction packing — no XOR
// mask on slots).
//
// Word  : 18 bits
// Slot 0: bits [17:13]  (5-bit opcode)
// Slot 1: bits [12:8]   (5-bit opcode)
// Slot 2: bits [7:3]    (5-bit opcode)
// Slot 3: bits [2:0]    (3 bits; high 2 bits of opcode implicit 00)
//
// Jump/call/if/-if/next consume the remainder of the current word as the
// target address. Address width per slot:
//   slot 0: 13 bits  (masked to ADDR_BITS)
//   slot 1:  8 bits  (high bits of P preserved)
//   slot 2:  3 bits  (high bits of P preserved)
//   slot 3: invalid (no room for address)
//
// Memory interface: single-ported, synchronous-read friendly.
// mem_addr is a REGISTERED request presented to the RAM input; fetches
// issue that address in ST_FETCH, then ST_FWAIT/ST_FWAIT2 absorb the
// extra cycle before I samples mem_rdata. Data reads do the same via
// ST_MWAIT/ST_MEMRD. Registering mem_addr removes the placement-sensitive
// combinational path into GW1NZ-1 BSRAM that could glitch on some seeds.
//
// Stacks: dstk[] and rstk[] are held in BSRAM rather than LUT4-based
// distributed RAM. T, S, R are still registers; the stacks proper hold
// what's "below S" and "below R". Each cycle we issue a synchronous
// read at next_dsp-1 (resp. next_rsp-1), where next_dsp/next_rsp is the
// pointer's value AFTER the current op fires. The result lands one
// edge later, so on the cycle after a pop, dstk_rdata is already the
// new top-of-below. Push collisions (a write to addr X immediately
// followed by a read at X) are handled by an explicit write-through
// forwarding mux in the BSRAM block — push semantics need the just-
// written value visible to the next cycle's pop.
//
// Build flags (cumulative — MINIMAL implies NO_BIG):
//   `define NO_BIG    strip the five most expensive ops that don't pay for
//                     themselves on small hw: CALL, OVER, POP, PUSH, MULS.
//                     Enough to fit the monitor + core on GW1NZ-1 while
//                     keeping the bulk of the F18A ISA.
//   `define MINIMAL   also strip EX, UNEX, NEXT, MIF, SHL, SHR. The
//                     resulting core only does linear data-pump style
//                     programs (UART echo, mailbox relay).
//   default           full 32-op ISA — for simulation + unit tests.

module f18a_core #(
    parameter ADDR_BITS  = 9,
    parameter DSTK_DEPTH = 16,           // 16 by default; grow for deep recursion / stack-heavy programs
    parameter DSP_BITS   = 4,            // ceil(log2(DSTK_DEPTH))
    parameter RSTK_DEPTH = 16,           // return stack depth; 16 at synth was the GA144 default
    parameter RSP_BITS   = 4,            // ceil(log2(RSTK_DEPTH))
    parameter [ADDR_BITS-1:0] RESET_PC = 0,
    parameter [ADDR_BITS-1:0] MULS_HANDLER_ADDR = 'h3E0,  // only used when MULS_TRAP is defined
    // Address of the GA144-style "comm port" used for port execution
    // (DB001 §3.3.2). When P (or @p's address) equals PORT_ADDR, P is
    // NOT incremented after the read — the core re-fetches from the
    // port until control transfers elsewhere, letting an external host
    // stream an instruction stream straight in. Default is the highest
    // address in the address space, which the f18a_soc memory map does
    // not assign to a real register, so legacy builds see no behavior
    // change unless they wire a real port at this address.
    parameter [ADDR_BITS-1:0] PORT_ADDR = {ADDR_BITS{1'b1}},
    // Reset value for B (DB001 §3.1: "B is set to the address of io").
    // For the f18a_soc SoC we set it to UART_TX so the core wakes up
    // able to !b out of the box. Default 0 preserves legacy test
    // expectations.
    parameter [ADDR_BITS-1:0] RESET_B  = {ADDR_BITS{1'b0}}
) (
    input  wire                  clk,
    input  wire                  resetn,

    // Registered memory request presented to the external RAM / IO mux.
    output reg  [ADDR_BITS-1:0]  mem_addr,
    output reg                   mem_we,
    output reg  [17:0]           mem_wdata,
    input  wire [17:0]           mem_rdata,

    // Handshake from the memory/IO side. mem_ready=0 stalls the core in
    // its current memory state — used by the SoC to turn UART RX/TX (and
    // any other GA144-style "neighbor port") into blocking ports: a read
    // hangs in ST_MWAIT until data arrives, a write hangs in ST_MEMWR
    // until the previous byte has been accepted. Tying mem_ready high
    // recovers the legacy single-cycle handshake (see TB stubs).
    input  wire                  mem_ready,

    output wire [17:0]           dbg_T,
    output wire [17:0]           dbg_I,
    output wire [ADDR_BITS-1:0]  dbg_P,
    output wire [1:0]            dbg_slot
);

`ifdef MINIMAL
    `define NO_CALL
    `define NO_EX
    `define NO_UNEX
    `define NO_NEXT
    `define NO_MIF
    `define NO_SHL
    `define NO_SHR
    `define NO_OVER
    `define NO_POP
    `define NO_PUSH
    `define NO_MULS
`endif
`ifdef NO_BIG
    `define NO_CALL
    `define NO_OVER
    `define NO_POP
    `define NO_PUSH
    `define NO_MULS
`endif
// ACK build: keep CALL + PUSH + POP (swap via rstk) for the Ackermann
// program. Drop OVER + MULS to stay in the LUT budget.
`ifdef ACK
    `define NO_OVER
    `define NO_MULS
`endif
// RECURSIVE build: BSRAM-backed c1_ram frees enough LUTs that we can
// keep CALL/RET (and OVER), only dropping POP+PUSH+MULS. 29 of 32 ops
// available — recursion works on hardware. Synth at ~89 % LUT4.
`ifdef RECURSIVE
    `define NO_POP
    `define NO_PUSH
    `define NO_MULS
`endif
// FULLISA build: all architected user opcodes available in silicon —
// native +* and the P9 carry latch. The 54 MHz LITE variant was retired
// (see git log around f32f2e5) — it didn't reliably make timing on
// GW1NZ-1, and we now boot from the streamed-instruction port, so the
// trap-based `_muls` shim is no longer needed for the default build.

    localparam [4:0]
        OP_RET  = 5'o00, OP_EX   = 5'o01, OP_JMP  = 5'o02, OP_CALL = 5'o03,
        OP_UNEX = 5'o04, OP_NEXT = 5'o05, OP_IF   = 5'o06, OP_MIF  = 5'o07,
        OP_FETP = 5'o10, OP_FETA = 5'o11, OP_FETB = 5'o12, OP_FET  = 5'o13,
        OP_STP  = 5'o14, OP_STA  = 5'o15, OP_STB  = 5'o16, OP_ST   = 5'o17,
        OP_MULS = 5'o20, OP_SHL  = 5'o21, OP_SHR  = 5'o22, OP_INV  = 5'o23,
        OP_ADD  = 5'o24, OP_AND  = 5'o25, OP_XOR  = 5'o26, OP_DROP = 5'o27,
        OP_DUP  = 5'o30, OP_POP  = 5'o31, OP_OVER = 5'o32, OP_A    = 5'o33,
        OP_NOP  = 5'o34, OP_PUSH = 5'o35, OP_BSTO = 5'o36, OP_ASTO = 5'o37;

    localparam [2:0]
        ST_FETCH  = 3'd0,
        ST_FWAIT  = 3'd1,
        ST_FWAIT2 = 3'd2,
        ST_EXEC   = 3'd3,
        ST_MWAIT  = 3'd4,
        ST_MEMRD  = 3'd5,
        ST_MEMWR  = 3'd6;

    reg [2:0] st;
    reg [1:0] slot;

    // T, S, R, I, A are explicitly NOT cleared by the synchronous reset
    // (per F18A spec: "Other registers, stack contents, and RAM are
    // not directly affected by reset" — they keep their prior values
    // across `H` halts). We give them an initial value at declaration
    // so power-up state is deterministic in both sim and on the FPGA.
    reg [17:0] T = 18'd0;
    reg [17:0] S = 18'd0;
    reg [17:0] R = 18'd0;
    reg [17:0] I = 18'd0;
    reg [ADDR_BITS-1:0] P;
`ifndef NO_P9_ARITH
    // F18A extended arithmetic is keyed off P9 and a sticky carry
    // latch. We model the latch explicitly and reset it to 0 — power-up
    // / halt-then-resume behavior stays deterministic.
    reg carry;
`endif
    reg [17:0] A = 18'd0;
    reg [17:0] B;     // B is reset by the synchronous block (DB001 §3.1).

    // Stacks live in BSRAM. The block-RAM pragma steers yosys away from
    // distributed-LUT inference; pre-fetched reads + write-through
    // forwarding hide the synchronous-read latency.
    (* ram_style = "block" *) reg [17:0] dstk [0:DSTK_DEPTH-1];
    (* ram_style = "block" *) reg [17:0] rstk [0:RSTK_DEPTH-1];
    reg [17:0] dstk_rdata;
    reg [17:0] rstk_rdata;
    reg [DSP_BITS-1:0] dsp;
    reg [RSP_BITS-1:0] rsp;

    localparam [DSP_BITS-1:0] DSP_ZERO = {DSP_BITS{1'b0}};
    localparam [DSP_BITS-1:0] DSP_ONE  = {{(DSP_BITS-1){1'b0}}, 1'b1};
    localparam [RSP_BITS-1:0] RSP_ZERO = {RSP_BITS{1'b0}};
    localparam [RSP_BITS-1:0] RSP_ONE  = {{(RSP_BITS-1){1'b0}}, 1'b1};

    // Slot decode. Slot 3 stores only 3 bits — the high 3 bits of a
    // 5-bit op (low 2 bits are zero). That gives 8 valid slot-3 ops,
    // every fourth op in the table (matching real F18A's `last_slot
    // _ops` set: ; unext @p !p +* + dup .). The asm.py side encodes
    // slot 3 as (op >> 2) & 0b111.
    wire [4:0] op =
        (slot == 2'd0) ? I[17:13] :
        (slot == 2'd1) ? I[12:8]  :
        (slot == 2'd2) ? I[7:3]   :
                         {I[2:0], 2'b00};

    wire [ADDR_BITS-1:0] jaddr =
        (slot == 2'd0) ? I[ADDR_BITS-1:0] :
        (slot == 2'd1) ? {P[ADDR_BITS-1:8], I[7:0]} :
        (slot == 2'd2) ? {P[ADDR_BITS-1:3], I[2:0]} :
                         P;

    // ALU combinational. In this 1K flat-memory model, "P9" currently means
    // the visible high address bit of P, so code running at 0x200..0x3ff sees
    // extended arithmetic enabled.
    //
    // With native +* and P9 enabled, a single 19-bit add covers both the
    // plain `+` and the P9-extended `+`/`+*` case. The carry input is
    // gated by arith_ext, so the low 18 bits match a plain T+S when
    // arith_ext=0 — collapsing what used to be two parallel adders + an
    // arith_ext-controlled output mux down to one chain. ~14 LUTs and
    // 20 ALU primitives saved; FMAX still ~91 MHz post-route.
    //
    // The LITE build (NO_P9_ARITH) keeps the plain 18-bit alu_add chain
    // — there's no carry latch and the unified path would just be a
    // 19-bit adder with a hard-tied 0 carry-in, which yosys doesn't
    // always fold cleanly on this Gowin mapping.
`ifdef NO_P9_ARITH
    wire [17:0] alu_add = T + S;
`else
    // The "P9" carry-extension flag is bit 9 of P. For ADDR_BITS<10 P
    // doesn't have a bit 9, so we zero-extend; for ADDR_BITS>=10 we just
    // index. The localparam keeps the {N{}} repeat strictly non-negative.
    localparam P_PAD = (ADDR_BITS < 10) ? (10 - ADDR_BITS) : 0;
    wire [9:0]  P_ext = (ADDR_BITS >= 10) ? P[9:0]
                                          : {{P_PAD{1'b0}}, P[ADDR_BITS-1:0]};
    wire        arith_ext = P_ext[9];
    wire        carry_in = carry & arith_ext;
    wire [18:0] alu_add_ext = {1'b0, S} + {1'b0, T} + {{18{1'b0}}, carry_in};
    wire [17:0] alu_add = alu_add_ext[17:0];
`endif
    wire [17:0] alu_and = T & S;
    wire [17:0] alu_xor = T ^ S;
    wire [17:0] alu_inv = ~T;
    wire [17:0] alu_shl = {T[16:0], 1'b0};
    wire [17:0] alu_shr = {T[17], T[17:1]};

`ifndef MULS_TRAP
    // Multiply-step (+*) precomputation.
    //   If A[0]: pre = T + S (+ carry when arith_ext)  else pre = T
    //   Then shift {pre, A} right by 1: new T = {0, pre[17:1]},
    //   new A = {pre[0], A[17:1]}.
    // This implements one bit of an unsigned 18×18 multiply; stack values
    // must be arranged and `unext` counted to build the full product.
    wire [17:0] mul_pre  = A[0] ? alu_add : T;
    wire [17:0] mul_newT = {1'b0, mul_pre[17:1]};
    wire [17:0] mul_newA = {mul_pre[0], A[17:1]};
`endif

    // Shared loop-control helpers. Keeping these as named wires encourages
    // the synthesiser to share the compare/subtract across NEXT and UNEX.
    wire        dsp_empty  = (dsp == DSP_ZERO);
    wire        rsp_empty  = (rsp == RSP_ZERO);
    wire [DSP_BITS-1:0] dsp_prev = dsp - DSP_ONE;
    wire [DSP_BITS-1:0] dsp_next = dsp + DSP_ONE;
    wire [RSP_BITS-1:0] rsp_prev = rsp - RSP_ONE;
    wire [RSP_BITS-1:0] rsp_next = rsp + RSP_ONE;
    wire        R_is_zero = (R == 18'd0);
    wire [17:0] R_dec     = R - 18'd1;
    wire [ADDR_BITS-1:0] current_word = P - 1'b1;

    // ── Stack push/pop classification ──────────────────────────────
    // Combinationally decode whether the current op (in ST_EXEC, or the
    // implicit push at ST_MEMRD) modifies dsp/rsp, and whether it writes
    // to the stack BSRAMs. The classification feeds two things:
    //   * next_dsp/next_rsp — used as the BSRAM read address one cycle
    //     ahead, so by the time the op completes, dstk_rdata holds the
    //     value at the new dsp-1 (= "S below" after the op).
    //   * dstk_we / rstk_we — fired in the same cycle as the op, so the
    //     write commits at the same clock edge that updates T/S/R/dsp.
    // Falling through to dsp_op_push=0/dsp_op_pop=0 leaves the stack
    // untouched and dstk_rdata stable (steady-state read at dsp-1).
    reg dsp_op_push;
    reg dsp_op_pop;
    reg rsp_op_push;
    reg rsp_op_pop;
    reg dstk_we;
    reg rstk_we;

    always @* begin
        dsp_op_push = 1'b0;
        dsp_op_pop  = 1'b0;
        rsp_op_push = 1'b0;
        rsp_op_pop  = 1'b0;
        dstk_we     = 1'b0;
        rstk_we     = 1'b0;
        if (st == ST_EXEC) begin
            case (op)
            OP_RET: rsp_op_pop = !rsp_empty;
`ifndef NO_CALL
            OP_CALL: begin rsp_op_push = 1'b1; rstk_we = 1'b1; end
`endif
`ifndef NO_UNEX
            OP_UNEX: if (R_is_zero && !rsp_empty) rsp_op_pop = 1'b1;
`endif
`ifndef NO_NEXT
            OP_NEXT: if (R_is_zero && !rsp_empty) rsp_op_pop = 1'b1;
`endif
            OP_DROP, OP_ADD, OP_AND, OP_XOR,
            OP_ASTO, OP_BSTO,
            OP_STA, OP_STB, OP_ST, OP_STP: dsp_op_pop = 1'b1;
            OP_DUP, OP_A: begin dsp_op_push = 1'b1; dstk_we = 1'b1; end
`ifndef NO_OVER
            OP_OVER: begin dsp_op_push = 1'b1; dstk_we = 1'b1; end
`endif
`ifndef NO_POP
            OP_POP:  begin dsp_op_push = 1'b1; rsp_op_pop = 1'b1; dstk_we = 1'b1; end
`endif
`ifndef NO_PUSH
            OP_PUSH: begin dsp_op_pop = 1'b1; rsp_op_push = 1'b1; rstk_we = 1'b1; end
`endif
`ifndef NO_MULS
  `ifdef MULS_TRAP
            OP_MULS: begin rsp_op_push = 1'b1; rstk_we = 1'b1; end
  `endif
`endif
            default: ;
            endcase
        end
        if (st == ST_MEMRD) begin
            dsp_op_push = 1'b1;
            dstk_we     = 1'b1;
        end
    end

    wire [DSP_BITS-1:0] next_dsp = dsp_op_push ? dsp_next :
                                   dsp_op_pop  ? dsp_prev : dsp;
    wire [RSP_BITS-1:0] next_rsp = rsp_op_push ? rsp_next :
                                   rsp_op_pop  ? rsp_prev : rsp;

    // BSRAM read addr is "where dsp-1 will be after this op". When the
    // next state is ST_FETCH (slot 3 done, mem op done, etc.) dsp may
    // not change for several cycles — the read just keeps re-reading
    // the same addr, which is harmless and keeps dstk_rdata stable.
    wire [DSP_BITS-1:0] dstk_raddr = next_dsp - DSP_ONE;
    wire [RSP_BITS-1:0] rstk_raddr = next_rsp - RSP_ONE;

    // dstk push wdata is always S (the value getting demoted). rstk push
    // wdata is always R. waddr is the current pointer (next-free slot).
    wire [DSP_BITS-1:0] dstk_waddr = dsp;
    wire [RSP_BITS-1:0] rstk_waddr = rsp;
    wire [17:0]         dstk_wdata = S;
    wire [17:0]         rstk_wdata = R;

    // BSRAM blocks. Synchronous read with explicit write-through
    // forwarding: when waddr == raddr in the same cycle (push followed
    // by next-cycle pop reading what we just wrote), rdata sees the
    // new value. Without this the pop would observe stale data.
    always @(posedge clk) begin
        if (dstk_we) dstk[dstk_waddr] <= dstk_wdata;
        if (dstk_we && dstk_raddr == dstk_waddr)
            dstk_rdata <= dstk_wdata;
        else
            dstk_rdata <= dstk[dstk_raddr];
    end

    always @(posedge clk) begin
        if (rstk_we) rstk[rstk_waddr] <= rstk_wdata;
        if (rstk_we && rstk_raddr == rstk_waddr)
            rstk_rdata <= rstk_wdata;
        else
            rstk_rdata <= rstk[rstk_raddr];
    end

    assign dbg_T    = T;
    assign dbg_I    = I;
    assign dbg_P    = P;
    assign dbg_slot = slot;

    always @(posedge clk) begin
        if (!resetn) begin
            // F18A reset (DB001 §3.1):
            //   * P  is set to the layout-configured reset address
            //     (multiport-execute port or ROM 0xAA — for us the
            //      streaming INST_PORT).
            //   * B  is set to the address of io.
            //   * Stack pointers reset.
            //   * carry latch reset (we model P9's sticky carry
            //     explicitly, so power-up behavior stays deterministic).
            //   * T, S, R, A, I, dstk[], rstk[] are NOT reset — held
            //     state is preserved across reset (per spec).
            //   * The local FSM bookkeeping (st, slot, mem_addr) and
            //     the mem_we strobe must also be cleared so we don't
            //     issue spurious writes during the rst_cnt warm-up.
            st       <= ST_FETCH;
            slot     <= 2'd0;
            P        <= RESET_PC;
            B        <= {{(18-ADDR_BITS){1'b0}}, RESET_B};
            dsp      <= DSP_ZERO;
            rsp      <= RSP_ZERO;
`ifndef NO_P9_ARITH
            carry    <= 1'b0;
`endif
            mem_addr <= RESET_PC;
            mem_we   <= 1'b0;
        end else begin
            mem_we <= 1'b0;

            case (st)

                ST_FETCH: begin
                    mem_addr <= P;
                    st       <= ST_FWAIT;
                end

                ST_FWAIT: begin
                    // Stall fetch while the addressed slot is a port
                    // that hasn't produced a word yet (mem_ready=0).
                    if (mem_ready) st <= ST_FWAIT2;
                end

                ST_FWAIT2: begin
                    I    <= mem_rdata;
                    // Port execution (DB001 §3.3.2): hold P at the port
                    // so the next fetch reads the next streamed word
                    // from the same source. Plain RAM advances P
                    // normally.
                    if (P != PORT_ADDR) P <= P + 1'b1;
                    slot <= 2'd0;
                    st   <= ST_EXEC;
                end

                ST_EXEC: begin
                    if (slot == 2'd3) st <= ST_FETCH;
                    else              slot <= slot + 1'b1;

                    case (op)
                    OP_NOP: begin end

                    OP_RET: begin
                        if (rsp_empty) begin
                            R  <= 18'd0;
                            st <= ST_FETCH;
                        end else begin
                            P   <= R[ADDR_BITS-1:0];
                            R   <= rstk_rdata;
                            rsp <= rsp_prev;
                            st  <= ST_FETCH;
                        end
                    end

                    OP_JMP: begin
                        P  <= jaddr;
                        st <= ST_FETCH;
                    end

                    OP_IF: begin
                        if (T == 18'd0) P <= jaddr;
                        st <= ST_FETCH;
                    end

`ifndef NO_EX
                    OP_EX: begin
                        P <= R[ADDR_BITS-1:0];
                        R <= {{(18-ADDR_BITS){1'b0}}, P};
                        st <= ST_FETCH;
                    end
`endif
`ifndef NO_CALL
                    OP_CALL: begin
                        // rstk write happens via combinational rstk_we.
                        rsp <= rsp_next;
                        R   <= {{(18-ADDR_BITS){1'b0}}, P};
                        P   <= jaddr;
                        st  <= ST_FETCH;
                    end
`endif
`ifndef NO_MIF
                    OP_MIF: begin
                        if (!T[17]) P <= jaddr;
                        st <= ST_FETCH;
                    end
`endif
`ifndef NO_UNEX
                    OP_UNEX: begin
                        // DB001 §2.3: "unext loops back to slot 0 of the
                        // current word; when the loop is complete, execution
                        // continues with the slot following unext." So:
                        //   * branch case (R != 0): re-fetch the current word
                        //     so slot resets to 0.
                        //   * exit case (R == 0): pop R and let the default
                        //     ST_EXEC slot-advance fall through. If unext was
                        //     in slot 3, the default already goes to ST_FETCH
                        //     for the next word; if it was in slot 0/1/2 the
                        //     default advances slot in the cached I.
                        if (R_is_zero) begin
                            if (rsp_empty) begin
                                R <= 18'd0;
                            end else begin
                                R   <= rstk_rdata;
                                rsp <= rsp_prev;
                            end
                        end else begin
                            R  <= R_dec;
                            P  <= current_word;
                            st <= ST_FETCH;
                        end
                    end
`endif
`ifndef NO_NEXT
                    OP_NEXT: begin
                        if (R_is_zero) begin
                            if (rsp_empty) begin
                                R <= 18'd0;
                            end else begin
                                R   <= rstk_rdata;
                                rsp <= rsp_prev;
                            end
                        end else begin
                            R <= R_dec;
                            P <= jaddr;
                        end
                        st <= ST_FETCH;
                    end
`endif

                    // ── Memory ops ───────────────────────────
                    // Reads / writes present a registered address here,
                    // then the wait states below absorb the sync RAM delay.
                    OP_FETP: begin
                        mem_addr <= P;
                        // Same port-execution rule as fetch: don't
                        // advance P if it points at the comm port.
                        if (P != PORT_ADDR) P <= P + 1'b1;
                        slot <= slot;
                        st   <= ST_MWAIT;
                    end
                    OP_FETA: begin
                        mem_addr <= A[ADDR_BITS-1:0];
                        A    <= A + 18'd1;
                        slot <= slot;
                        st   <= ST_MWAIT;
                    end
                    OP_FETB: begin
                        mem_addr <= B[ADDR_BITS-1:0];
                        slot <= slot;
                        st   <= ST_MWAIT;
                    end
                    OP_FET: begin
                        mem_addr <= A[ADDR_BITS-1:0];
                        slot <= slot;
                        st   <= ST_MWAIT;
                    end

                    OP_STP: begin
                        mem_addr  <= P;
                        mem_wdata <= T;
                        mem_we    <= 1'b1;
                        P         <= P + 1'b1;
                        T         <= S;
                        S         <= dstk_rdata;
                        dsp       <= dsp_prev;
                        slot      <= slot;
                        st        <= ST_MEMWR;
                    end
                    OP_STA: begin
                        mem_addr  <= A[ADDR_BITS-1:0];
                        mem_wdata <= T;
                        mem_we    <= 1'b1;
                        A         <= A + 18'd1;
                        T         <= S;
                        S         <= dstk_rdata;
                        dsp       <= dsp_prev;
                        slot      <= slot;
                        st        <= ST_MEMWR;
                    end
                    OP_STB: begin
                        mem_addr  <= B[ADDR_BITS-1:0];
                        mem_wdata <= T;
                        mem_we    <= 1'b1;
                        T         <= S;
                        S         <= dstk_rdata;
                        dsp       <= dsp_prev;
                        slot      <= slot;
                        st        <= ST_MEMWR;
                    end
                    OP_ST: begin
                        mem_addr  <= A[ADDR_BITS-1:0];
                        mem_wdata <= T;
                        mem_we    <= 1'b1;
                        T         <= S;
                        S         <= dstk_rdata;
                        dsp       <= dsp_prev;
                        slot      <= slot;
                        st        <= ST_MEMWR;
                    end

                    // ── ALU ────────────────────────────────
                    OP_INV: T <= alu_inv;
`ifndef NO_SHL
                    OP_SHL: T <= alu_shl;
`endif
`ifndef NO_SHR
                    OP_SHR: T <= alu_shr;
`endif
`ifndef NO_MULS
                    OP_MULS: begin
`ifdef MULS_TRAP
                        // Plain CALL synthesis to MULS_HANDLER_ADDR. The
                        // handler returns to whatever word was about to be
                        // fetched after the +* word (= P), so any ops in
                        // slots 1..3 of the user's MULS word are skipped.
                        // Programs must put +* in slot 0 of its word.
                        rsp <= rsp_next;
                        R   <= {{(18-ADDR_BITS){1'b0}}, P};
                        P   <= MULS_HANDLER_ADDR;
                        st  <= ST_FETCH;
`else
  `ifndef NO_P9_ARITH
                        if (arith_ext && A[0]) carry <= alu_add_ext[18];
  `endif
                        T <= mul_newT;
                        A <= mul_newA;
`endif
                    end
`endif

                    OP_ADD: begin
                        T <= alu_add;
`ifndef NO_P9_ARITH
                        if (arith_ext) carry <= alu_add_ext[18];
`endif
                        S   <= dstk_rdata;
                        dsp <= dsp_prev;
                    end
                    OP_AND: begin
                        T   <= alu_and;
                        S   <= dstk_rdata;
                        dsp <= dsp_prev;
                    end
                    OP_XOR: begin
                        T   <= alu_xor;
                        S   <= dstk_rdata;
                        dsp <= dsp_prev;
                    end

                    // ── Stack ops ────────────────────────────
                    OP_DROP: begin
                        T   <= S;
                        S   <= dstk_rdata;
                        dsp <= dsp_prev;
                    end

                    OP_DUP: begin
                        // dstk write fires via combinational dstk_we.
                        dsp <= dsp_next;
                        S   <= T;
                    end

`ifndef NO_OVER
                    OP_OVER: begin
                        // ( a b -- a b a )
                        dsp <= dsp_next;
                        T <= S;
                        S <= T;
                    end
`endif
`ifndef NO_POP
                    OP_POP: begin
                        // R → data stack
                        dsp <= dsp_next;
                        S   <= T;
                        T   <= R;
                        if (rsp_empty) begin
                            R <= 18'd0;
                        end else begin
                            R   <= rstk_rdata;
                            rsp <= rsp_prev;
                        end
                    end
`endif
`ifndef NO_PUSH
                    OP_PUSH: begin
                        // data stack → R
                        rsp <= rsp_next;
                        R   <= T;
                        T   <= S;
                        S   <= dstk_rdata;
                        dsp <= dsp_prev;
                    end
`endif

                    OP_A: begin
                        dsp <= dsp_next;
                        S   <= T;
                        T   <= A;
                    end

                    OP_ASTO: begin
                        A   <= T;          // full 18 bits (MULS shifts through A)
                        T   <= S;
                        S   <= dstk_rdata;
                        dsp <= dsp_prev;
                    end

                    OP_BSTO: begin
                        B   <= T;
                        T   <= S;
                        S   <= dstk_rdata;
                        dsp <= dsp_prev;
                    end

                    default: begin end
                    endcase
                end

                ST_MWAIT: begin
                    // Hold here while the addressed slot is a blocking
                    // port that isn't ready (mem_ready=0). The SoC drops
                    // mem_ready for UART RX with no byte yet, etc.
                    if (mem_ready) st <= ST_MEMRD;
                end

                ST_MEMRD: begin
                    // dstk write fires via combinational dstk_we (st==ST_MEMRD).
                    dsp <= dsp_next;
                    S   <= T;
                    T   <= mem_rdata;

                    if (slot == 2'd3) st <= ST_FETCH;
                    else begin
                        slot <= slot + 1'b1;
                        st   <= ST_EXEC;
                    end
                end

                ST_MEMWR: begin
                    // Same blocking-port handshake as the read path.
                    // The SoC ignores the write while mem_ready=0, so
                    // we hold the address/data registers and re-pulse
                    // mem_we until it accepts. This protects mbox_10
                    // from being clobbered before TX has consumed it.
                    if (!mem_ready) begin
                        mem_we <= 1'b1;
                    end else if (slot == 2'd3) st <= ST_FETCH;
                    else begin
                        slot <= slot + 1'b1;
                        st   <= ST_EXEC;
                    end
                end

                default: st <= ST_FETCH;
            endcase
        end
    end

endmodule
