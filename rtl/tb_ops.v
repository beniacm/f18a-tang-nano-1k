// F18A per-opcode unit tests.
//
// Wraps the core with a 512-word async-read RAM. Each test:
//   1. Clears state (reset)
//   2. Seeds RAM / registers for the test
//   3. Runs the CPU until P reaches a sentinel address
//   4. Checks an expected register/memory outcome
//
// Prints "PASS <name>" or "FAIL <name>: got X, expected Y" per test.
// Summary + exit code at the end.
//
// Build + run:
//   iverilog -g2012 -o /tmp/tb_ops tb_ops.v f18a_core.v && vvp /tmp/tb_ops

`timescale 1ns/1ps
`default_nettype none

module tb_ops;

    // ── Opcodes (must match f18a_core.v) ─────────────
    localparam [4:0]
        OP_RET=5'o00, OP_EX=5'o01, OP_JMP=5'o02, OP_CALL=5'o03,
        OP_UNEX=5'o04, OP_NEXT=5'o05, OP_IF=5'o06, OP_MIF=5'o07,
        OP_FETP=5'o10, OP_FETA=5'o11, OP_FETB=5'o12, OP_FET=5'o13,
        OP_STP=5'o14, OP_STA=5'o15, OP_STB=5'o16, OP_ST=5'o17,
        OP_MULS=5'o20, OP_SHL=5'o21, OP_SHR=5'o22, OP_INV=5'o23,
        OP_ADD=5'o24, OP_AND=5'o25, OP_XOR=5'o26, OP_DROP=5'o27,
        OP_DUP=5'o30, OP_POP=5'o31, OP_OVER=5'o32, OP_A=5'o33,
        OP_NOP=5'o34, OP_PUSH=5'o35, OP_BSTO=5'o36, OP_ASTO=5'o37;

    // ── Instruction-word packers ─────────────────────
    // Standard 4-slot word: slot3 is a 3-bit padded opcode.
    function [17:0] pack4;
        input [4:0] s0, s1, s2;
        input [4:0] s3;
        pack4 = {s0, s1, s2, s3[2:0]};
    endfunction
    // Slot-0 branch: 5-bit op + 13-bit address (fits in ADDR_BITS=9).
    function [17:0] br0;
        input [4:0] op;
        input [8:0] target;
        br0 = {op, 4'd0, target};
    endfunction
    // Slot-0 regular op + slot-1 branch (8-bit target).
    function [17:0] op_br1;
        input [4:0] op0;
        input [4:0] opb;
        input [7:0] target;
        op_br1 = {op0, opb, target};
    endfunction

    // ── Clock + DUT ───────────────────────────────────
    reg clk = 0;
    always #10 clk = ~clk;     // 50 MHz; fast for sim
    reg resetn = 1'b0;

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
        .mem_rdata(mem_rdata), .mem_ready(1'b1),
        .dbg_T     (dbg_T),
        .dbg_I     (dbg_I),
        .dbg_P     (dbg_P),
        .dbg_slot  (dbg_slot)
    );

    // Async-read 512-word RAM
    reg [17:0] mem [0:511];
    always @(posedge clk)
        if (mem_we) mem[mem_addr] <= mem_wdata;
    always @(*) mem_rdata = mem[mem_addr];

    // ── Helpers ──────────────────────────────────────
    integer passed = 0;
    integer failed = 0;
    integer tsteps = 0;

    task clear_mem;
        integer i;
        for (i = 0; i < 512; i = i + 1) mem[i] = 18'd0;
    endtask

    task reset_cpu;
        begin
            resetn = 1'b0;
            @(posedge clk); @(posedge clk);
            resetn = 1'b1;
            @(posedge clk);
        end
    endtask

    task step(input integer n);
        integer i;
        for (i = 0; i < n; i = i + 1) @(posedge clk);
    endtask

    // Run until dbg_P reaches `sentinel` OR timeout.
    task run_until_p(input [8:0] sentinel, input integer max_cycles);
        integer i;
        begin
            for (i = 0; i < max_cycles; i = i + 1) begin
                @(posedge clk);
                if (dbg_P === sentinel) i = max_cycles;  // break
            end
            tsteps = tsteps + i;
        end
    endtask

    localparam [3:0] PTR_ONE = 4'd1;

    // Stack-depth accessors. Our core's stacks are 16-deep; we never push
    // more than 3 in tests. dstk[dsp-1] is the item "below" S.
    wire [17:0] dstk_top = cpu.dstk[cpu.dsp - PTR_ONE];
    wire [17:0] rstk_top = cpu.rstk[cpu.rsp - PTR_ONE];

    // Assertion helpers
    task assert_eq;
        input [255:0] name;
        input [17:0]  got;
        input [17:0]  want;
        begin
            if (got === want) begin
                $display("PASS %0s: 0x%05h", name, got);
                passed = passed + 1;
            end else begin
                $display("FAIL %0s: got 0x%05h want 0x%05h", name, got, want);
                failed = failed + 1;
            end
        end
    endtask

    task assert_eq9;
        input [255:0] name;
        input [8:0]   got;
        input [8:0]   want;
        begin
            if (got === want) begin
                $display("PASS %0s: 0x%03h", name, got);
                passed = passed + 1;
            end else begin
                $display("FAIL %0s: got 0x%03h want 0x%03h", name, got, want);
                failed = failed + 1;
            end
        end
    endtask

    // ── Tests ────────────────────────────────────────
    integer i;
    initial begin
        $dumpfile("tb_ops.vcd");
        $dumpvars(0, tb_ops);

        // ───────── NOP ─────────
        // Three NOPs + RET. After one full word, P should land on 1 (next
        // word, incremented by ST_FWAIT) and T should remain 0.
        clear_mem;
        mem[0] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        mem[1] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);   // park here
        mem[2] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h002, 40);
        assert_eq("NOP: T unchanged",  dbg_T, 18'd0);

        // ───────── FETP (@p) ─────────
        // @p reads literal from P, pushes to T. Use 3 @p's loading three
        // distinct literals; expect stack to end up with 11 on top and
        // 22 below it.
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_FETP, OP_FETP, OP_RET);
        mem[1] = 18'h00033;   // loaded first  (goes to T, then pushed as others come in)
        mem[2] = 18'h00022;
        mem[3] = 18'h00011;
        mem[4] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);  // park
        reset_cpu;
        run_until_p(9'h005, 60);
        assert_eq("FETP: T (top)",         dbg_T,    18'h00011);
        assert_eq("FETP: S (below)",       cpu.S,    18'h00022);
        assert_eq("FETP: dstk[dsp-1]",     dstk_top, 18'h00033);

        // ───────── DUP ─────────
        // @p 0x07 ; dup → ( 7 7 )
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_DUP, OP_NOP, OP_RET);
        mem[1] = 18'h00007;
        mem[2] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h003, 50);
        assert_eq("DUP: T",  dbg_T, 18'h00007);
        assert_eq("DUP: S",  cpu.S, 18'h00007);

        // ───────── DROP ─────────
        // @p 0x01 ; @p 0x02 ; drop → T=1, S=old below
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_FETP, OP_DROP, OP_RET);
        mem[1] = 18'h00001;
        mem[2] = 18'h00002;
        mem[3] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h004, 60);
        assert_eq("DROP: T",  dbg_T, 18'h00001);

        // ───────── ADD (+) ─────────
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_FETP, OP_ADD, OP_RET);
        mem[1] = 18'h00005;
        mem[2] = 18'h00003;
        mem[3] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h004, 60);
        assert_eq("ADD: T=5+3",  dbg_T, 18'h00008);

        // ───────── AND ─────────
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_FETP, OP_AND, OP_RET);
        mem[1] = 18'h00F0F;
        mem[2] = 18'h000FF;
        mem[3] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h004, 60);
        assert_eq("AND: 0x0F0F & 0x00FF",  dbg_T, 18'h0000F);

        // ───────── XOR ─────────
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_FETP, OP_XOR, OP_RET);
        mem[1] = 18'h00F0F;
        mem[2] = 18'h00055;
        mem[3] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h004, 60);
        assert_eq("XOR: 0x0F0F ^ 0x0055", dbg_T, 18'h00F5A);

        // ───────── INV (-) ─────────
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_INV, OP_NOP, OP_RET);
        mem[1] = 18'h00000;
        mem[2] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h003, 50);
        assert_eq("INV: ~0",  dbg_T, 18'h3FFFF);

        // ───────── SHL (2*) ─────────
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_SHL, OP_NOP, OP_RET);
        mem[1] = 18'h00081;
        mem[2] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h003, 50);
        assert_eq("SHL: 0x81 << 1",  dbg_T, 18'h00102);

        // ───────── SHR (2/) ─────────
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_SHR, OP_NOP, OP_RET);
        mem[1] = 18'h00102;
        mem[2] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h003, 50);
        assert_eq("SHR: 0x102 >> 1",  dbg_T, 18'h00081);

        // ───────── OVER ─────────
        // @p 0x01; @p 0x02; over → ( 1 2 1 ) so T=1, S=2, dstk_top=1
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_FETP, OP_OVER, OP_RET);
        mem[1] = 18'h00001;
        mem[2] = 18'h00002;
        mem[3] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h004, 60);
        assert_eq("OVER: T",         dbg_T,    18'h00001);
        assert_eq("OVER: S",         cpu.S,    18'h00002);
        assert_eq("OVER: dstk[-1]",  dstk_top, 18'h00001);

        // ───────── A (push A onto data stack) ─────────
        // Set A via `a!` then `a` pushes A back as T.
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_ASTO, OP_A, OP_RET);
        mem[1] = 18'h01234;
        mem[2] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h003, 50);
        assert_eq("A: pushes A",  dbg_T, 18'h01234);

        // ───────── ASTO (a!) ─────────
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_ASTO, OP_NOP, OP_RET);
        mem[1] = 18'h01FA5;
        mem[2] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h003, 50);
        assert_eq("ASTO: A register",  cpu.A, 18'h01FA5);

        // ───────── BSTO (b!) ─────────
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_BSTO, OP_NOP, OP_RET);
        mem[1] = 18'h005A5;
        mem[2] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h003, 50);
        assert_eq("BSTO: B register",  cpu.B, 18'h005A5);

        // ───────── FETA (@+) ─────────
        // Seed mem[5]=0xCAFE; @p 5 a! @+ → T = mem[5], A = 6
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_ASTO, OP_FETA, OP_RET);
        mem[1] = 18'h00005;         // lit for a!
        mem[2] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        mem[5] = 18'h0CAFE;
        reset_cpu;
        run_until_p(9'h003, 60);
        assert_eq("FETA: T=mem[5]",  dbg_T, 18'h0CAFE);
        assert_eq("FETA: A auto-inc", cpu.A, 18'h00006);

        // ───────── FET (@) ─────────
        // @p 7 a! @ → T = mem[7], A unchanged
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_ASTO, OP_FET, OP_RET);
        mem[1] = 18'h00007;
        mem[2] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        mem[7] = 18'h0F00D;
        reset_cpu;
        run_until_p(9'h003, 60);
        assert_eq("FET: T=mem[7]",  dbg_T, 18'h0F00D);
        assert_eq("FET: A unchanged", cpu.A, 18'h00007);

        // ───────── FETB (@b) ─────────
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_BSTO, OP_FETB, OP_RET);
        mem[1] = 18'h00009;
        mem[2] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        mem[9] = 18'h0BEEF;
        reset_cpu;
        run_until_p(9'h003, 60);
        assert_eq("FETB: T=mem[9]",  dbg_T, 18'h0BEEF);

        // ───────── STA (!+) ─────────
        // @p 12 a! @p 0xAA ! → writes mem[12]=0xAA but via STA auto-inc so
        // A ends up 13. Actually use STA directly.
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_ASTO, OP_FETP, OP_STA); // a! 12; @p 0xAA; !+
        // slot 3 must be a 3-bit-safe opcode. STA=0o15 → 3-bit 5. That's not
        // in SLOT3_OK (0o0/1/4). Restructure into two words.
        mem[0] = pack4(OP_FETP, OP_ASTO, OP_NOP, OP_RET);
        mem[1] = 18'h0000C;      // 12
        mem[2] = pack4(OP_FETP, OP_STA, OP_NOP, OP_RET);
        mem[3] = 18'h000AA;
        mem[4] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h005, 80);
        assert_eq("STA: mem[12]",      mem[18'h0C], 18'h000AA);
        assert_eq("STA: A auto-inc",   cpu.A,      18'h0000D);

        // ───────── ST (!) ─────────
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_ASTO, OP_NOP, OP_RET);
        mem[1] = 18'h00010;
        mem[2] = pack4(OP_FETP, OP_ST, OP_NOP, OP_RET);
        mem[3] = 18'h000BB;
        mem[4] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h005, 80);
        assert_eq("ST: mem[16]",      mem[18'h10], 18'h000BB);
        assert_eq("ST: A unchanged",  cpu.A,       18'h00010);

        // ───────── STB (!b) ─────────
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_BSTO, OP_NOP, OP_RET);
        mem[1] = 18'h00014;
        mem[2] = pack4(OP_FETP, OP_STB, OP_NOP, OP_RET);
        mem[3] = 18'h000CC;
        mem[4] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h005, 80);
        assert_eq("STB: mem[20]",  mem[18'h14], 18'h000CC);

        // ───────── STP (!p) ─────────
        // !p writes T to [P], then P++. After: mem[P_at_issue] = T, P has
        // advanced one past. Setup: @p 0xDD; !p → mem[next] = 0xDD; that
        // "next" location is the slot 2 of word 0 would consume — we do a
        // separate word to control positioning.
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_STP, OP_NOP, OP_RET);
        mem[1] = 18'h000DD;    // literal pushed to T
        mem[2] = 18'h00000;    // !p writes here (P=2 when !p runs)
        mem[3] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h004, 80);
        assert_eq("STP: mem[2]",  mem[2], 18'h000DD);

        // ───────── JMP ─────────
        // word 0 jumps to word 5 (a NOP/RET target). Use slot-0 branch.
        clear_mem;
        mem[0] = br0(OP_JMP, 9'h005);
        mem[5] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h006, 40);
        assert_eq9("JMP: landed",  dbg_P, 9'h006);

        // ───────── IF (jump if T==0) ─────────
        // With T=0, slot-0 if jumps to target.
        clear_mem;
        mem[0] = br0(OP_IF, 9'h005);
        mem[5] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h006, 40);
        assert_eq9("IF (T==0): taken",  dbg_P, 9'h006);

        // IF not taken (T != 0): fall through. @p 0x1; if:5 → at word 2
        // we still hit the sentinel RET-fallthrough path.
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_NOP, OP_NOP, OP_RET);
        mem[1] = 18'h00001;
        mem[2] = br0(OP_IF, 9'h00A);   // T!=0 → fall through to mem[3]
        mem[3] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);   // we should land here
        reset_cpu;
        run_until_p(9'h004, 80);
        assert_eq9("IF (T!=0): fell through", dbg_P, 9'h004);

        // ───────── MIF (-if: jump if T[17]==0) ─────────
        // T=0 is positive → T[17]=0 → MIF jumps.
        clear_mem;
        mem[0] = br0(OP_MIF, 9'h005);
        mem[5] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h006, 40);
        assert_eq9("MIF (T[17]=0): taken",  dbg_P, 9'h006);

        // MIF with negative T: push 0x20000 (bit 17 set) then MIF → fall through.
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_NOP, OP_NOP, OP_RET);
        mem[1] = 18'h20000;                  // T[17] = 1
        mem[2] = br0(OP_MIF, 9'h00A);        // fall through
        mem[3] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h004, 80);
        assert_eq9("MIF (T[17]=1): fell through",  dbg_P, 9'h004);

        // ───────── CALL / RET (round trip) ─────────
        // call to subroutine at 0x10, which runs RET. Return site = P after
        // the call (1). We expect to continue at mem[1].
        clear_mem;
        mem[0]     = br0(OP_CALL, 9'h010);
        mem[1]     = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);  // landing pad
        mem[18'h10]= pack4(OP_RET, OP_NOP, OP_NOP, OP_RET);  // pop R, return to 1
        reset_cpu;
        run_until_p(9'h002, 60);
        assert_eq9("CALL/RET: returned",  dbg_P, 9'h002);

        // ───────── PUSH / POP ─────────
        // @p 0xAB ; push → R=0xAB ; pop → T=0xAB back on data stack.
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_PUSH, OP_POP, OP_RET);
        mem[1] = 18'h000AB;
        mem[2] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h003, 60);
        assert_eq("PUSH/POP: T",  dbg_T, 18'h000AB);

        // ───────── EX (swap P ↔ R) ─────────
        // Put address 0x20 into R via push, then EX swaps P (=2 at that
        // moment) into R, and R (=0x20) into P. Landing at 0x20.
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_PUSH, OP_EX, OP_RET);
        mem[1] = 18'h00020;
        mem[18'h20] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h021, 60);
        assert_eq9("EX: P is target",  dbg_P, 9'h021);

        // ───────── UNEX (in-word loop) ─────────
        // Put 2 onto R via push (so R=2, data stack empty for ALU),
        // then in a single word: nop nop nop unext → loop 3 times total
        // (R=2,1,0 then fall). After finish P advances past. No visible
        // effect beyond reaching the next word, so check P lands correctly.
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_PUSH, OP_NOP, OP_RET);
        mem[1] = 18'h00002;                          // R <- 2 via push
        mem[2] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_UNEX);
        mem[3] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h004, 120);
        assert_eq9("UNEX: fell through at R=0",  dbg_P, 9'h004);
        assert_eq("UNEX: empty-rsp clears R",    cpu.R, 18'd0);

        // ───────── NEXT ─────────
        // Put 2 in R, then a word with next:back. Should loop 3 times to
        // the target, decrementing R each time. End R=whatever was below.
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_PUSH, OP_NOP, OP_RET);
        mem[1] = 18'h00002;                          // R <- 2
        mem[2] = br0(OP_NEXT, 9'h002);               // loop back to self until R==0
        mem[3] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h004, 150);
        assert_eq9("NEXT: fell through at R=0",  dbg_P, 9'h004);
        assert_eq("NEXT: empty-rsp clears R",    cpu.R, 18'd0);

        // ───────── POP with empty rsp ─────────
        clear_mem;
        mem[0] = pack4(OP_POP, OP_NOP, OP_NOP, OP_RET);
        mem[1] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        cpu.R = 18'h00123;
        run_until_p(9'h002, 60);
        assert_eq("POP empty-rsp: T gets R",     dbg_T, 18'h00123);
        assert_eq("POP empty-rsp: R cleared",    cpu.R, 18'd0);

        // ───────── MULS (+*) ─────────
        // Set T=0, S=3, A=5. One +* should give T=1, A= (pre[0]=3[0]=1 →
        // A_new = {1, 00000010} = 9'b100000010 → 0x102 ... in 18 bits).
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_BSTO, OP_FETP, OP_RET);  // B <- 3 (use B to stash)
        mem[1] = 18'h00003;
        mem[2] = pack4(OP_FETP, OP_ASTO, OP_NOP, OP_RET);   // wait wrong, lit followed by a!
        // Redo the setup more carefully: we want T=0, S=3, A=5 before MULS.
        // Sequence: @p 3 ; @p 5 ; a! (A=5, T=3) ; @p 0 ; drop ...
        // Simplest: @p 3 ; @p 5 ; a! — now S=3, T=5 pushed?
        //   @p 3: T=3, S=(don't care)
        //   @p 5: T=5, S=3
        //   a!: A=5, T=3, S=(deeper). Good — now T=3, S=3? No, S becomes
        //   dstk[dsp-1] which is uninitialised. Let's add another seed.
        //   @p 0 ; @p 3 ; @p 5 ; a!:
        //     T=0, then T=3 S=0, then T=5 S=3, then a!: A=5, T=3, S=0.
        // OK T=3, but we need T=0 for a fresh MULS accumulator. Set
        // up: @p 0 ; @p 3 ; @p 5 ; a! ; swap-less -- need another step.
        // Simpler: @p 0 push @p 3 @p 5 a! pop → T=0, S=3 still on stack,
        // but POP brings R=0 back.
        // Let's just use two words to stage it cleanly.
        clear_mem;
        // word 0: @p 5 a! — A=5, T was pushed but we clear below.
        mem[0] = pack4(OP_FETP, OP_ASTO, OP_NOP, OP_RET);
        mem[1] = 18'h00005;
        // word 2: @p 3 @p 0 — stack becomes ( 3 0 ) with T=0, S=3.
        mem[2] = pack4(OP_FETP, OP_FETP, OP_NOP, OP_RET);
        mem[3] = 18'h00003;
        mem[4] = 18'h00000;
        // word 5: +* (MULS)
        mem[5] = pack4(OP_MULS, OP_NOP, OP_NOP, OP_RET);
        mem[6] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h007, 120);
        // After setup: T=0, S=3, A=5 (bit 0 = 1).
        // After +*: pre = T+S = 3. T_new = {0, pre[17:1]} = 1.
        //           A_new = {pre[0], A[17:1]} = {1, 00...010} = 0x20002.
        assert_eq("MULS: T",  dbg_T, 18'h00001);
        assert_eq("MULS: A",  cpu.A, 18'h20002);

        // ═════════════════════════════════════════════
        // Edge-case tests
        // ═════════════════════════════════════════════

        // ── Slot-1 branch (8-bit address field) ──────
        // Word layout: slot 0 = NOP, slot 1 = JMP whose target sits in the
        // low 8 bits of the word. High bits of P are preserved.
        // Encoding: [NOP, JMP, addr[7:0]] ; slot 3 becomes addr[2:0].
        clear_mem;
        mem[0] = (OP_NOP << 13) | (OP_JMP << 8) | 18'h00A;  // jmp to 0x00A in slot 1
        mem[18'h0A] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h00B, 60);
        assert_eq9("JMP slot-1 (8-bit addr)",  dbg_P, 9'h00B);

        // ── Slot-2 branch (3-bit address field) ──────
        // P's high bits preserved, so target is within the current 8-word
        // window. From word 0, slot-2 can reach 0..7. Let's jump to 5.
        clear_mem;
        mem[0] = (OP_NOP << 13) | (OP_NOP << 8) | (OP_JMP << 3) | 3'b101;
        mem[18'h05] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h006, 60);
        assert_eq9("JMP slot-2 (3-bit addr)",  dbg_P, 9'h006);

        // ── SHR arithmetic (sign-bit preservation) ─────
        // T = 0x20000 (bit 17 set → "negative"). SHR should give 0x30000
        // (sign-extended). Check.
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_SHR, OP_NOP, OP_RET);
        mem[1] = 18'h20000;
        mem[2] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h003, 50);
        assert_eq("SHR: arithmetic (neg)",  dbg_T, 18'h30000);

        // ── Nested CALL/RET ──────────────────────────
        // Main calls A, A calls B, B returns, A returns, main falls through.
        clear_mem;
        mem[0] = br0(OP_CALL, 9'h010);                        // call A
        mem[1] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);       // landing
        mem[18'h10] = br0(OP_CALL, 9'h020);                   // A: call B
        mem[18'h11] = pack4(OP_RET, OP_NOP, OP_NOP, OP_RET);  // A: return after B
        mem[18'h20] = pack4(OP_RET, OP_NOP, OP_NOP, OP_RET);  // B: return
        reset_cpu;
        run_until_p(9'h002, 100);
        assert_eq9("CALL/RET nested: back to 2",  dbg_P, 9'h002);

        // ── UNEX count verification ──────────────────
        // Set A=0, push R=3, then run a word of (@+ nop nop unext) that
        // re-executes slot 0 each iteration. The `;` RET pad at end of
        // the PUSH word would pop R off — so we jump out instead.
        //   word 0: @p 0 a!
        //   word 2: @p 3    (T <- 3)
        //   word 4: push ; jump:loop   (R <- 3, then JMP to loop so RET
        //                               in slot 3 doesn't fire)
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_ASTO, OP_NOP, OP_RET);
        mem[1] = 18'h00000;
        mem[2] = pack4(OP_FETP, OP_NOP,  OP_NOP, OP_RET);
        mem[3] = 18'h00003;
        mem[4] = op_br1(OP_PUSH, OP_JMP, 8'h006);    // push; jump:6
        mem[6] = pack4(OP_FETA, OP_NOP, OP_NOP, OP_UNEX);
        mem[7] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h008, 200);
        // UNEX loops while R>0. R goes 3→2→1→0 through three decrement
        // iterations, then the R==0 pass falls through. The @+ in slot 0
        // runs once on each of those 4 total passes → A ends at 4.
        assert_eq("UNEX: A after loop",  cpu.A, 18'h00004);

        // ── IF boundary: T=1 does NOT jump ──
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_NOP, OP_NOP, OP_RET);
        mem[1] = 18'h00001;
        mem[2] = br0(OP_IF, 9'h00A);
        mem[3] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);   // expect here
        reset_cpu;
        run_until_p(9'h004, 80);
        assert_eq9("IF (T=1): fall through",  dbg_P, 9'h004);

        // ── MIF boundary: T with only bit 17 set → still negative ──
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_NOP, OP_NOP, OP_RET);
        mem[1] = 18'h20001;                 // bit 17 = 1 → "negative"
        mem[2] = br0(OP_MIF, 9'h00A);       // -if: fall through
        mem[3] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h004, 80);
        assert_eq9("MIF (T=0x20001 neg): fall",  dbg_P, 9'h004);

        // ── Stack depth: push+drop+push+drop sequence ──
        // Verify dsp wrapping isn't visible to correct code: 4 items
        // pushed+popped should come back consistently.
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_FETP, OP_FETP, OP_RET);
        mem[1] = 18'h00001;
        mem[2] = 18'h00002;
        mem[3] = 18'h00003;
        mem[4] = pack4(OP_FETP, OP_NOP, OP_NOP, OP_RET);
        mem[5] = 18'h00004;    // now stack T=4 S=3 ... dstk: 2,1
        mem[6] = pack4(OP_DROP, OP_DROP, OP_DROP, OP_RET);   // drops 4,3,2
        mem[7] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h008, 120);
        // After all drops: T=1, S=below.
        assert_eq("Stack: 4-deep → drop3 → T=1",  dbg_T, 18'h00001);

        // ── MULS multi-step ─────────────────────────
        // Initial: T=0, S=5, A=3 (18'h00003).
        // Step 1: A[0]=1, pre = T+S = 5. T ← {0, 5>>1} = 2.
        //                            A ← {pre[0]=1, A[17:1]=17'h00001}
        //                              = 18'h20001.
        // Step 2: A[0]=1 (=1 from 0x20001), pre = 2+5 = 7. T ← 3.
        //                            A ← {pre[0]=1, A[17:1]=17'h10000}
        //                              = 18'h30000.
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_ASTO, OP_NOP, OP_RET);   // A = 3
        mem[1] = 18'h00003;
        mem[2] = pack4(OP_FETP, OP_FETP, OP_NOP, OP_RET);   // push 5, then 0
        mem[3] = 18'h00005;
        mem[4] = 18'h00000;
        mem[5] = pack4(OP_MULS, OP_MULS, OP_NOP, OP_RET);   // +* +*
        mem[6] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        run_until_p(9'h007, 120);
        assert_eq("MULS×2: T",  dbg_T, 18'h00003);
        assert_eq("MULS×2: A",  cpu.A, 18'h30000);

        // ───────── FETP did literal already tested ─────────
        // (combined in other tests)

        // ── Summary ──────────────────────────────────
        $display("\n=== TESTS: %0d passed, %0d failed  (%0d cycles) ===",
                 passed, failed, tsteps);
        if (failed == 0) $display("ALL PASS");
        else             $display("SOME TESTS FAILED");
        $finish;
    end

    initial begin
        #20000000 $display("TB TIMEOUT"); $finish;
    end

endmodule
