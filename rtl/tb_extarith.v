`timescale 1ns/1ps
`default_nettype none

module tb_extarith;
    localparam [4:0]
        OP_RET  = 5'o00,
        OP_JMP  = 5'o02,
        OP_FETP = 5'o10,
        OP_MULS = 5'o20,
        OP_SHL  = 5'o21,
        OP_SHR  = 5'o22,
        OP_ADD  = 5'o24,
        OP_XOR  = 5'o26,
        OP_NOP  = 5'o34,
        OP_ASTO = 5'o37;

    function [17:0] pack4;
        input [4:0] s0, s1, s2;
        input [4:0] s3;
        pack4 = {s0, s1, s2, s3[2:0]};
    endfunction

    function [17:0] br0;
        input [4:0] op;
        input [9:0] target;
        br0 = {op, 3'd0, target};
    endfunction

    reg clk = 0;
    always #10 clk = ~clk;
    reg resetn = 1'b0;

    wire [9:0]  mem_addr;
    wire        mem_we;
    wire [17:0] mem_wdata;
    reg  [17:0] mem_rdata;

    wire [17:0] dbg_T;
    wire [9:0]  dbg_P;

    f18a_core #(.ADDR_BITS(10), .RESET_PC(10'h000)) cpu (
        .clk       (clk),
        .resetn    (resetn),
        .mem_addr  (mem_addr),
        .mem_we    (mem_we),
        .mem_wdata (mem_wdata),
        .mem_rdata(mem_rdata), .mem_ready(1'b1),
        .dbg_T     (dbg_T),
        .dbg_I     (),
        .dbg_P     (dbg_P),
        .dbg_slot  ()
    );

    reg [17:0] mem [0:1023];
    always @(posedge clk)
        if (mem_we) mem[mem_addr] <= mem_wdata;
    always @(*) mem_rdata = mem[mem_addr];

    integer passed = 0;
    integer failed = 0;

    task clear_mem;
        integer i;
        for (i = 0; i < 1024; i = i + 1) mem[i] = 18'd0;
    endtask

    task reset_cpu;
        begin
            resetn = 1'b0;
            @(posedge clk); @(posedge clk);
            resetn = 1'b1;
            @(posedge clk);
        end
    endtask

    task run_until_p(input [9:0] sentinel, input integer max_cycles);
        integer i;
        begin
            for (i = 0; i < max_cycles; i = i + 1) begin
                @(posedge clk);
                if (dbg_P === sentinel) i = max_cycles;
            end
        end
    endtask

    task assert_eq18;
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

    task assert_eq1;
        input [255:0] name;
        input         got;
        input         want;
        begin
            if (got === want) begin
                $display("PASS %0s: %0d", name, got);
                passed = passed + 1;
            end else begin
                $display("FAIL %0s: got %0d want %0d", name, got, want);
                failed = failed + 1;
            end
        end
    endtask

    initial begin
        // ── Plain add ignores carry when P9=0 ─────────────────────
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_FETP, OP_ADD, OP_RET);
        mem[1] = 18'd5;
        mem[2] = 18'd7;
        mem[3] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        cpu.carry = 1'b1;
        run_until_p(10'h004, 80);
        assert_eq18("ADD normal: T", dbg_T, 18'd12);
        assert_eq1 ("ADD normal: carry unchanged", cpu.carry, 1'b1);

        // ── Slot-0 jump into P9 space enables add-with-carry ──────
        clear_mem;
        mem[0]       = br0(OP_JMP, 10'h200);
        mem[10'h200] = pack4(OP_FETP, OP_FETP, OP_ADD, OP_RET);
        mem[10'h201] = 18'd5;
        mem[10'h202] = 18'd7;
        mem[10'h203] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        cpu.carry = 1'b1;
        run_until_p(10'h204, 100);
        assert_eq18("ADD ext: T", dbg_T, 18'd13);
        assert_eq1 ("ADD ext: carry out", cpu.carry, 1'b0);

        // ── Plain +* ignores carry when P9=0 ──────────────────────
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_ASTO, OP_NOP, OP_RET);
        mem[1] = 18'd1;   // A = 1
        mem[2] = pack4(OP_FETP, OP_FETP, OP_MULS, OP_RET);
        mem[3] = 18'd1;   // S = 1
        mem[4] = 18'd0;   // T = 0
        mem[5] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        cpu.carry = 1'b1;
        run_until_p(10'h006, 120);
        assert_eq18("MULS normal: T", dbg_T, 18'd0);
        assert_eq18("MULS normal: A", cpu.A, 18'h20000);
        assert_eq1 ("MULS normal: carry unchanged", cpu.carry, 1'b1);

        // ── Slot-0 jump into P9 space enables carry-aware +* ──────
        clear_mem;
        mem[0]       = br0(OP_JMP, 10'h200);
        mem[10'h200] = pack4(OP_FETP, OP_ASTO, OP_NOP, OP_RET);
        mem[10'h201] = 18'd1;   // A = 1
        mem[10'h202] = pack4(OP_FETP, OP_FETP, OP_MULS, OP_RET);
        mem[10'h203] = 18'd1;   // S = 1
        mem[10'h204] = 18'd0;   // T = 0
        mem[10'h205] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        cpu.carry = 1'b1;
        run_until_p(10'h206, 140);
        assert_eq18("MULS ext: T", dbg_T, 18'd1);
        assert_eq18("MULS ext: A", cpu.A, 18'd0);
        assert_eq1 ("MULS ext: carry out", cpu.carry, 1'b0);

        // ── Plain SHL drops MSB, shifts in 0 (no carry interaction) ──
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_SHL, OP_NOP, OP_RET);
        mem[1] = 18'h2C001;        // bit 17 set, bit 0 set
        mem[2] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        cpu.carry = 1'b1;
        run_until_p(10'h003, 60);
        assert_eq18("SHL normal: drops MSB, fills 0", dbg_T, 18'h18002);
        assert_eq1 ("SHL normal: carry unchanged",     cpu.carry, 1'b1);

        // ── P9 SHL = rotate-through-carry left ───────────────────────
        clear_mem;
        mem[0]       = br0(OP_JMP, 10'h200);
        mem[10'h200] = pack4(OP_FETP, OP_SHL, OP_NOP, OP_RET);
        mem[10'h201] = 18'h2C001;
        mem[10'h202] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        cpu.carry = 1'b1;
        run_until_p(10'h203, 80);
        // input  = 0x2C001  bit 17 = 1
        // expect = (0x2C001 << 1) | carry_in = 0x18002 | 1 = 0x18003
        // new carry = old MSB = 1
        assert_eq18("SHL ext (RCL): T",      dbg_T,    18'h18003);
        assert_eq1 ("SHL ext (RCL): carry",  cpu.carry, 1'b1);

        // ── Plain SHR is arithmetic (sign-extends) ───────────────────
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_SHR, OP_NOP, OP_RET);
        mem[1] = 18'h2C001;        // bit 17 set, bit 0 set
        mem[2] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        cpu.carry = 1'b0;
        run_until_p(10'h003, 60);
        // sign-extend: bit 17 stays 1; result = 0x36000 | (0x2C001 >> 1) = 0x36000
        assert_eq18("SHR normal: arith (sign-ext)", dbg_T, 18'h36000);
        assert_eq1 ("SHR normal: carry unchanged",  cpu.carry, 1'b0);

        // ── P9 SHR = rotate-through-carry right ──────────────────────
        clear_mem;
        mem[0]       = br0(OP_JMP, 10'h200);
        mem[10'h200] = pack4(OP_FETP, OP_SHR, OP_NOP, OP_RET);
        mem[10'h201] = 18'h2C001;
        mem[10'h202] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        cpu.carry = 1'b1;
        run_until_p(10'h203, 80);
        // input  = 0x2C001, carry_in = 1
        // expect = (carry_in << 17) | (0x2C001 >> 1) = 0x20000 | 0x16000 = 0x36000
        // new carry = old LSB = 1
        assert_eq18("SHR ext (RCR): T",     dbg_T,    18'h36000);
        assert_eq1 ("SHR ext (RCR): carry", cpu.carry, 1'b1);

        // ── Plain XOR is bitwise XOR (no carry interaction) ──────────
        clear_mem;
        mem[0] = pack4(OP_FETP, OP_FETP, OP_XOR, OP_RET);
        mem[1] = 18'h2C0F0;
        mem[2] = 18'h0FFFF;
        mem[3] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        cpu.carry = 1'b1;
        run_until_p(10'h004, 80);
        // T_in = mem[2] = 0x0FFFF, S = mem[1] = 0x2C0F0
        // expect T_out = 0x2C0F0 ^ 0x0FFFF = 0x23F0F
        assert_eq18("XOR normal: T",      dbg_T, 18'h23F0F);
        assert_eq1 ("XOR normal: carry unchanged", cpu.carry, 1'b1);

        // ── P9 XOR = subtract-with-borrow ────────────────────────────
        // T - S - carry_in (= borrow); carry_out = 1 if borrow.
        // Pre-clear carry → straight subtract.
        clear_mem;
        mem[0]       = br0(OP_JMP, 10'h200);
        mem[10'h200] = pack4(OP_FETP, OP_FETP, OP_XOR, OP_RET);
        mem[10'h201] = 18'd5;       // S (loaded first, lands in S)
        mem[10'h202] = 18'd12;      // T (top: subtrahend? no, minuend on T)
        mem[10'h203] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        // Stack semantics for `-`: ( a b -- a-b ). Here S=5 (a), T=12 (b)?
        // Wait: @p loads literal as new T, pushes prev T to S.
        // Sequence: @p [5] → T=5. @p [12] → T=12, S=5.
        // XOR (P9 = sub): T_new = T - S - 0 = 12 - 5 = 7.
        reset_cpu;
        cpu.carry = 1'b0;
        run_until_p(10'h204, 100);
        assert_eq18("XOR ext (SUB): 12-5", dbg_T, 18'd7);
        assert_eq1 ("XOR ext (SUB): no borrow", cpu.carry, 1'b0);

        // ── P9 XOR with borrow: 5 - 12 - 0 = -7 (= 0x3FFF9), borrow=1 ──
        clear_mem;
        mem[0]       = br0(OP_JMP, 10'h200);
        mem[10'h200] = pack4(OP_FETP, OP_FETP, OP_XOR, OP_RET);
        mem[10'h201] = 18'd12;      // S
        mem[10'h202] = 18'd5;       // T
        mem[10'h203] = pack4(OP_NOP, OP_NOP, OP_NOP, OP_RET);
        reset_cpu;
        cpu.carry = 1'b0;
        run_until_p(10'h204, 100);
        assert_eq18("XOR ext (SUB): 5-12 wraps", dbg_T, 18'h3FFF9);
        assert_eq1 ("XOR ext (SUB): borrow",      cpu.carry, 1'b1);

        $display("\n=== EXT ARITH TESTS: %0d passed, %0d failed ===", passed, failed);
        if (failed == 0) $display("ALL PASS");
        else             $display("SOME TESTS FAILED");
        $finish;
    end

    initial begin
        #20000000 $display("TB TIMEOUT"); $finish;
    end
endmodule
