// Standalone simulation testbench for the iterative Ackermann program.
//
// Drives f18a_core directly with a flat 1024-word memory pre-loaded from
// /tmp/ack.hex. Watches for a write to ACK_OUT (0x3F1 — the same address
// the SoC uses for UART_TX, so the program is hardware-portable) and
// prints the result.
//
// The program reads its inputs from B = INST_PORT (0x3F9). To keep this
// TB sim-only and self-contained, we mock the port: any read of address
// 0x3F9 returns the next input word from a 3-deep queue (count, n_init,
// m_init) and pops the queue. Reads after the queue empties return 0
// (the program shouldn't issue more than 3 input reads).
`timescale 1ns/1ps

module tb_ack;
    parameter integer DSTK_DEPTH = 128;
    parameter integer DSP_BITS   = 7;
    parameter integer TIMEOUT    = 2_000_000_000; // ns (≈ 1e9 cycles @ 500 MHz sim)

    // M_INIT, N_INIT, and repeat count are overridable at run time with
    // +m=N +n=N +r=N.
    integer m_init = 3;
    integer n_init = 3;
    integer repeat_count = 1;

    reg clk = 0;
    always #1 clk = ~clk;             // 500 MHz sim clock — meaningless in absolute terms

    // Memory.  Async read so the core's ST_FETCH/ST_FWAIT works.
    reg [17:0] mem [0:1023];

    reg resetn = 0;
    initial begin
        void'($value$plusargs("m=%d", m_init));
        void'($value$plusargs("n=%d", n_init));
        void'($value$plusargs("r=%d", repeat_count));
        $readmemh("/tmp/ack.hex", mem);
        #20 resetn = 1;
    end

    wire [10:0] mem_addr;
    wire        mem_we;
    wire [17:0] mem_wdata;
    reg  [17:0] mem_rdata;

    // Mock INST_PORT: a 3-deep FIFO of program inputs. The SoC's real
    // INST_PORT assembles bytes from UART; here we just feed the
    // already-known input words straight at the port read.
    localparam [10:0] INST_PORT = 11'h7F9;
    reg [17:0] inst_port_q [0:2];
    integer    inst_port_pos = 0;

    initial begin
        inst_port_q[0] = repeat_count;
        inst_port_q[1] = n_init;
        inst_port_q[2] = m_init;
    end

    // Bit 10 = I/O selector (matches f18a_soc); BSRAM is the low 10 bits.
    always @(*) begin
        if (mem_addr == INST_PORT)
            mem_rdata = (inst_port_pos < 3) ? inst_port_q[inst_port_pos] : 18'd0;
        else
            mem_rdata = mem[mem_addr[9:0]];
    end
    always @(posedge clk) if (mem_we && !mem_addr[10]) mem[mem_addr[9:0]] <= mem_wdata;

    // Pop the FIFO on completion of each read of INST_PORT — the core
    // captures mem_rdata in ST_MEMRD, so we advance the queue when we
    // see that state pulse. (We just look at the FSM state directly
    // since this is a simulation mock.)
    always @(posedge clk) if (resetn) begin
        if (cpu.st == 3'd5 /*ST_MEMRD*/ && mem_addr == INST_PORT
            && inst_port_pos < 3)
            inst_port_pos <= inst_port_pos + 1;
    end

    // Core under test.
    wire [17:0] dbg_T, dbg_I;
    wire [10:0] dbg_P;
    wire [1:0]  dbg_slot;
    f18a_core #(
        .ADDR_BITS  (11),
        .DSTK_DEPTH (DSTK_DEPTH),
        .DSP_BITS   (DSP_BITS),
        .RESET_PC   (11'h000)
    ) cpu (
        .clk(clk), .resetn(resetn),
        .mem_addr (mem_addr),
        .mem_we   (mem_we),
        .mem_wdata(mem_wdata),
        .mem_rdata(mem_rdata), .mem_ready(1'b1), .task_switch_req(1'b0), .task_switch_data(18'd0), .dbg_running_task(),
        .dbg_T(dbg_T), .dbg_I(dbg_I), .dbg_P(dbg_P), .dbg_slot(dbg_slot)
    );

    // Result watcher + dstk-depth high-water mark. The SoC ships with
    // a 64-deep dstack (BSRAM-backed), so any program that wants to
    // run on hardware needs peak dstk ≤ 64 — fail the sim loudly if
    // it doesn't.
    localparam integer HW_DSTK_DEPTH = 64;
    localparam [10:0] OUT_ADDR = 11'h7F1;
    integer cycles = 0;
    integer peak_dsp = 0;
    reg done = 1'b0;
    always @(posedge clk) if (resetn) begin
        cycles <= cycles + 1;
        if (cpu.dsp > peak_dsp) peak_dsp <= cpu.dsp;
        if (mem_we && mem_addr == OUT_ADDR && !done) begin
            if (repeat_count == 1)
                $display("[cycle %0d] A(%0d, %0d) = %0d (0x%05h)   peak dstk = %0d",
                         cycles, m_init, n_init, mem_wdata, mem_wdata, peak_dsp);
            else
                $display("[cycle %0d] A(%0d, %0d) x %0d = %0d (0x%05h)   peak dstk = %0d   cycles/iter = %0f",
                         cycles, m_init, n_init, repeat_count, mem_wdata, mem_wdata,
                         peak_dsp, cycles * 1.0 / repeat_count);
            if (peak_dsp > HW_DSTK_DEPTH)
                $display("FAIL peak dstk %0d exceeds hardware depth %0d",
                         peak_dsp, HW_DSTK_DEPTH);
            done <= 1'b1;
        end
    end

    initial begin
        // Optional VCD for post-mortem.
        if ($test$plusargs("vcd")) begin
            $dumpfile("/tmp/ack.vcd");
            $dumpvars(0, tb_ack);
        end
        wait (done);
        #20 $display("[cycle %0d] halted cleanly.", cycles);
        $finish;
    end

    initial begin
        #(TIMEOUT) $display("TIMEOUT after %0d cycles  (T=0x%05h, P=0x%03h, slot=%0d)",
                            cycles, dbg_T, dbg_P, dbg_slot);
        $finish;
    end
endmodule
