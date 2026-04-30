// tb_forth_runtime.v — fast iverilog harness for compiled Forth lines.
//
// Pre-loads /tmp/forth_run.hex (a flat 2K image emitted by forth_compile.py
// + asm.py) into a synchronous RAM, emulates the FULLISA UART surface
// (UART_STAT.tx_room always set; UART_RX/UART_STAT.rx_avail unused for
// now), and prints every UART_TX byte as a single line:
//
//     OUT 0xNN
//
// Stops on either:
//   * a write to the "done" RAM cell at 0x37E (the compiler emits a
//     write of 1 there as the last action before its halt loop),
//   * a configurable cycle budget (defaults to 200_000 cycles, override
//     with +cycles=N).
//
// Using a RAM-write marker instead of an in-band UART byte means any
// byte value (including 0x04) is allowed inside user output.

`timescale 1ns/1ps

module tb_forth_runtime;
    reg clk = 0;
    always #1 clk = ~clk;
    reg resetn = 0;

    wire [10:0] mem_addr;
    wire        mem_we;
    wire [17:0] mem_wdata;
    reg  [17:0] mem_rdata;
    reg  [17:0] mem [0:1023];

    integer cycle_budget = 200000;
    integer i;
    initial begin
        for (i = 0; i < 1024; i = i + 1) mem[i] = 18'd0;
        $readmemh("/tmp/forth_run.hex", mem);
        void'($value$plusargs("cycles=%d", cycle_budget));
        #20 resetn = 1;
    end

    wire is_stat = (mem_addr == 11'h7F2);
    wire is_tx   = (mem_addr == 11'h7F1);
    wire is_rx   = (mem_addr == 11'h7F0);
    wire is_io   = mem_addr[10];

    always @(posedge clk) if (resetn) begin
        if (mem_we && !is_io) mem[mem_addr[9:0]] <= mem_wdata;
        if (is_stat)      mem_rdata <= 18'h00002;   // tx_room=1 always
        else if (is_rx)   mem_rdata <= 18'h00000;
        else if (is_tx)   mem_rdata <= 18'h00000;
        else              mem_rdata <= mem[mem_addr[9:0]];
    end

    f18a_core #(
        .ADDR_BITS(11),
        .DSTK_DEPTH(16),
        .DSP_BITS(4),
        .RSTK_DEPTH(32),
        .RSP_BITS(5),
        .RESET_PC(11'h000)
    ) cpu (
        .clk(clk), .resetn(resetn),
        .mem_addr(mem_addr), .mem_we(mem_we),
        .mem_wdata(mem_wdata), .mem_rdata(mem_rdata), .mem_ready(1'b1), .task_switch_req(1'b0), .task_switch_data(18'd0), .dbg_running_task(),
        .dbg_T(), .dbg_I(), .dbg_P(), .dbg_slot()
    );

    wire is_done_marker = (mem_addr == 11'h37E) && mem_we;

    integer cycles = 0;
    reg     done   = 1'b0;
    always @(posedge clk) if (resetn) begin
        cycles <= cycles + 1;
        if (mem_we && is_tx) begin
            $display("OUT 0x%02h", mem_wdata[7:0]);
        end
        if (is_done_marker) begin
            $display("DONE cycle=%0d", cycles);
            done <= 1'b1;
        end
        if (done) begin
            #4 $finish;
        end
        if (cycles > cycle_budget) begin
            $display("TIMEOUT cycles=%0d", cycles);
            $finish;
        end
    end
endmodule
