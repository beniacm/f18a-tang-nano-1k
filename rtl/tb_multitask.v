// tb_multitask.v — fast iverilog harness for the multitasking demo.
//
// Drives the f18a_core directly (no SoC), mimics the SoC's TASK_CTRL
// + UART_TX behaviour: every write to TASK_CTRL (0x7F7) pulses
// task_switch_req for one cycle with the data; every write to
// UART_TX (0x7F1) is captured and printed. Loads /tmp/mt_abab.hex
// into a flat 2K word RAM (lower 1K = code/data, upper 0x400..7FF
// = the I/O band — but only 0x7F1, 0x7F7 are observed; everything
// else returns 0).
//
// Stops after capturing N bytes (default 16, override with +bytes=N).

`timescale 1ns/1ps

module tb_multitask;
    reg clk = 0;
    always #1 clk = ~clk;
    reg resetn = 0;

    wire [10:0] mem_addr;
    wire        mem_we;
    wire [17:0] mem_wdata;
    reg  [17:0] mem_rdata;
    reg  [17:0] mem [0:2047];

    integer max_bytes = 16;
    integer i;
    initial begin
        for (i = 0; i < 2048; i = i + 1) mem[i] = 18'd0;
        $readmemh("/tmp/mt_abab.hex", mem);
        void'($value$plusargs("bytes=%d", max_bytes));
        #20 resetn = 1;
    end

    // SoC stub: forward TASK_CTRL writes as task_switch_req pulses.
    reg          task_switch_req  = 1'b0;
    reg  [17:0]  task_switch_data = 18'd0;
    wire is_io       = mem_addr[10];
    wire is_uart_tx  = (mem_addr == 11'h7F1);
    wire is_task_ctl = (mem_addr == 11'h7F7);

    always @(posedge clk) if (resetn) begin
        if (mem_we && !is_io) mem[mem_addr[9:0]] <= mem_wdata;
        mem_rdata <= mem[mem_addr[9:0]];   // I/O reads aren't exercised here
        task_switch_req <= 1'b0;
        if (mem_we && is_task_ctl) begin
            task_switch_req  <= 1'b1;
            task_switch_data <= mem_wdata;
        end
    end

    f18a_core #(
        .ADDR_BITS(11),
        .NTASKS(2), .TASK_BITS(1),
        .DSTK_DEPTH(64), .DSP_BITS(6),
        .RSTK_DEPTH(128), .RSP_BITS(7),
        .RESET_PC(11'h000),
        .TASK1_PC(11'h00B)
    ) cpu (
        .clk(clk), .resetn(resetn),
        .mem_addr(mem_addr), .mem_we(mem_we),
        .mem_wdata(mem_wdata), .mem_rdata(mem_rdata),
        .mem_ready(1'b1),
        .task_switch_req(task_switch_req),
        .task_switch_data(task_switch_data),
        .dbg_running_task(),
        .dbg_T(), .dbg_I(), .dbg_P(), .dbg_slot()
    );

    integer cycles    = 0;
    integer bytes_out = 0;
    integer cap_max   = 256;
    reg [7:0] capture [0:255];

    integer trace_until = 0;
    initial void'($value$plusargs("trace=%d", trace_until));
    always @(posedge clk) if (resetn) begin
        cycles <= cycles + 1;
        if (cycles < trace_until)
            $display("cyc=%0d task=%0d st=%0d slot=%0d P=%03h addr=%03h we=%b wd=%h rd=%h ti=%b pend=%b sidx=%0d nxt=%0d",
                     cycles, cpu.running_task, cpu.st, cpu.slot, cpu.P,
                     mem_addr, mem_we, mem_wdata, mem_rdata,
                     task_switch_req, cpu.pending_switch,
                     cpu.swctl_idx, cpu.next_task);
        if (mem_we && is_uart_tx) begin
            $display("UART %0d: 0x%02h '%s' @ cycle %0d task=%0d",
                     bytes_out, mem_wdata[7:0], mem_wdata[7:0],
                     cycles, cpu.running_task);
            if (bytes_out < cap_max) capture[bytes_out] <= mem_wdata[7:0];
            bytes_out <= bytes_out + 1;
            if (bytes_out + 1 >= max_bytes) begin
                $display("DONE %0d bytes in %0d cycles", bytes_out + 1, cycles);
                #4 $finish;
            end
        end
        if (cycles > 200000) begin
            $display("TIMEOUT cycles=%0d bytes=%0d", cycles, bytes_out);
            $finish;
        end
    end
endmodule
