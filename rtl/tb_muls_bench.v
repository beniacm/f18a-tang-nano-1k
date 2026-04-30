// tb_muls_bench.v — measure cycles for the 16-step `+*` benchmark.
//
// Loads /tmp/muls_bench.hex into a flat 1K memory and measures the native
// `+*` benchmark. Counts cycles from reset release until the program writes
// the magic cookie 1 to 0x37E. Reports both the visible UART_TX byte
// (sanity check) and the total cycle count.
`timescale 1ns/1ps

module tb_muls_bench;
    reg clk = 0;
    always #1 clk = ~clk;
    reg resetn = 0;

    reg [17:0] mem [0:1023];
    integer i;
    initial begin
        for (i = 0; i < 1024; i = i + 1) mem[i] = 18'd0;
        $readmemh("/tmp/muls_bench.hex", mem);
        #20 resetn = 1;
    end

    wire [9:0]  mem_addr;
    wire        mem_we;
    wire [17:0] mem_wdata;
    reg  [17:0] mem_rdata;
    always @(posedge clk) if (resetn) begin
        if (mem_we) mem[mem_addr] <= mem_wdata;
        mem_rdata <= mem[mem_addr];
    end

    f18a_core #(
        .ADDR_BITS  (10),
        .DSTK_DEPTH (16),
        .DSP_BITS   (4),
        .RSTK_DEPTH (16),
        .RSP_BITS   (4),
        .RESET_PC   (10'h000)
    ) cpu (
        .clk(clk), .resetn(resetn),
        .mem_addr(mem_addr), .mem_we(mem_we),
        .mem_wdata(mem_wdata), .mem_rdata(mem_rdata), .mem_ready(1'b1), .task_switch_req(1'b0), .task_switch_data(18'd0), .dbg_running_task(),
        .dbg_T(), .dbg_I(), .dbg_P(), .dbg_slot()
    );

    integer cycles = 0;
    reg     done   = 1'b0;
    always @(posedge clk) if (resetn) begin
        cycles <= cycles + 1;
        if (mem_we && mem_addr == 10'h3F1) begin
            $display("UART_TX 0x%02h", mem_wdata[7:0]);
        end
        if (mem_we && mem_addr == 10'h37E && mem_wdata[7:0] === 8'h01 && !done) begin
            $display("CYCLES %0d", cycles);
            done <= 1'b1;
        end
        if (done) #4 $finish;
        if (cycles > 200000) begin
            $display("TIMEOUT cycles=%0d", cycles);
            $finish;
        end
    end
endmodule
