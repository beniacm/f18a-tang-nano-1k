`timescale 1ns/1ps

module tb_fullisa_case;
    function integer clog2_int;
        input integer value;
        integer v;
        begin
            v = value - 1;
            clog2_int = 0;
            while (v > 0) begin
                clog2_int = clog2_int + 1;
                v = v >> 1;
            end
            if (clog2_int == 0) clog2_int = 1;
        end
    endfunction

    localparam integer DSTK_DEPTH = 16;
    localparam integer RSTK_DEPTH = 32;
    localparam integer DSP_BITS   = clog2_int(DSTK_DEPTH);
    localparam integer RSP_BITS   = clog2_int(RSTK_DEPTH);

    reg clk = 0;
    always #1 clk = ~clk;

    reg resetn = 0;
    initial begin
        $readmemh("/tmp/current_case.hex", c1_mem.mem);
        #20 resetn = 1;
    end

    wire [9:0]  mem_addr;
    wire        mem_we;
    wire [17:0] mem_wdata;
    wire [17:0] mem_rdata;

    c1_bram #(.ADDR_BITS(10), .DEPTH(1024), .INIT_FILE("")) c1_mem (
        .clk(clk), .addr(mem_addr), .we(mem_we),
        .din(mem_wdata), .dout(mem_rdata)
    );

    f18a_core #(
        .ADDR_BITS(10),
        .DSTK_DEPTH(DSTK_DEPTH),
        .DSP_BITS(DSP_BITS),
        .RSTK_DEPTH(RSTK_DEPTH),
        .RSP_BITS(RSP_BITS),
        .RESET_PC(10'h000)
    ) cpu (
        .clk(clk), .resetn(resetn),
        .mem_addr(mem_addr), .mem_we(mem_we),
        .mem_wdata(mem_wdata), .mem_rdata(mem_rdata), .mem_ready(1'b1), .task_switch_req(1'b0), .task_switch_data(18'd0), .dbg_running_task(),
        .dbg_T(), .dbg_I(), .dbg_P(), .dbg_slot()
    );

    integer cycles = 0;
    reg done = 0;
    always @(posedge clk) if (resetn) begin
        cycles <= cycles + 1;
        if (mem_we && mem_addr == 10'h1E1 && !done) begin
            $display("EMIT 0x%02h at cycle %0d", mem_wdata[7:0], cycles);
            done <= 1;
        end
        if (cycles > 20000 && !done) begin
            $display("TIMEOUT at cycle %0d", cycles);
            $finish_and_return(1);
        end
        if (done) begin
            #20 $finish;
        end
    end
endmodule
