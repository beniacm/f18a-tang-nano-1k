// Sim the native +* path through the c1_bram wrapper, run the core, and
// watch the !b emit at the end.
`timescale 1ns/1ps

module tb_muls;
    parameter integer DSTK_DEPTH = 64;
    parameter integer DSP_BITS   = 6;

    reg clk = 0;
    always #1 clk = ~clk;

    reg resetn = 0;
    initial begin
        // Overlay debug_mulstep program (T=0,S=10,A=11,+*,!b)
        c1_mem.mem[18'h000] = 18'h11EE0;   // @p b!
        c1_mem.mem[18'h001] = 18'h001E1;
        c1_mem.mem[18'h002] = 18'h11CE0;   // @p
        c1_mem.mem[18'h003] = 18'h0000A;   // 10
        c1_mem.mem[18'h004] = 18'h11CE0;   // @p
        c1_mem.mem[18'h005] = 18'h00000;   // 0
        c1_mem.mem[18'h006] = 18'h11CE0;   // @p
        c1_mem.mem[18'h007] = 18'h0000B;   // 11
        c1_mem.mem[18'h008] = 18'h3FCE0;   // a!
        c1_mem.mem[18'h009] = 18'h21CE0;   // +*
        c1_mem.mem[18'h00A] = 18'h1DCE0;   // !b
        c1_mem.mem[18'h00B] = 18'h0400B;   // jump:halt
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
        .ADDR_BITS  (10),
        .DSTK_DEPTH (DSTK_DEPTH),
        .DSP_BITS   (DSP_BITS),
        .RESET_PC   (10'h000)
    ) cpu (
        .clk(clk), .resetn(resetn),
        .mem_addr(mem_addr), .mem_we(mem_we),
        .mem_wdata(mem_wdata), .mem_rdata(mem_rdata), .mem_ready(1'b1),
        .dbg_T(), .dbg_I(), .dbg_P(), .dbg_slot()
    );

    // Watch for write to 0x1E1
    integer cycles = 0;
    reg done = 0;
    always @(posedge clk) if (resetn) begin
        cycles <= cycles + 1;
        if (mem_we && mem_addr == 10'h1E1 && !done) begin
            if (mem_wdata !== 18'h00005) begin
                $display("FAIL native MULS via c1_bram: expected 0x00005, got 0x%05h", mem_wdata);
                $finish_and_return(1);
            end
            $display("PASS native MULS via c1_bram: T=0x%05h at cycle %0d", mem_wdata, cycles);
            done <= 1;
        end
        if (cycles > 5000 && !done) begin
            $display("TIMEOUT  cycles=%0d  P=0x%03h", cycles, cpu.P);
            $finish_and_return(1);
        end
        if (done) begin
            #20 $finish;
        end
    end
endmodule
