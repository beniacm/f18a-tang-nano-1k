// Single-port synchronous-read BSRAM for the F18A core's program / data
// memory. Inferred as one or more SP-mode BSRAM blocks on GW1NZ-1.
//
// Read latency: 1 cycle from the sampled addr input (addr in cycle N →
// dout in cycle N+1). f18a_core now registers mem_addr before it reaches
// this block, so fetches and data reads each carry one explicit extra
// wait cycle in the FSM to line up with the sampled address here.
//
// Write semantics: read-first. A simultaneous read of the address
// being written returns the OLD value. The F18A FSM never issues a
// read of the address it's also writing in the same cycle, so this
// distinction is academic for us.
//
// INIT_FILE: path passed to `$readmemh` at elaboration. Empty means
// "leave uninitialised" (ram contents at power-on are device-defined —
// the Gowin synth init flow drops in zeros for BSRAM, so the program
// must be loaded via the back-door W command before G if INIT_FILE is
// not provided).

module c1_bram #(
    parameter ADDR_BITS = 9,
    parameter DEPTH     = 512,
    parameter INIT_FILE = ""
) (
    input  wire                  clk,
    input  wire [ADDR_BITS-1:0]  addr,
    input  wire                  we,
    input  wire [17:0]           din,
    output reg  [17:0]           dout
);
    reg [17:0] mem [0:DEPTH-1];

    initial begin
        if (INIT_FILE != "") $readmemh(INIT_FILE, mem);
    end

    always @(posedge clk) begin
        if (we) mem[addr] <= din;
        dout <= mem[addr];
    end
endmodule
