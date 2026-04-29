// F18A SoC for the Tang Nano 1K — one F18A softcore + UART + a GA144-
// style streaming instruction port. The core boots with P pointing at
// INST_PORT (DB001 §3.3.2 port-execution); the host streams 18-bit
// instruction words as packed UART triples, and the core executes them
// straight off the wire — the bitstream itself contains no monitor and
// no boot ROM.
//
// Memory map (1024×18 BSRAM):
//   0x000 .. 0x3EF : core RAM (host streams a loader here, then jumps in)
//   0x3F0 .. 0x3F2 : UART byte-mode I/O
//   0x3F9          : streamed instruction port
//
// I/O registers (core-visible):
//   0x3F0 UART_RX   — read: low byte = next received byte, pop on read
//                     blocks (mem_ready=0) when no byte is available
//   0x3F1 UART_TX   — write: enqueue low byte for transmit
//                     blocks while the previous byte is still in flight
//   0x3F2 UART_STAT — read: bit0 rx_avail, bit1 tx_room
//   0x3F9 INST_PORT — read: 3 incoming UART bytes assembled into one
//                     18-bit word; blocks until 3 bytes have arrived

module top (
    input  wire clk,
    output wire led_r,
    output wire led_g,
    output wire led_b,
    output wire uart_tx,
    input  wire uart_rx,
    input  wire button_a,    // Tang Nano 1K user buttons (active-low,
    input  wire button_b     // pulled up internally — pressed = 0)
);

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

// Default 64-deep dstack / 128-deep rstack. The stacks live in BSRAM
// (one block each — see f18a_core.v), so depth is essentially free
// up to the BSRAM word capacity (256 entries × 18 bits per cell).
// 128-deep rstk fits A(3,3) recursive (peak rsp = 121); 64-deep dstk
// is plenty for any reasonable F18A program. Override with
// `make f18a.fs DSTK_DEPTH=8 RSTK_DEPTH=8` for the GA144-faithful build.
`ifdef DSTK_DEPTH
    localparam integer C1_DSTK_DEPTH = `DSTK_DEPTH;
`else
    localparam integer C1_DSTK_DEPTH = 64;
`endif

`ifdef RSTK_DEPTH
    localparam integer C1_RSTK_DEPTH = `RSTK_DEPTH;
`else
    localparam integer C1_RSTK_DEPTH = 128;
`endif

    localparam integer C1_DSP_BITS = clog2_int(C1_DSTK_DEPTH);
    localparam integer C1_RSP_BITS = clog2_int(C1_RSTK_DEPTH);

    // ── System clock ─────────────────────────────────────────────────
    // Default: 27 MHz board oscillator straight through. Build with
    // `-DPLL_FREQ=54` (or 81 / 108) to multiply via the on-chip rPLL.
    // UART_DIV auto-scales from SYS_HZ.
`ifdef PLL_FREQ
    localparam integer SYS_HZ = `PLL_FREQ * 1_000_000;
    wire sys_clk;
    gowin_rpll #(.PLL_FREQ(`PLL_FREQ)) pll_inst (
        .clkin (clk),
        .clkout(sys_clk),
        .lock  ()
    );
`else
    localparam integer SYS_HZ = 27_000_000;
    wire sys_clk = clk;
`endif

    // ── POR reset + soft reset button ─────────────────────────────────
    // Free-running saturating counter; 16 bits (~2.4 ms at 27 MHz) is
    // long enough for the on-chip clock and BSRAM to settle. Button B
    // (Tang Nano 1K's nRST front-panel button, pin 44) acts as a soft
    // reset for the rest of the SoC: pressing it forces resetn=0, which
    // sends the F18A back to RESET_PC=0x3F9 (INST_PORT) and clears the
    // mailboxes / inst-port byte counter. BSRAM contents survive, but
    // the host can stream a fresh program back-to-back without
    // re-flashing the bitstream — same shape as a power-on cycle.
`ifdef SIM
    reg [7:0] rst_cnt = 0;
`else
    reg [15:0] rst_cnt = 0;
`endif
    reg [1:0] btn_b_sync = 2'b11;       // 11 = unpressed (active-low pad)
    always @(posedge sys_clk) btn_b_sync <= {btn_b_sync[0], button_b};
    wire por_done   = &rst_cnt;
    wire btn_b_high = btn_b_sync[1];
    wire resetn     = por_done & btn_b_high;
    always @(posedge sys_clk)
        if (!por_done) rst_cnt <= rst_cnt + 1'b1;

    // ══════════════════════════════════════════════════════════════════════
    // UART TX (115200 baud → SYS_HZ/115200 cycles/bit)
    // ══════════════════════════════════════════════════════════════════════
    localparam integer UART_DIV_INT = (SYS_HZ + 57600) / 115200;
    localparam integer UART_DIV_BITS = (UART_DIV_INT < 256)  ? 8  :
                                       (UART_DIV_INT < 1024) ? 10 :
                                       (UART_DIV_INT < 4096) ? 12 : 16;
    localparam [UART_DIV_BITS-1:0] UART_DIV = UART_DIV_INT[UART_DIV_BITS-1:0];

    reg        tx_start = 1'b0;
    reg  [7:0] tx_data  = 8'd0;
    reg        tx_busy = 1'b0;
    reg  [3:0] tx_bitcnt;
    reg  [9:0] tx_shift;
    reg  [UART_DIV_BITS-1:0] tx_div;

    assign uart_tx = tx_busy ? tx_shift[0] : 1'b1;

    always @(posedge sys_clk) begin
        if (tx_start && !tx_busy) begin
            tx_busy   <= 1'b1;
            tx_shift  <= {1'b1, tx_data, 1'b0};
            tx_bitcnt <= 4'd10;
            tx_div    <= {UART_DIV_BITS{1'b0}};
        end else if (tx_busy) begin
            if (tx_div == UART_DIV - 1) begin
                tx_div    <= {UART_DIV_BITS{1'b0}};
                tx_shift  <= {1'b1, tx_shift[9:1]};
                tx_bitcnt <= tx_bitcnt - 1'b1;
                if (tx_bitcnt == 4'd1) tx_busy <= 1'b0;
            end else begin
                tx_div <= tx_div + 1'b1;
            end
        end
    end

    // ══════════════════════════════════════════════════════════════════════
    // UART RX
    // ══════════════════════════════════════════════════════════════════════
    reg [1:0] rx_sync;
    reg       rx_busy  = 1'b0;
    reg [3:0] rx_bitcnt;
    reg [7:0] rx_shift;
    reg [UART_DIV_BITS-1:0] rx_div;
    reg [7:0] rx_data;
    reg       rx_avail = 1'b0;
    wire      rx_pop;

    reg [1:0] rx_idle_cnt = 2'd3;
    always @(posedge sys_clk) begin
        rx_sync <= {rx_sync[0], uart_rx};
        if (rx_busy)                  rx_idle_cnt <= 2'd0;
        else if (!rx_sync[1])         rx_idle_cnt <= 2'd0;
        else if (rx_idle_cnt != 2'd3) rx_idle_cnt <= rx_idle_cnt + 1'b1;

        if (!rx_busy) begin
            if (!rx_sync[1] && rx_idle_cnt >= 2'd1) begin
                rx_busy   <= 1'b1;
                rx_div    <= {UART_DIV_BITS{1'b0}};
                rx_bitcnt <= 4'd9;
            end
        end else if (rx_bitcnt == 4'd9) begin
            if (rx_div == (UART_DIV / 2) - 1) begin
                rx_div <= {UART_DIV_BITS{1'b0}};
                if (rx_sync[1]) rx_busy <= 1'b0;
                else            rx_bitcnt <= 4'd8;
            end else rx_div <= rx_div + 1'b1;
        end else begin
            if (rx_div == UART_DIV - 1) begin
                rx_div    <= {UART_DIV_BITS{1'b0}};
                rx_shift  <= {rx_sync[1], rx_shift[7:1]};
                rx_bitcnt <= rx_bitcnt - 1'b1;
                if (rx_bitcnt == 4'd1) begin
                    rx_data  <= {rx_sync[1], rx_shift[7:1]};
                    rx_avail <= 1'b1;
                    rx_busy  <= 1'b0;
                end
            end else rx_div <= rx_div + 1'b1;
        end
        if (rx_pop) rx_avail <= 1'b0;
    end

    // ══════════════════════════════════════════════════════════════════════
    // TX mailbox — one byte deep. The core writes UART_TX and stalls
    // (mem_ready=0) until the byte has been launched to the line.
    // ══════════════════════════════════════════════════════════════════════
    reg [17:0] mbox_10;
    reg        mbox_10_full = 1'b0;

    // 11-bit address space split top-bit by purpose:
    //   bit 10 = 0 → core RAM (full 1024 cells of program/data BSRAM)
    //   bit 10 = 1 → I/O registers (UART, LED, BUTTON, INST_PORT, …)
    // Real F18A nodes mirror code+ROM into the same byte-aligned window;
    // we trade that for a flat-but-large code page. The BSRAM module
    // takes only the low 10 bits; the I/O decode looks at bit 10.
    localparam integer C1_ADDR_BITS = 11;
    localparam integer C1_MEM_DEPTH = 1024;

    // ══════════════════════════════════════════════════════════════════════
    // INST_PORT byte-assembler (DB001 §3.3.2 port-execution).
    // While the core is fetching INST_PORT (and only then), each arriving
    // UART byte shifts into rx_inst_lo → rx_inst_mid → rx_inst_hi[1:0].
    // Once 3 bytes are assembled the SoC presents the 18-bit word at
    // INST_PORT and lets the fetch advance; the count clears one cycle
    // after the core captures the word, making room for the next.
    // ══════════════════════════════════════════════════════════════════════
    reg  [7:0] rx_inst_lo;
    reg  [7:0] rx_inst_mid;
    reg  [1:0] rx_inst_hi;
    reg  [1:0] rx_inst_count = 2'd0;
    reg        rx_inst_consume_d = 1'b0;

    // ── Button A GPIO ─────────────────────────────────────────────────
    // Two-FF synchroniser into sys_clk. Active-low at the pad → active-
    // high view at the BUTTON read register (bit 0). Button B is wired
    // as the soft reset (see por_done/btn_b_sync above) so it doesn't
    // appear here.
    reg [1:0] btn_a_sync = 2'b11;
    always @(posedge sys_clk)
        btn_a_sync <= {btn_a_sync[0], button_a};
    wire button_a_pressed = ~btn_a_sync[1];

    // ── LEDs (program-controlled GPIO) ────────────────────────────────
    // Memory-mapped at 0x3F3, bits 2:0 = R, G, B. Active-high in the
    // register; pin output is inverted because the Tang Nano 1K's
    // RGB LED is common-anode (active-low at the pad). On reset
    // led_reg=0 → all LEDs off, so a freshly-flashed bitstream
    // shows blank LEDs until the program writes 0x3F3.
    //
    // Real F18A nodes expose their I/O at 0x15D ("io" in ga-tools'
    // named-address table); we keep the LED in the 0x3Fx I/O band
    // alongside UART/INST_PORT so the existing IO decode covers it.
    reg [2:0] led_reg = 3'b000;
    assign led_r = ~led_reg[0];
    assign led_g = ~led_reg[1];
    assign led_b = ~led_reg[2];

    // ══════════════════════════════════════════════════════════════════════
    // Core 1 (F18A softcore)
    // ══════════════════════════════════════════════════════════════════════
    wire c1_resetn = resetn;

    wire [C1_ADDR_BITS-1:0]  c1_addr;
    wire        c1_we;
    wire [17:0] c1_wdata;
    reg  [17:0] c1_rdata;

    wire c1_mem_ready;

    f18a_core #(.ADDR_BITS(C1_ADDR_BITS), .DSTK_DEPTH(C1_DSTK_DEPTH), .DSP_BITS(C1_DSP_BITS),
                .RSTK_DEPTH(C1_RSTK_DEPTH), .RSP_BITS(C1_RSP_BITS),
                .RESET_PC(11'h7F9),         // boot at INST_PORT
                .MULS_HANDLER_ADDR(11'h3E0),
                .PORT_ADDR(11'h7F9),        // INST_PORT
                .RESET_B (11'h7F1)          // F18A spec: B set to addr of io
                ) cpu (
        .clk       (sys_clk),
        .resetn    (c1_resetn),
        .mem_addr  (c1_addr),
        .mem_we    (c1_we),
        .mem_wdata (c1_wdata),
        .mem_rdata (c1_rdata),
        .mem_ready (c1_mem_ready),
        .dbg_T     (), .dbg_I     (), .dbg_P     (), .dbg_slot  ()
    );

    // I/O registers live in the upper half of the 11-bit address space
    // (bit 10 = 1), so the lower 1024 cells are pure program/data RAM.
    localparam [C1_ADDR_BITS-1:0]
        C1_UART_RX   = 11'h7F0,
        C1_UART_TX   = 11'h7F1,
        C1_UART_STAT = 11'h7F2,
        C1_LED       = 11'h7F3,
        C1_BUTTON    = 11'h7F4,
        C1_INST_PORT = 11'h7F9;

    wire c1_is_io  = c1_addr[10];
    wire c1_is_ram = ~c1_is_io;
    wire c1_core_ram_we = c1_we && c1_is_ram;

    // While the core is fetching/reading INST_PORT, route incoming UART
    // bytes into the 3-byte instruction-assembly buffer instead of the
    // byte-mode rx_data path. The pop signal back to the UART RX FSM has
    // to fire on those bytes too, otherwise the next byte never makes
    // it through.
    wire fetch_inst_port = c1_resetn && !c1_we && c1_is_io
                           && c1_addr == C1_INST_PORT;
    wire rx_inst_capture = fetch_inst_port && rx_avail
                           && rx_inst_count != 2'd3;
    wire rx_byte_pop     = c1_resetn && !c1_we && c1_is_io
                           && c1_addr == C1_UART_RX && rx_avail;
    assign rx_pop = rx_byte_pop | rx_inst_capture;

    // GA144-style blocking neighbor ports:
    //   read UART_RX     blocks until rx_avail
    //   write UART_TX    blocks until !mbox_10_full (TX consumed prev)
    //   read INST_PORT   blocks until 3 bytes have been assembled into
    //                    one 18-bit word — this is the port-execution
    //                    feed (DB001 §3.3.2)
    // Every other I/O register stays non-blocking like BRAM.
    wire c1_io_blocked =
        c1_is_io && (
            (c1_addr == C1_UART_RX    && !c1_we && !rx_avail) ||
            (c1_addr == C1_UART_TX    &&  c1_we &&  mbox_10_full) ||
            (c1_addr == C1_INST_PORT  && !c1_we && rx_inst_count != 2'd3)
        );
    assign c1_mem_ready = !c1_io_blocked;

    // BSRAM only takes the low 10 bits — bit 10 (the I/O selector) is
    // dropped here. Writes are gated by c1_core_ram_we which already
    // requires c1_is_ram, so I/O writes don't bleed into the BSRAM.
    wire [17:0] c1_bram_dout;
    c1_bram #(
        .ADDR_BITS (10),
        .DEPTH     (C1_MEM_DEPTH),
        .INIT_FILE ("")
    ) c1_mem (
        .clk  (sys_clk),
        .addr (c1_addr[9:0]),
        .we   (c1_core_ram_we),
        .din  (c1_wdata),
        .dout (c1_bram_dout)
    );

    reg [17:0] c1_io_rdata;
    always @(*) begin
        case (c1_addr)
            C1_UART_RX:   c1_io_rdata = {10'd0, rx_data};
            C1_UART_STAT: c1_io_rdata = {16'd0, ~mbox_10_full, rx_avail};
            C1_LED:       c1_io_rdata = {15'd0, led_reg};
            C1_BUTTON:    c1_io_rdata = {17'd0, button_a_pressed};
            C1_INST_PORT: c1_io_rdata = {rx_inst_hi, rx_inst_mid, rx_inst_lo};
            default:      c1_io_rdata = 18'd0;
        endcase
    end

    always @(*) begin
        if (c1_is_io) c1_rdata = c1_io_rdata;
        else          c1_rdata = c1_bram_dout;
    end

    // ══════════════════════════════════════════════════════════════════════
    // State updates
    // ══════════════════════════════════════════════════════════════════════
    always @(posedge sys_clk) begin
        tx_start <= 1'b0;
        if (!resetn) begin
            mbox_10_full        <= 1'b0;
            rx_inst_count       <= 2'd0;
            rx_inst_consume_d   <= 1'b0;
        end else begin
            // INST_PORT byte-assembler.
            // Stage 1 (rx_inst_capture): a UART byte arrives while the
            //         core is fetching INST_PORT and the buffer isn't
            //         full — shift the byte into lo/mid/hi.
            // Stage 2 (rx_inst_consume_d): once count==3 the SoC
            //         presents the assembled word and drives mem_ready
            //         high. The core's fetch FSM advances on that
            //         posedge, so one cycle later we clear the count
            //         to make room for the next streamed instruction.
            //         (Holding the buffer for one extra cycle keeps
            //         c1_io_rdata valid through ST_FWAIT2's I<=mem_rdata
            //         capture.)
            rx_inst_consume_d <= 1'b0;
            if (rx_inst_consume_d) begin
                rx_inst_count <= 2'd0;
            end else if (fetch_inst_port && rx_inst_count == 2'd3) begin
                rx_inst_consume_d <= 1'b1;
            end else if (rx_inst_capture) begin
                case (rx_inst_count)
                    2'd0: rx_inst_lo  <= rx_data;
                    2'd1: rx_inst_mid <= rx_data;
                    2'd2: rx_inst_hi  <= rx_data[1:0];
                    default: ;
                endcase
                rx_inst_count <= rx_inst_count + 1'b1;
            end

            // Drain the TX mailbox to the line whenever the shifter is idle.
            if (mbox_10_full && !tx_busy) begin
                tx_data      <= mbox_10[7:0];
                tx_start     <= 1'b1;
                mbox_10_full <= 1'b0;
            end

            // Gate the TX-mailbox write on !mbox_10_full. Without the
            // gate, a write while the previous byte is still queued
            // would silently overwrite it; with c1_mem_ready also held
            // low for that case the core stalls in ST_MEMWR until the
            // slot opens, so the write commits exactly once.
            if (c1_resetn && c1_we && c1_is_io && c1_addr == C1_UART_TX
                && !mbox_10_full) begin
                mbox_10      <= c1_wdata;
                mbox_10_full <= 1'b1;
            end

            // LED register write — bottom 3 bits are R, G, B (active
            // high in software; pin output inverts for active-low pads).
            if (c1_resetn && c1_we && c1_is_io && c1_addr == C1_LED) begin
                led_reg <= c1_wdata[2:0];
            end
        end
    end

endmodule
