// ============================================================================
// clk_audio_pll.v
// Audio clock generator with PLL-derived MCLK for PCM1808 ADC.
//
// Drop-in replacement for clk_audio_gen.v that ALSO outputs MCLK.
//
// Clock plan (PCM1808 in 384fs slave mode — set MD0=HIGH, MD1=GND):
//   27 MHz crystal -> rPLL -> 18 MHz  (MCLK / SCKI)
//   MCLK / 6  = 3 MHz       (BCK)
//   BCK  / 64 = 46.875 kHz  (LRCK)
//
//   18 MHz / 384 = 46.875 kHz sample rate (matches existing system)
//
// PLL math (Gowin rPLL: CLKOUT = FCLKIN × (FBDIV+1) / (IDIV+1), VCO = CLKOUT × ODIV)
//   IDIV_SEL  = 2   -> input  /3
//   FBDIV_SEL = 1   -> feedback ×2
//   ODIV_SEL  = 48  -> VCO post-divider
//   CLKOUT = 27 × 2 / 3   = 18 MHz exactly
//   VCO    = 18 × 48      = 864 MHz (in 500-1250 MHz range)
//
// PCM1808 wiring change required:
//   MD0 must be tied to 3.3V/5V (was GND for 256fs mode)
//   MD1 stays at GND
//
// Tang Nano 20K constraint required:
//   IO_LOC "mclk_out" 75;  (any free pin works, pin 75 matches existing wiring)
// ============================================================================

module clk_audio_pll (
    input  wire clk_27m,
    input  wire rst_n,
    output wire mclk,           // 18 MHz to PCM1808 SCK pin
    output wire bck,            // 3 MHz bit clock (gated by pll_lock)
    output wire lrck,           // 46.875 kHz word clock (gated by pll_lock)
    output wire sample_en,      // pulse on LRCK falling edge
    output wire pll_lock        // PLL locked indicator (good for debug LED)
);

// Internal undriven clock signals
reg bck_int;
reg lrck_int;

// ── Gowin rPLL: 27 MHz → 18 MHz ────────────────────────────────────────────
rPLL #(
    .FCLKIN              ("27"),
    .DEVICE              ("GW2AR-18C"),
    .IDIV_SEL            (2),         // /3
    .FBDIV_SEL           (1),         // ×2  → CLKOUT = 27 × 2 / 3 = 18 MHz
    .ODIV_SEL            (48),        // VCO = 18 × 48 = 864 MHz
    .DYN_SDIV_SEL        (2),
    .PSDA_SEL            ("0000"),
    .DUTYDA_SEL          ("1000"),
    .CLKOUT_FT_DIR       (1'b1),
    .CLKOUTP_FT_DIR      (1'b1),
    .CLKOUT_DLY_STEP     (0),
    .CLKOUTP_DLY_STEP    (0),
    .CLKFB_SEL           ("internal"),
    .CLKOUT_BYPASS       ("false"),
    .CLKOUTP_BYPASS      ("false"),
    .CLKOUTD_BYPASS      ("false"),
    .DYN_DA_EN           ("false"),
    .DYN_FBDIV_SEL       ("false"),
    .DYN_IDIV_SEL        ("false"),
    .DYN_ODIV_SEL        ("false")
) u_pll (
    .CLKIN    (clk_27m),
    .CLKFB    (1'b0),
    .RESET    (~rst_n),
    .RESET_P  (1'b0),
    .IDSEL    (6'b000000),
    .FBDSEL   (6'b000000),
    .ODSEL    (6'b000000),
    .PSDA     (4'b0000),
    .DUTYDA   (4'b0000),
    .FDLY     (4'b1111),
    .CLKOUT   (mclk),
    .LOCK     (pll_lock),
    .CLKOUTP  (),
    .CLKOUTD  (),
    .CLKOUTD3 ()
);

// ── BCK generation: divide 18 MHz MCLK by 6 → 3 MHz ──────────────────────
// Toggle every 3 MCLK cycles → half-period = 3, full period = 6 MCLK = 3 MHz.
reg [1:0] bck_cnt;

// Combine reset with PLL lock — keep clocks gated until PLL is stable
wire clk_rst_n = rst_n & pll_lock;

always @(posedge mclk or negedge clk_rst_n) begin
    if (!clk_rst_n) begin
        bck_cnt <= 2'd0;
        bck_int <= 1'b1;
    end else begin
        if (bck_cnt == 2'd2) begin
            bck_cnt <= 2'd0;
            bck_int <= ~bck_int;
        end else begin
            bck_cnt <= bck_cnt + 2'd1;
        end
    end
end

// ── BCK edge detect ────────────────────────────────────────────────────────
// I2S spec: LRCK changes on BCK FALLING edge so it's stable at next rising
reg bck_d;
wire bck_rise = bck_int & ~bck_d;
wire bck_fall = ~bck_int & bck_d;
always @(posedge mclk or negedge clk_rst_n) begin
    if (!clk_rst_n)
        bck_d <= 1'b0;
    else
        bck_d <= bck_int;
end

// ── LRCK generation: 64 BCK per LRCK frame, toggle on BCK falling ──────────
reg [5:0] lrck_cnt;

always @(posedge mclk or negedge clk_rst_n) begin
    if (!clk_rst_n) begin
        lrck_cnt <= 6'd0;
        lrck_int <= 1'b1;
    end else if (bck_fall) begin
        if (lrck_cnt == 6'd31) begin
            lrck_cnt <= 6'd0;
            lrck_int <= ~lrck_int;
        end else begin
            lrck_cnt <= lrck_cnt + 6'd1;
        end
    end
end

// Drive output clocks from internal regs
assign bck  = bck_int;
assign lrck = lrck_int;

// ── sample_en: pulse on LRCK falling edge (left-channel start) ─────────────
reg lrck_d;
wire lrck_fall = ~lrck_int & lrck_d;
always @(posedge mclk or negedge clk_rst_n) begin
    if (!clk_rst_n)
        lrck_d <= 1'b0;
    else
        lrck_d <= lrck_int;
end
assign sample_en = lrck_fall;

endmodule
