// ============================================================================
// mclk_check.v — visual check that MCLK is actually 18 MHz.
//
// Divides MCLK by 2^25 (~33.5M) and toggles LED[2] from it.
// If MCLK = 18MHz → LED toggles at 18M / 33.5M / 2 ≈ 0.27 Hz (slow blink)
// If MCLK = 0Hz   → LED stays still
// If MCLK = 27MHz → LED toggles at 0.4 Hz (fastest plausible)
// If MCLK = 1.5MHz (the buggy half-rate I had) → LED toggles every ~22 sec
//
// Also bit-bangs MCLK directly out to mclk_out so the chip gets it.
// LED[3] does the same divide from clk_27m for a known reference rate.
//
// LED[0] = heartbeat (always)
// LED[1] = pll_lock
// LED[2] = MCLK divided down (compare blink rate to LED[3])
// LED[3] = clk_27m divided down (reference, ~0.4 Hz)
// LED[4] = LRCK divided (should be very slow ~0.0007 Hz, basically off)
// LED[5] = BCK divided (should be slow ~0.044 Hz, very slow)
// ============================================================================

module mclk_check (
    input  wire clk_27m,
    input  wire btn_s1,
    input  wire btn_s2,
    output wire mclk_out,
    output wire adc_bck,
    output wire adc_lrck,
    output wire dac_din,
    input  wire adc_dout,
    output wire sd_clk, sd_cmd, sd_dat1, sd_dat2, sd_dat3,
    input  wire sd_dat0,
    output wire [5:0] led
);

assign sd_clk=0; assign sd_cmd=1; assign sd_dat1=1; assign sd_dat2=1; assign sd_dat3=1;
assign dac_din=0;

reg [9:0] por_cnt; reg rst_n;
always @(posedge clk_27m) begin
    if (por_cnt < 10'd1023) begin por_cnt <= por_cnt + 1; rst_n <= 0; end
    else rst_n <= 1;
end

wire mclk, bck, lrck, sample_en, pll_lock;
clk_audio_pll u_pll (
    .clk_27m(clk_27m), .rst_n(rst_n),
    .mclk(mclk), .bck(bck), .lrck(lrck), .sample_en(sample_en), .pll_lock(pll_lock)
);
assign mclk_out = mclk;
assign adc_bck = bck;
assign adc_lrck = lrck;

// Heartbeat from clk_27m
reg [24:0] hb_cnt; reg hb;
always @(posedge clk_27m) begin
    if (hb_cnt >= 25'd13_500_000) begin hb_cnt <= 0; hb <= ~hb; end
    else hb_cnt <= hb_cnt + 1;
end

// Big divider on MCLK — blink rate proportional to actual MCLK frequency
reg [24:0] mclk_div;
always @(posedge mclk) mclk_div <= mclk_div + 1;

// Reference divider on clk_27m
reg [24:0] clk27m_div;
always @(posedge clk_27m) clk27m_div <= clk27m_div + 1;

// Divider on BCK (should toggle slow if BCK is running at 3 MHz)
reg [20:0] bck_div;
always @(posedge bck) bck_div <= bck_div + 1;

// Divider on LRCK (fast 46.875 kHz, will look like flickering)
reg [14:0] lrck_div;
always @(posedge lrck) lrck_div <= lrck_div + 1;

assign led[0] = ~hb;
assign led[1] = ~pll_lock;
assign led[2] = ~mclk_div[24];   // MCLK / 2^25 toggling
assign led[3] = ~clk27m_div[24]; // 27MHz / 2^25 toggling (reference)
assign led[4] = ~lrck_div[14];   // LRCK / 2^15 toggling
assign led[5] = ~bck_div[20];    // BCK / 2^21 toggling

endmodule
