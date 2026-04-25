// ============================================================================
// dumb_passthrough.v
// Bit-level pass-through: ADC DOUT -> DAC DIN with same BCK/LRCK on both.
// No I2S parsing — just forwards the serial bitstream.
// If this produces clean audio, the chips work and our i2s_rx/i2s_tx
// is the bug. If still static, the clocks themselves are wrong.
// ============================================================================

module dumb_passthrough (
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

// Direct wire-through of bitstream (with one register to retime)
reg dout_reg;
always @(posedge mclk) dout_reg <= adc_dout;
assign dac_din = dout_reg;

// Heartbeat
reg [24:0] hb_cnt; reg hb;
always @(posedge clk_27m) begin
    if (hb_cnt >= 25'd13_500_000) begin hb_cnt <= 0; hb <= ~hb; end
    else hb_cnt <= hb_cnt + 1;
end

// DOUT activity detector
reg dout_d, dout_active_reg;
reg [22:0] dout_act_cnt;
always @(posedge mclk) begin
    dout_d <= adc_dout;
    if (adc_dout != dout_d) begin dout_active_reg <= 1; dout_act_cnt <= 0; end
    else if (dout_act_cnt < 23'd5_000_000) dout_act_cnt <= dout_act_cnt + 1;
    else dout_active_reg <= 0;
end

assign led[0] = ~hb;
assign led[1] = ~pll_lock;
assign led[2] = ~dout_active_reg;
assign led[3] = 1;
assign led[4] = 1;
assign led[5] = 1;

endmodule
