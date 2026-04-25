// ============================================================================
// adc_debug.v
// Diagnostic bitstream — shows exactly where the I2S RX chain is broken.
//
// LED[0] = heartbeat (FPGA alive)
// LED[1] = PLL locked (MCLK running)
// LED[2] = BCK is toggling (audio clock div working)
// LED[3] = LRCK is toggling
// LED[4] = DOUT is changing (PCM1808 sending non-stuck data)
// LED[5] = any adc_audio bit is non-zero (frame parsed correctly)
//
// All LEDs should be lit when working.
// ============================================================================

module adc_debug (
    input  wire clk_27m,
    input  wire btn_s1,
    input  wire btn_s2,

    output wire adc_bck,
    output wire adc_lrck,
    input  wire adc_dout,
    output wire dac_din,
    output wire mclk_out,

    output wire sd_clk,
    output wire sd_cmd,
    input  wire sd_dat0,
    output wire sd_dat1,
    output wire sd_dat2,
    output wire sd_dat3,

    output wire [5:0] led
);

assign sd_clk  = 1'b0;
assign sd_cmd  = 1'b1;
assign sd_dat1 = 1'b1;
assign sd_dat2 = 1'b1;
assign sd_dat3 = 1'b1;
assign dac_din = 1'b0;

// ── Reset ────────────────────────────────────────────────────────────────
reg [9:0] por_cnt;
reg rst_n;
always @(posedge clk_27m) begin
    if (por_cnt < 10'd1023) begin
        por_cnt <= por_cnt + 1;
        rst_n <= 1'b0;
    end else
        rst_n <= 1'b1;
end

// ── PLL audio clocks ────────────────────────────────────────────────────
wire mclk, bck, lrck, sample_en, pll_lock;

clk_audio_pll u_clkgen (
    .clk_27m   (clk_27m),
    .rst_n     (rst_n),
    .mclk      (mclk),
    .bck       (bck),
    .lrck      (lrck),
    .sample_en (sample_en),
    .pll_lock  (pll_lock)
);

assign adc_bck  = bck;
assign adc_lrck = lrck;
assign mclk_out = mclk;

// ── BCK toggle detector ─────────────────────────────────────────────────
reg bck_d;
reg [22:0] bck_seen_cnt;
reg bck_seen;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        bck_d <= 0; bck_seen_cnt <= 0; bck_seen <= 0;
    end else begin
        bck_d <= bck;
        if (bck != bck_d) begin
            bck_seen <= 1;
            bck_seen_cnt <= 0;
        end else if (bck_seen_cnt < 23'd5_000_000)
            bck_seen_cnt <= bck_seen_cnt + 1;
        else
            bck_seen <= 0;
    end
end

// ── LRCK toggle detector ────────────────────────────────────────────────
reg lrck_d;
reg [22:0] lrck_seen_cnt;
reg lrck_seen;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        lrck_d <= 0; lrck_seen_cnt <= 0; lrck_seen <= 0;
    end else begin
        lrck_d <= lrck;
        if (lrck != lrck_d) begin
            lrck_seen <= 1;
            lrck_seen_cnt <= 0;
        end else if (lrck_seen_cnt < 23'd5_000_000)
            lrck_seen_cnt <= lrck_seen_cnt + 1;
        else
            lrck_seen <= 0;
    end
end

// ── DOUT activity detector — sees any 1 bit on input ────────────────────
reg dout_d;
reg [22:0] dout_act_cnt;
reg dout_active;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        dout_d <= 0; dout_act_cnt <= 0; dout_active <= 0;
    end else begin
        dout_d <= adc_dout;
        if (adc_dout != dout_d) begin
            dout_active <= 1;
            dout_act_cnt <= 0;
        end else if (dout_act_cnt < 23'd5_000_000)
            dout_act_cnt <= dout_act_cnt + 1;
        else
            dout_active <= 0;
    end
end

// ── I2S RX ─────────────────────────────────────────────────────────────
wire signed [31:0] adc_audio;
wire               adc_valid;

i2s_rx u_rx (
    .clk     (clk_27m),
    .rst_n   (rst_n),
    .bck     (bck),
    .lrck    (lrck),
    .din     (adc_dout),
    .audio_l (adc_audio),
    .valid   (adc_valid)
);

// ── Audio data non-zero detector ────────────────────────────────────────
reg [22:0] data_seen_cnt;
reg data_nonzero;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        data_nonzero <= 0; data_seen_cnt <= 0;
    end else begin
        if (adc_valid && adc_audio != 32'd0) begin
            data_nonzero <= 1;
            data_seen_cnt <= 0;
        end else if (data_seen_cnt < 23'd5_000_000)
            data_seen_cnt <= data_seen_cnt + 1;
        else
            data_nonzero <= 0;
    end
end

// ── Heartbeat ───────────────────────────────────────────────────────────
reg [24:0] hb_cnt;
reg hb;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin hb_cnt <= 0; hb <= 0; end
    else begin
        if (hb_cnt >= 25'd13_500_000) begin hb_cnt <= 0; hb <= ~hb; end
        else hb_cnt <= hb_cnt + 1;
    end
end

assign led[0] = ~hb;
assign led[1] = ~pll_lock;
assign led[2] = ~bck_seen;
assign led[3] = ~lrck_seen;
assign led[4] = ~dout_active;
assign led[5] = ~data_nonzero;

endmodule
