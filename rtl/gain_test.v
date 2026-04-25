// ============================================================================
// gain_test.v
// Variable gain test — ADC -> gain -> DAC
// Find the right input level for the triode engine.
//
// S1 = gain up (double)
// S2 = gain down (halve)
// Gain range: 0.25x to 64x (8 steps)
//
// LED[0] = heartbeat (1Hz)
// LED[1] = ADC activity
// LED[2:4] = gain level (binary, 0-7)
// LED[5] = clip indicator (output exceeds threshold)
// ============================================================================

module gain_test (
    input  wire clk_27m,
    input  wire btn_s1,
    input  wire btn_s2,

    output wire adc_bck,
    output wire adc_lrck,
    input  wire adc_dout,
    output wire dac_din,
    output wire mclk_out,    // 18MHz MCLK to PCM1808 SCK (pin 75)

    output wire sd_clk,
    output wire sd_cmd,
    input  wire sd_dat0,
    output wire sd_dat1,
    output wire sd_dat2,
    output wire sd_dat3,

    output wire [5:0] led
);

// SD card inactive
assign sd_clk  = 1'b0;
assign sd_cmd  = 1'b1;
assign sd_dat1 = 1'b1;
assign sd_dat2 = 1'b1;
assign sd_dat3 = 1'b1;

// ── Power-on reset ───────────────────────────────────────────────────────
reg [9:0] por_cnt;
reg rst_n;

always @(posedge clk_27m) begin
    if (por_cnt < 10'd1023) begin
        por_cnt <= por_cnt + 1;
        rst_n <= 1'b0;
    end else
        rst_n <= 1'b1;
end

// ── Audio Clocks ─────────────────────────────────────────────────────────
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

assign mclk_out = mclk;

assign adc_bck  = bck;
assign adc_lrck = lrck;

// ── I2S Receiver (ADC) ──────────────────────────────────────────────────
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

// ── Buttons ─────────────────────────────────────────────────────────────
reg [15:0] s1_db, s2_db;
reg s1_prev, s2_prev;
reg [2:0] gain_level;  // 0-7: 0.25x, 0.5x, 1x, 2x, 4x, 8x, 16x, 64x

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        s1_db <= 0; s2_db <= 0;
        s1_prev <= 1; s2_prev <= 1;
        gain_level <= 3'd3;  // default 2x
    end else begin
        s1_prev <= btn_s1;
        s2_prev <= btn_s2;
        if (s1_db > 0) s1_db <= s1_db - 1;
        if (s2_db > 0) s2_db <= s2_db - 1;

        // S1 = gain up
        if (!btn_s1 && s1_prev && s1_db == 0) begin
            if (gain_level < 3'd7) gain_level <= gain_level + 1;
            s1_db <= 16'hFFFF;
        end
        // S2 = gain down
        if (!btn_s2 && s2_prev && s2_db == 0) begin
            if (gain_level > 3'd0) gain_level <= gain_level - 1;
            s2_db <= 16'hFFFF;
        end
    end
end

// ── Gain Apply ──────────────────────────────────────────────────────────
// Levels: 0=0.25x(>>2), 1=0.5x(>>1), 2=1x, 3=2x(<<1), 4=4x(<<2),
//         5=8x(<<3), 6=16x(<<4), 7=64x(<<6)
reg signed [31:0] gained_audio;

always @(*) begin
    case (gain_level)
        3'd0: gained_audio = adc_audio >>> 2;
        3'd1: gained_audio = adc_audio >>> 1;
        3'd2: gained_audio = adc_audio;
        3'd3: gained_audio = adc_audio <<< 1;
        3'd4: gained_audio = adc_audio <<< 2;
        3'd5: gained_audio = adc_audio <<< 3;
        3'd6: gained_audio = adc_audio <<< 4;
        3'd7: gained_audio = adc_audio <<< 6;
    endcase
end

// ── I2S Transmitter (DAC) ───────────────────────────────────────────────
i2s_tx u_tx (
    .clk     (clk_27m),
    .rst_n   (rst_n),
    .bck     (bck),
    .lrck    (lrck),
    .audio_l (gained_audio),
    .audio_r (gained_audio),
    .load    (adc_valid),
    .dout    (dac_din)
);

// ── LED[0]: Heartbeat (~1Hz) ────────────────────────────────────────────
reg [24:0] hb_cnt;
reg hb;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin hb_cnt <= 0; hb <= 0; end
    else begin
        if (hb_cnt >= 25'd13_500_000) begin hb_cnt <= 0; hb <= ~hb; end
        else hb_cnt <= hb_cnt + 1;
    end
end

// ── LED[1]: ADC activity ────────────────────────────────────────────────
reg [19:0] act_cnt;
reg act_led;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin act_cnt <= 0; act_led <= 0; end
    else begin
        if (adc_valid) begin act_led <= 1; act_cnt <= 0; end
        else if (act_cnt < 20'd675_000) act_cnt <= act_cnt + 1;
        else act_led <= 0;
    end
end

// ── LED[5]: Clip indicator ──────────────────────────────────────────────
// Lights when output exceeds ~0.75 of full scale (approx ±49152 in Q16.16)
reg clip_led;
reg [19:0] clip_cnt;

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin clip_led <= 0; clip_cnt <= 0; end
    else begin
        if (adc_valid) begin
            if (gained_audio > 32'sd3211264 || gained_audio < -32'sd3211264) begin
                clip_led <= 1;
                clip_cnt <= 0;
            end
        end
        if (clip_cnt < 20'd675_000)
            clip_cnt <= clip_cnt + 1;
        else
            clip_led <= 0;
    end
end

// Active-low LEDs
assign led[0] = ~hb;
assign led[1] = ~act_led;
assign led[2] = ~gain_level[0];
assign led[3] = ~gain_level[1];
assign led[4] = ~gain_level[2];
assign led[5] = ~clip_led;

endmodule
