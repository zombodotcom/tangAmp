// ============================================================================
// adc_vu_test.v
// ADC input level meter — shows PCM1802 input on LEDs as VU bar.
// No DAC output needed. Tests that ADC wiring and I2S receive work.
//
// LED[0] = heartbeat
// LED[1] = ADC activity (blinks when receiving valid samples)
// LED[2:5] = VU meter (4-segment level bar from ADC input)
//
// S1 = reset peak hold
// S2 = toggle peak hold mode (LED[5] shows peak instead of instant)
// ============================================================================

module adc_vu_test (
    input  wire clk_27m,
    input  wire btn_s1,    // pin 88, reset peak
    input  wire btn_s2,    // pin 87, toggle peak hold

    output wire adc_bck,   // I2S bit clock  (pin 76)
    output wire adc_lrck,  // I2S word clock (pin 77)
    input  wire adc_dout,  // ADC data (pin 48)
    output wire dac_din,   // unused but declared for CST compatibility

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

// DAC unused — drive low
assign dac_din = 1'b0;

// ── Power-on reset ───────────────────────────────────────────────────────
reg [9:0] por_cnt;
reg rst_n;
always @(posedge clk_27m) begin
    if (por_cnt < 10'd1023) begin
        por_cnt <= por_cnt + 1;
        rst_n <= 1'b0;
    end else begin
        rst_n <= 1'b1;
    end
end

// ── Audio Clock Generator ────────────────────────────────────────────────
wire bck, lrck, sample_en;

clk_audio_gen u_clkgen (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .bck       (bck),
    .lrck      (lrck),
    .sample_en (sample_en)
);

assign adc_bck  = bck;
assign adc_lrck = lrck;

// ── I2S Receiver ─────────────────────────────────────────────────────────
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

// ── Absolute value + VU levels ───────────────────────────────────────────
reg [31:0] abs_val;
reg [3:0] vu;
reg [31:0] peak_val;
reg peak_mode;

// Debounce S1 (peak reset)
reg [15:0] s1_db;
reg s1_prev;
wire s1_press = (!btn_s1 && s1_prev && s1_db == 0);

// Debounce S2 (peak mode toggle)
reg [15:0] s2_db;
reg s2_prev;
wire s2_press = (!btn_s2 && s2_prev && s2_db == 0);

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        s1_prev <= 1; s1_db <= 0;
        s2_prev <= 1; s2_db <= 0;
        peak_mode <= 0;
        peak_val <= 0;
    end else begin
        s1_prev <= btn_s1;
        s2_prev <= btn_s2;
        if (s1_db > 0) s1_db <= s1_db - 1;
        if (s2_db > 0) s2_db <= s2_db - 1;

        if (s1_press) begin
            peak_val <= 0;
            s1_db <= 16'hFFFF;
        end
        if (s2_press) begin
            peak_mode <= ~peak_mode;
            s2_db <= 16'hFFFF;
        end
    end
end

always @(posedge clk_27m) begin
    if (adc_valid) begin
        abs_val <= adc_audio[31] ? (~adc_audio + 1) : adc_audio;

        // Track peak
        if (abs_val > peak_val)
            peak_val <= abs_val;

        // VU thresholds (Q16.16 format)
        // These are quite low to catch guitar-level signals
        vu[0] <= (abs_val > 32'd256);       // any signal at all
        vu[1] <= (abs_val > 32'd4096);      // quiet
        vu[2] <= (abs_val > 32'd32768);     // moderate (0.5V)
        vu[3] <= (abs_val > 32'd131072);    // loud (2V)
    end
end

// ── Peak VU (for LED[5] in peak mode) ────────────────────────────────────
reg [3:0] peak_vu;
always @(posedge clk_27m) begin
    peak_vu[0] <= (peak_val > 32'd256);
    peak_vu[1] <= (peak_val > 32'd4096);
    peak_vu[2] <= (peak_val > 32'd32768);
    peak_vu[3] <= (peak_val > 32'd131072);
end

// ── Heartbeat ────────────────────────────────────────────────────────────
reg [24:0] hb_cnt;
reg hb;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin hb_cnt <= 0; hb <= 0; end
    else begin
        if (hb_cnt >= 25'd13_500_000) begin hb_cnt <= 0; hb <= ~hb; end
        else hb_cnt <= hb_cnt + 1;
    end
end

// ── ADC activity ─────────────────────────────────────────────────────────
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

// Active-low LEDs
assign led[0] = ~hb;
assign led[1] = ~act_led;
assign led[2] = ~vu[0];
assign led[3] = ~vu[1];
assign led[4] = ~vu[2];
assign led[5] = peak_mode ? ~peak_vu[3] : ~vu[3];

endmodule
