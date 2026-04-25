// ============================================================================
// tangamp_passthrough.v
// Minimal I2S passthrough: ADC -> DAC (no processing)
// Use this to verify hardware wiring before enabling the amp chain.
//
// LED[0] = heartbeat (1Hz blink = clock is running)
// LED[1] = ADC activity (blinks when receiving I2S data)
// LED[5] = always on (indicates passthrough mode)
// ============================================================================

module tangamp_passthrough (
    input  wire clk_27m,       // 27MHz crystal, pin 52
    input  wire btn_s1,        // Button S1, pin 88 (unused, active low)
    input  wire btn_s2,        // Button S2, pin 87 (unused, active low)

    // I2S clocks (directly drive both ADC and DAC from these pins)
    output wire adc_bck,       // bit clock  (pin 76)
    output wire adc_lrck,      // word clock (pin 77)
    output wire mclk_out,      // 18MHz MCLK to PCM1808 SCK (pin 75)

    // I2S data
    input  wire adc_dout,      // serial data from PCM1808 ADC (pin 48)
    output wire dac_din,       // serial data to PCM5102 DAC   (pin 49)

    // SD card (directly drive unused to safe state)
    output wire sd_clk,
    output wire sd_cmd,
    input  wire sd_dat0,
    output wire sd_dat1,
    output wire sd_dat2,
    output wire sd_dat3,

    // Status LEDs
    output wire [5:0] led
);

// SD card inactive
assign sd_clk  = 1'b0;
assign sd_cmd  = 1'b1;
assign sd_dat1 = 1'b1;
assign sd_dat2 = 1'b1;
assign sd_dat3 = 1'b1;

// ── Power-on reset ───────────────────────────────────────────────────────
reg [9:0] por_cnt = 0;
reg rst_n = 0;

always @(posedge clk_27m) begin
    if (por_cnt < 10'd1023) begin
        por_cnt <= por_cnt + 1;
        rst_n <= 1'b0;
    end else begin
        rst_n <= 1'b1;
    end
end

// ── Audio Clock Generator (PLL — provides MCLK for PCM1808) ─────────────
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

// ── I2S Receiver (ADC) ──────────────────────────────────────────────────
wire signed [31:0] adc_audio;
wire               adc_valid;

i2s_rx u_rx (
    .clk     (mclk),
    .rst_n   (rst_n),
    .bck     (bck),
    .lrck    (lrck),
    .din     (adc_dout),
    .audio_l (adc_audio),
    .valid   (adc_valid)
);

// ── I2S Transmitter (DAC) — direct passthrough ──────────────────────────
i2s_tx u_tx (
    .clk     (mclk),
    .rst_n   (rst_n),
    .bck     (bck),
    .lrck    (lrck),
    .audio_l (adc_audio),
    .audio_r (adc_audio),
    .load    (adc_valid),
    .dout    (dac_din)
);

// ── LED[0]: Heartbeat (~1Hz) ────────────────────────────────────────────
reg [24:0] hb_cnt;
reg hb;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        hb_cnt <= 0;
        hb <= 0;
    end else begin
        if (hb_cnt >= 25'd13_500_000) begin
            hb_cnt <= 0;
            hb <= ~hb;
        end else
            hb_cnt <= hb_cnt + 1;
    end
end

// ── LED[1]: ADC activity ────────────────────────────────────────────────
reg [19:0] act_cnt;
reg act_led;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        act_cnt <= 0;
        act_led <= 0;
    end else begin
        if (adc_valid) begin
            act_led <= 1'b1;
            act_cnt <= 0;
        end else if (act_cnt < 20'd675_000)
            act_cnt <= act_cnt + 1;
        else
            act_led <= 1'b0;
    end
end

// Active-low LEDs
assign led[0] = ~hb;        // heartbeat
assign led[1] = ~act_led;   // ADC activity
assign led[2] = 1'b1;       // off
assign led[3] = 1'b1;       // off
assign led[4] = 1'b1;       // off
assign led[5] = 1'b0;       // on = passthrough indicator

endmodule
