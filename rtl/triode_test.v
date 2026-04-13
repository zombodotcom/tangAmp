// ============================================================================
// triode_test.v
// Triode engine isolation test — ADC -> gain -> triode_engine -> DAC
// No tone stack, no cabinet IR, no output transformer.
// Hear the raw tube distortion character.
//
// S1 = cycle input gain: 1x, 2x, 4x, 8x (LED[2:3] show binary)
// S2 = toggle 1-stage vs 2-stage cascade (LED[4])
//
// LED[0] = heartbeat (1Hz)
// LED[1] = ADC activity
// LED[5] = VU meter (output level indicator)
// ============================================================================

module triode_test (
    input  wire clk_27m,
    input  wire btn_s1,
    input  wire btn_s2,

    output wire adc_bck,
    output wire adc_lrck,
    input  wire adc_dout,
    output wire dac_din,

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

// ── Button Debounce ─────────────────────────────────────────────────────
reg [15:0] s1_db, s2_db;
reg s1_prev, s2_prev;
reg [1:0] gain_sel;       // 0=1x, 1=2x, 2=4x, 3=8x
reg stage_sel;             // 0=1-stage, 1=2-stage

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        s1_db <= 0; s2_db <= 0;
        s1_prev <= 1; s2_prev <= 1;
        gain_sel <= 2'd2;  // default 4x (<<<2)
        stage_sel <= 1;    // default 2-stage
    end else begin
        s1_prev <= btn_s1;
        s2_prev <= btn_s2;
        if (s1_db > 0) s1_db <= s1_db - 1;
        if (s2_db > 0) s2_db <= s2_db - 1;

        if (!btn_s1 && s1_prev && s1_db == 0) begin
            gain_sel <= gain_sel + 1;
            s1_db <= 16'hFFFF;
        end
        if (!btn_s2 && s2_prev && s2_db == 0) begin
            stage_sel <= ~stage_sel;
            s2_db <= 16'hFFFF;
        end
    end
end

// ── Input Gain ──────────────────────────────────────────────────────────
reg signed [31:0] gained_in;

always @(*) begin
    case (gain_sel)
        2'd0: gained_in = adc_audio;
        2'd1: gained_in = adc_audio <<< 1;
        2'd2: gained_in = adc_audio <<< 2;
        2'd3: gained_in = adc_audio <<< 3;
    endcase
end

// ── Triode Engine (2-stage, no power amp) ───────────────────────────────
// We instantiate two engines: 1-stage and 2-stage, mux output.
// This avoids runtime parameter changes.

wire signed [31:0] triode_out_1s, triode_out_2s;
wire triode_valid_1s, triode_valid_2s;

triode_engine #(
    .NUM_STAGES (1),
    .POWER_AMP  (0)
) triode_1s (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .sample_en (adc_valid & ~stage_sel),
    .audio_in  (gained_in),
    .audio_out (triode_out_1s),
    .out_valid (triode_valid_1s)
);

triode_engine #(
    .NUM_STAGES (2),
    .POWER_AMP  (0)
) triode_2s (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .sample_en (adc_valid & stage_sel),
    .audio_in  (gained_in),
    .audio_out (triode_out_2s),
    .out_valid (triode_valid_2s)
);

wire signed [31:0] triode_out   = stage_sel ? triode_out_2s : triode_out_1s;
wire               triode_valid = stage_sel ? triode_valid_2s : triode_valid_1s;

// ── Output scaling ──────────────────────────────────────────────────────
wire signed [31:0] dac_audio = triode_out >>> 2;

// ── I2S Transmitter (DAC) ───────────────────────────────────────────────
i2s_tx u_tx (
    .clk     (clk_27m),
    .rst_n   (rst_n),
    .bck     (bck),
    .lrck    (lrck),
    .audio_l (dac_audio),
    .audio_r (dac_audio),
    .load    (triode_valid),
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

// ── LED[5]: VU meter (output level) ─────────────────────────────────────
reg vu_led;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n)
        vu_led <= 0;
    else if (triode_valid) begin
        if (triode_out[31])
            vu_led <= (-triode_out > 32'd131072);
        else
            vu_led <= (triode_out > 32'd131072);
    end
end

// Active-low LEDs
assign led[0] = ~hb;
assign led[1] = ~act_led;
assign led[2] = ~gain_sel[0];
assign led[3] = ~gain_sel[1];
assign led[4] = ~stage_sel;
assign led[5] = ~vu_led;

endmodule
