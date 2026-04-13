// ============================================================================
// noisegate_test.v
// Noise gate test — ADC -> noise_gate -> DAC
// Tune the noise gate threshold before integrating into the full chain.
//
// S1 = threshold up
// S2 = threshold down
// 8 threshold levels (mapped to noise_gate threshold byte)
//
// LED[0] = heartbeat (1Hz)
// LED[1] = ADC activity
// LED[2:4] = threshold level (binary, 0-7)
// LED[5] = gate open indicator (lit when signal passes through)
// ============================================================================

module noisegate_test (
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

// ── Buttons ─────────────────────────────────────────────────────────────
reg [15:0] s1_db, s2_db;
reg s1_prev, s2_prev;
reg [2:0] thresh_level;  // 0-7

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        s1_db <= 0; s2_db <= 0;
        s1_prev <= 1; s2_prev <= 1;
        thresh_level <= 3'd2;  // default mid-low
    end else begin
        s1_prev <= btn_s1;
        s2_prev <= btn_s2;
        if (s1_db > 0) s1_db <= s1_db - 1;
        if (s2_db > 0) s2_db <= s2_db - 1;

        // S1 = threshold up
        if (!btn_s1 && s1_prev && s1_db == 0) begin
            if (thresh_level < 3'd7) thresh_level <= thresh_level + 1;
            s1_db <= 16'hFFFF;
        end
        // S2 = threshold down
        if (!btn_s2 && s2_prev && s2_db == 0) begin
            if (thresh_level > 3'd0) thresh_level <= thresh_level - 1;
            s2_db <= 16'hFFFF;
        end
    end
end

// ── Threshold mapping ───────────────────────────────────────────────────
// Map 0-7 to noise_gate threshold byte values:
// 0=off(0), 1=2, 2=4, 3=8, 4=16, 5=32, 6=64, 7=128
reg [7:0] threshold_val;

always @(*) begin
    case (thresh_level)
        3'd0: threshold_val = 8'd0;
        3'd1: threshold_val = 8'd2;
        3'd2: threshold_val = 8'd4;
        3'd3: threshold_val = 8'd8;
        3'd4: threshold_val = 8'd16;
        3'd5: threshold_val = 8'd32;
        3'd6: threshold_val = 8'd64;
        3'd7: threshold_val = 8'd128;
    endcase
end

// ── Noise Gate ──────────────────────────────────────────────────────────
wire signed [31:0] gated_audio;

noise_gate #(
    .FP_FRAC  (16),
    .FP_WIDTH (32)
) ngate (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .sample_en (adc_valid),
    .audio_in  (adc_audio),
    .audio_out (gated_audio),
    .threshold (threshold_val)
);

// ── I2S Transmitter (DAC) ───────────────────────────────────────────────
i2s_tx u_tx (
    .clk     (clk_27m),
    .rst_n   (rst_n),
    .bck     (bck),
    .lrck    (lrck),
    .audio_l (gated_audio),
    .audio_r (gated_audio),
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

// ── LED[5]: Gate open indicator ─────────────────────────────────────────
// Compare gated output to input — if gate is open, output ~= input
// Simple approach: if |gated_audio| > small threshold, gate is open
reg gate_open;
reg [19:0] gate_cnt;

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin gate_open <= 0; gate_cnt <= 0; end
    else begin
        if (adc_valid) begin
            if (gated_audio > 32'sd256 || gated_audio < -32'sd256) begin
                gate_open <= 1;
                gate_cnt <= 0;
            end
        end
        if (gate_cnt < 20'd675_000)
            gate_cnt <= gate_cnt + 1;
        else
            gate_open <= 0;
    end
end

// Active-low LEDs
assign led[0] = ~hb;
assign led[1] = ~act_led;
assign led[2] = ~thresh_level[0];
assign led[3] = ~thresh_level[1];
assign led[4] = ~thresh_level[2];
assign led[5] = ~gate_open;

endmodule
