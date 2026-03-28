// ============================================================================
// tangamp_top.v
// Top-Level Module for tangAmp FPGA Tube Amp Emulator
//
// Signal chain: PCM1802 ADC -> 2-stage 12AX7 cascade -> PCM5102 DAC
// Target: Tang Nano 20K (Gowin GW2A, 27MHz)
// ============================================================================

module tangamp_top (
    input  wire clk_27m,
    input  wire btn_rst_n,
    // I2S ADC (PCM1802)
    input  wire adc_dout,
    output wire adc_bck,
    output wire adc_lrck,
    // I2S DAC (PCM5102)
    output wire dac_din,
    output wire dac_bck,
    output wire dac_lrck,
    // Status LEDs
    output wire [5:0] led
);

// ── Reset synchronizer (2-FF) ──────────────────────────────────────────────
reg rst_n_r1, rst_n_sync;

always @(posedge clk_27m or negedge btn_rst_n) begin
    if (!btn_rst_n) begin
        rst_n_r1   <= 1'b0;
        rst_n_sync <= 1'b0;
    end else begin
        rst_n_r1   <= 1'b1;
        rst_n_sync <= rst_n_r1;
    end
end

// ── Audio Clock Generator ──────────────────────────────────────────────────
wire bck, lrck, sample_en;

clk_audio_gen u_clkgen (
    .clk       (clk_27m),
    .rst_n     (rst_n_sync),
    .bck       (bck),
    .lrck      (lrck),
    .sample_en (sample_en)
);

// Share BCK/LRCK between ADC and DAC
assign adc_bck  = bck;
assign adc_lrck = lrck;
assign dac_bck  = bck;
assign dac_lrck = lrck;

// ── I2S Receiver (ADC) ────────────────────────────────────────────────────
wire signed [31:0] adc_audio_l;
wire               adc_valid;

i2s_rx u_rx (
    .clk     (clk_27m),
    .rst_n   (rst_n_sync),
    .bck     (bck),
    .lrck    (lrck),
    .din     (adc_dout),
    .audio_l (adc_audio_l),
    .valid   (adc_valid)
);

// ── Triode Cascade (2 stages) ──────────────────────────────────────────────
wire signed [31:0] amp_out;
wire               amp_valid;

triode_cascade #(
    .NUM_STAGES  (2),
    .ATTEN_SHIFT (2)
) u_cascade (
    .clk       (clk_27m),
    .rst_n     (rst_n_sync),
    .sample_en (adc_valid),
    .audio_in  (adc_audio_l),
    .audio_out (amp_out),
    .out_valid (amp_valid)
);

// ── I2S Transmitter (DAC) ──────────────────────────────────────────────────
i2s_tx u_tx (
    .clk     (clk_27m),
    .rst_n   (rst_n_sync),
    .bck     (bck),
    .lrck    (lrck),
    .audio_l (amp_out),
    .audio_r (amp_out),       // mono: same signal both channels
    .load    (amp_valid),
    .dout    (dac_din)
);

// ── LED Indicators ─────────────────────────────────────────────────────────

// LED[0]: Heartbeat - toggle every ~13.5M clocks (0.5s at 27MHz)
reg [23:0] hb_cnt;
reg        hb_led;

always @(posedge clk_27m or negedge rst_n_sync) begin
    if (!rst_n_sync) begin
        hb_cnt <= 24'd0;
        hb_led <= 1'b0;
    end else begin
        if (hb_cnt == 24'd13_499_999) begin
            hb_cnt <= 24'd0;
            hb_led <= ~hb_led;
        end else begin
            hb_cnt <= hb_cnt + 24'd1;
        end
    end
end

// LED[1]: Sample activity - blink on sample_en
reg [19:0] act_cnt;
reg        act_led;

always @(posedge clk_27m or negedge rst_n_sync) begin
    if (!rst_n_sync) begin
        act_cnt <= 20'd0;
        act_led <= 1'b0;
    end else begin
        if (adc_valid) begin
            act_led <= 1'b1;
            act_cnt <= 20'd0;
        end else if (act_cnt < 20'd675_000) begin
            // Stay lit for ~25ms (675000 / 27M)
            act_cnt <= act_cnt + 20'd1;
        end else begin
            act_led <= 1'b0;
        end
    end
end

// LED[2]: Audio present - |audio_in| > threshold
// Threshold: ~0.01V in Q16.16 = 655
reg audio_present;

always @(posedge clk_27m or negedge rst_n_sync) begin
    if (!rst_n_sync) begin
        audio_present <= 1'b0;
    end else if (adc_valid) begin
        if (adc_audio_l > 32'sd655 || adc_audio_l < -32'sd655)
            audio_present <= 1'b1;
        else
            audio_present <= 1'b0;
    end
end

// Tang Nano 20K LEDs are active-low
assign led[0] = ~hb_led;
assign led[1] = ~act_led;
assign led[2] = ~audio_present;
assign led[3] = 1'b1;  // off
assign led[4] = 1'b1;  // off
assign led[5] = 1'b1;  // off

endmodule
