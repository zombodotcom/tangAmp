// ============================================================================
// tangamp_top_lite.v
// Lite version of tangAmp — fits in Tang Nano 20K without SD card or power amp.
//
// Signal chain:
//   PCM1802 ADC -> gain -> noise_gate -> NFB subtract -> [2x oversample] ->
//   triode_engine (2-stage 12AX7, no power amp) -> [downsample] ->
//   tone_stack_iir -> output_transformer -> cabinet_fir -> PCM5102 DAC
//
// S2 = bypass toggle (ADC -> DAC direct)
// LED[0] = heartbeat, LED[1] = ADC activity, LED[2:5] = VU meter
// LED[5] = bypass mode indicator
// ============================================================================

module tangamp_top_lite (
    input  wire clk_27m,       // 27MHz crystal, pin 4
    input  wire btn_s1,        // Button S1, pin 88 (unused for now)
    input  wire btn_s2,        // Button S2, pin 87 (bypass toggle)

    output wire adc_bck,       // I2S bit clock  (pin 76)
    output wire adc_lrck,      // I2S word clock (pin 77)
    input  wire adc_dout,      // ADC data (pin 48)
    output wire dac_din,
    output wire mclk_out,    // 18MHz MCLK to PCM1808 SCK (pin 75)       // DAC data (pin 49)

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
    end else begin
        rst_n <= 1'b1;
    end
end

// ── Bypass Mode Toggle (S2 debounce) ─────────────────────────────────────
reg [15:0] s2_debounce;
reg s2_prev;
reg bypass_mode;

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        s2_debounce <= 0;
        s2_prev <= 1;
        bypass_mode <= 0;
    end else begin
        s2_prev <= btn_s2;
        if (!btn_s2 && s2_prev) begin
            if (s2_debounce == 0) begin
                bypass_mode <= ~bypass_mode;
                s2_debounce <= 16'hFFFF;
            end
        end
        if (s2_debounce > 0)
            s2_debounce <= s2_debounce - 1;
    end
end

// ── Audio Clock Generator ────────────────────────────────────────────────
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

// ── Input gain (2x) ─────────────────────────────────────────────────────
localparam [9:0] INPUT_GAIN = 10'd512;
wire signed [31:0] scaled_in;
wire signed [63:0] gain_tmp = $signed(adc_audio) * $signed({1'b0, INPUT_GAIN});
assign scaled_in = gain_tmp[41:10];

// ── Noise Gate ───────────────────────────────────────────────────────────
wire signed [31:0] gated_in;

noise_gate #(
    .FP_FRAC  (16),
    .FP_WIDTH (32)
) ngate (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .sample_en (adc_valid),
    .audio_in  (scaled_in),
    .audio_out (gated_in),
    .threshold (8'h08)
);

// ── Negative Feedback Subtraction ────────────────────────────────────────
wire signed [31:0] nfb_signal;
reg  signed [31:0] triode_input;

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n)
        triode_input <= 0;
    else if (adc_valid)
        triode_input <= gated_in - nfb_signal;
end

// ── 2x Oversampling + Triode Engine (preamp only, no power amp) ─────────
wire        up_valid_a, up_valid_b;
wire signed [31:0] audio_up;

reg         triode_start;
reg  signed [31:0] triode_in;
wire signed [31:0] triode_out;
wire        triode_valid;

reg  signed [31:0] proc_a_sample;
reg         proc_a_valid;
reg  signed [31:0] proc_b_sample;
reg         proc_b_valid;

wire signed [31:0] audio_down;
wire        down_valid;

oversample_2x oversamp (
    .clk            (clk_27m),
    .rst_n          (rst_n),
    .sample_en_48k  (adc_valid),
    .audio_in       (triode_input),
    .up_valid_a     (up_valid_a),
    .up_valid_b     (up_valid_b),
    .audio_up       (audio_up),
    .processed_a    (proc_a_sample),
    .proc_valid_a   (proc_a_valid),
    .processed_b    (proc_b_sample),
    .proc_valid_b   (proc_b_valid),
    .audio_down     (audio_down),
    .down_valid     (down_valid)
);

triode_engine #(
    .NUM_STAGES  (2),
    .POWER_AMP   (0)       // No power amp — saves ~5000 LUTs
) triode (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .sample_en (triode_start),
    .audio_in  (triode_in),
    .audio_out (triode_out),
    .out_valid (triode_valid)
);

// ── Oversampling Controller ──────────────────────────────────────────────
localparam OS_IDLE       = 3'd0;
localparam OS_WAIT_TRI_A = 3'd1;
localparam OS_START_B    = 3'd2;
localparam OS_WAIT_TRI_B = 3'd3;

reg [2:0] os_state;
reg signed [31:0] up_b_held;
reg signed [31:0] triode_a_held;

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        os_state      <= OS_IDLE;
        triode_start  <= 1'b0;
        triode_in     <= 0;
        proc_a_valid  <= 1'b0;
        proc_b_valid  <= 1'b0;
        proc_a_sample <= 0;
        proc_b_sample <= 0;
        up_b_held     <= 0;
        triode_a_held <= 0;
    end else begin
        triode_start <= 1'b0;
        proc_a_valid <= 1'b0;
        proc_b_valid <= 1'b0;

        case (os_state)
        OS_IDLE: begin
            if (up_valid_a) begin
                triode_in    <= audio_up;
                triode_start <= 1'b1;
                os_state     <= OS_WAIT_TRI_A;
            end
        end
        OS_WAIT_TRI_A: begin
            if (up_valid_b)
                up_b_held <= audio_up;
            if (triode_valid) begin
                triode_a_held <= triode_out;
                os_state      <= OS_START_B;
            end
        end
        OS_START_B: begin
            triode_in    <= up_b_held;
            triode_start <= 1'b1;
            os_state     <= OS_WAIT_TRI_B;
        end
        OS_WAIT_TRI_B: begin
            if (triode_valid) begin
                proc_a_sample <= triode_a_held;
                proc_a_valid  <= 1'b1;
                proc_b_sample <= triode_out;
                proc_b_valid  <= 1'b1;
                os_state      <= OS_IDLE;
            end
        end
        default: os_state <= OS_IDLE;
        endcase
    end
end

// ── Tone Stack (3-band biquad EQ) ────────────────────────────────────────
wire signed [31:0] tone_out;
wire tone_valid;

tone_stack_iir tone (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .preset    (3'd0),       // Flat preset
    .sample_en (down_valid),
    .audio_in  (audio_down),
    .audio_out (tone_out),
    .out_valid (tone_valid)
);

// ── Output Transformer ───────────────────────────────────────────────────
wire signed [31:0] xf_out;
wire xf_valid;

output_transformer xformer (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .sample_en (tone_valid),
    .audio_in  (tone_out),
    .audio_out (xf_out),
    .out_valid (xf_valid)
);

// ── Cabinet IR (129-tap FIR, loaded from hex) ────────────────────────────
wire signed [31:0] cab_out;
wire cab_valid;

cabinet_fir #(
    .N_TAPS (129)
) cabinet (
    .clk         (clk_27m),
    .rst_n       (rst_n),
    .sample_en   (xf_valid),
    .audio_in    (xf_out),
    .audio_out   (cab_out),
    .out_valid   (cab_valid),
    .tap_wr_en   (1'b0),
    .tap_wr_addr (8'd0),
    .tap_wr_data (16'd0)
);

// ── Negative Feedback Register ───────────────────────────────────────────
nfb_register #(
    .NFB_SHIFT (3)
) nfb (
    .clk        (clk_27m),
    .rst_n      (rst_n),
    .fb_in      (cab_out),
    .fb_valid   (cab_valid),
    .nfb_signal (nfb_signal)
);

// ── DAC Output ───────────────────────────────────────────────────────────
wire signed [31:0] dac_audio = bypass_mode ? (adc_audio <<< 2) : (cab_out >>> 2);

i2s_tx u_tx (
    .clk     (clk_27m),
    .rst_n   (rst_n),
    .bck     (bck),
    .lrck    (lrck),
    .audio_l (dac_audio),
    .audio_r (dac_audio),
    .load    (bypass_mode ? adc_valid : cab_valid),
    .dout    (dac_din)
);

// ── LEDs ─────────────────────────────────────────────────────────────────
reg [24:0] hb_cnt;
reg hb;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin hb_cnt <= 0; hb <= 0; end
    else begin
        if (hb_cnt >= 25'd13_500_000) begin hb_cnt <= 0; hb <= ~hb; end
        else hb_cnt <= hb_cnt + 1;
    end
end

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

reg [31:0] abs_out;
reg [3:0] vu;
always @(posedge clk_27m) begin
    if (cab_valid) begin
        abs_out <= cab_out[31] ? -cab_out : cab_out;
        vu[0] <= (abs_out > 32'd65536);
        vu[1] <= (abs_out > 32'd196608);
        vu[2] <= (abs_out > 32'd524288);
        vu[3] <= (abs_out > 32'd983040);
    end
end

assign led[0] = ~hb;
assign led[1] = ~act_led;
assign led[2] = ~vu[0];
assign led[3] = ~vu[1];
assign led[4] = ~vu[2];
assign led[5] = ~bypass_mode;

endmodule
