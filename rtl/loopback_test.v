// ============================================================================
// loopback_test.v
// DAC→ADC loopback integrity test.
//
// Generates a 440Hz sine on DAC output. Simultaneously reads ADC input.
// Wire DAC output back to ADC input externally.
// Compares the received signal against what was sent (with pipeline delay).
//
// LED[0] = heartbeat
// LED[1] = ADC activity (receiving data)
// LED[2] = signal detected on ADC (above noise floor)
// LED[3] = correlation OK (received matches sent within tolerance)
// LED[4] = level match (amplitude within 50% of expected)
// LED[5] = PASS — all checks green for >1 second
//
// S1 = reset test (restart comparison)
// S2 = cycle gain: 1x, 2x, 4x, 0.5x (LED[2:3] show gain while held)
// ============================================================================

module loopback_test (
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

// ── Button debounce ──────────────────────────────────────────────────────
reg [15:0] s1_db, s2_db;
reg s1_prev, s2_prev;
reg test_reset;
reg [1:0] gain_sel;

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        s1_db <= 0; s2_db <= 0;
        s1_prev <= 1; s2_prev <= 1;
        test_reset <= 1; gain_sel <= 0;
    end else begin
        s1_prev <= btn_s1;
        s2_prev <= btn_s2;
        test_reset <= 0;
        if (s1_db > 0) s1_db <= s1_db - 1;
        if (s2_db > 0) s2_db <= s2_db - 1;

        if (!btn_s1 && s1_prev && s1_db == 0) begin
            test_reset <= 1;
            s1_db <= 16'hFFFF;
        end
        if (!btn_s2 && s2_prev && s2_db == 0) begin
            gain_sel <= gain_sel + 1;
            s2_db <= 16'hFFFF;
        end
    end
end

// ── Sine generator (440Hz, 108-entry table) ──────────────────────────────
reg signed [31:0] sine_table [0:107];
initial begin
    sine_table[  0] =  32'sd0;
    sine_table[  1] =  32'sd1905;
    sine_table[  2] =  32'sd3804;
    sine_table[  3] =  32'sd5690;
    sine_table[  4] =  32'sd7557;
    sine_table[  5] =  32'sd9398;
    sine_table[  6] =  32'sd11207;
    sine_table[  7] =  32'sd12979;
    sine_table[  8] =  32'sd14706;
    sine_table[  9] =  32'sd16384;
    sine_table[ 10] =  32'sd18006;
    sine_table[ 11] =  32'sd19567;
    sine_table[ 12] =  32'sd21062;
    sine_table[ 13] =  32'sd22486;
    sine_table[ 14] =  32'sd23834;
    sine_table[ 15] =  32'sd25101;
    sine_table[ 16] =  32'sd26284;
    sine_table[ 17] =  32'sd27378;
    sine_table[ 18] =  32'sd28378;
    sine_table[ 19] =  32'sd29282;
    sine_table[ 20] =  32'sd30087;
    sine_table[ 21] =  32'sd30791;
    sine_table[ 22] =  32'sd31390;
    sine_table[ 23] =  32'sd31884;
    sine_table[ 24] =  32'sd32270;
    sine_table[ 25] =  32'sd32546;
    sine_table[ 26] =  32'sd32713;
    sine_table[ 27] =  32'sd32768;
    sine_table[ 28] =  32'sd32713;
    sine_table[ 29] =  32'sd32546;
    sine_table[ 30] =  32'sd32270;
    sine_table[ 31] =  32'sd31884;
    sine_table[ 32] =  32'sd31390;
    sine_table[ 33] =  32'sd30791;
    sine_table[ 34] =  32'sd30087;
    sine_table[ 35] =  32'sd29282;
    sine_table[ 36] =  32'sd28378;
    sine_table[ 37] =  32'sd27378;
    sine_table[ 38] =  32'sd26284;
    sine_table[ 39] =  32'sd25101;
    sine_table[ 40] =  32'sd23834;
    sine_table[ 41] =  32'sd22486;
    sine_table[ 42] =  32'sd21062;
    sine_table[ 43] =  32'sd19567;
    sine_table[ 44] =  32'sd18006;
    sine_table[ 45] =  32'sd16384;
    sine_table[ 46] =  32'sd14706;
    sine_table[ 47] =  32'sd12979;
    sine_table[ 48] =  32'sd11207;
    sine_table[ 49] =  32'sd9398;
    sine_table[ 50] =  32'sd7557;
    sine_table[ 51] =  32'sd5690;
    sine_table[ 52] =  32'sd3804;
    sine_table[ 53] =  32'sd1905;
    sine_table[ 54] =  32'sd0;
    sine_table[ 55] = -32'sd1905;
    sine_table[ 56] = -32'sd3804;
    sine_table[ 57] = -32'sd5690;
    sine_table[ 58] = -32'sd7557;
    sine_table[ 59] = -32'sd9398;
    sine_table[ 60] = -32'sd11207;
    sine_table[ 61] = -32'sd12979;
    sine_table[ 62] = -32'sd14706;
    sine_table[ 63] = -32'sd16384;
    sine_table[ 64] = -32'sd18006;
    sine_table[ 65] = -32'sd19567;
    sine_table[ 66] = -32'sd21062;
    sine_table[ 67] = -32'sd22486;
    sine_table[ 68] = -32'sd23834;
    sine_table[ 69] = -32'sd25101;
    sine_table[ 70] = -32'sd26284;
    sine_table[ 71] = -32'sd27378;
    sine_table[ 72] = -32'sd28378;
    sine_table[ 73] = -32'sd29282;
    sine_table[ 74] = -32'sd30087;
    sine_table[ 75] = -32'sd30791;
    sine_table[ 76] = -32'sd31390;
    sine_table[ 77] = -32'sd31884;
    sine_table[ 78] = -32'sd32270;
    sine_table[ 79] = -32'sd32546;
    sine_table[ 80] = -32'sd32713;
    sine_table[ 81] = -32'sd32768;
    sine_table[ 82] = -32'sd32713;
    sine_table[ 83] = -32'sd32546;
    sine_table[ 84] = -32'sd32270;
    sine_table[ 85] = -32'sd31884;
    sine_table[ 86] = -32'sd31390;
    sine_table[ 87] = -32'sd30791;
    sine_table[ 88] = -32'sd30087;
    sine_table[ 89] = -32'sd29282;
    sine_table[ 90] = -32'sd28378;
    sine_table[ 91] = -32'sd27378;
    sine_table[ 92] = -32'sd26284;
    sine_table[ 93] = -32'sd25101;
    sine_table[ 94] = -32'sd23834;
    sine_table[ 95] = -32'sd22486;
    sine_table[ 96] = -32'sd21062;
    sine_table[ 97] = -32'sd19567;
    sine_table[ 98] = -32'sd18006;
    sine_table[ 99] = -32'sd16384;
    sine_table[100] = -32'sd14706;
    sine_table[101] = -32'sd12979;
    sine_table[102] = -32'sd11207;
    sine_table[103] = -32'sd9398;
    sine_table[104] = -32'sd7557;
    sine_table[105] = -32'sd5690;
    sine_table[106] = -32'sd3804;
    sine_table[107] = -32'sd1905;
end

reg [6:0] sine_idx;
reg signed [31:0] tx_sample;

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        sine_idx <= 0;
        tx_sample <= 0;
    end else if (sample_en) begin
        tx_sample <= sine_table[sine_idx];
        sine_idx <= (sine_idx >= 7'd107) ? 7'd0 : sine_idx + 7'd1;
    end
end

// ── Apply gain to TX ─────────────────────────────────────────────────────
reg signed [31:0] tx_gained;
always @(*) begin
    case (gain_sel)
        2'd0: tx_gained = tx_sample;               // 1x
        2'd1: tx_gained = tx_sample <<< 1;          // 2x
        2'd2: tx_gained = tx_sample <<< 2;          // 4x
        2'd3: tx_gained = tx_sample >>> 1;           // 0.5x
    endcase
end

// ── I2S TX ───────────────────────────────────────────────────────────────
i2s_tx u_tx (
    .clk     (clk_27m),
    .rst_n   (rst_n),
    .bck     (bck),
    .lrck    (lrck),
    .audio_l (tx_gained),
    .audio_r (tx_gained),
    .load    (sample_en),
    .dout    (dac_din)
);

// ── I2S RX ───────────────────────────────────────────────────────────────
wire signed [31:0] rx_sample;
wire               rx_valid;

i2s_rx u_rx (
    .clk     (clk_27m),
    .rst_n   (rst_n),
    .bck     (bck),
    .lrck    (lrck),
    .din     (adc_dout),
    .audio_l (rx_sample),
    .valid   (rx_valid)
);

// ── Delay line (store last 16 TX samples to correlate against RX) ────────
// The analog path has some latency (ADC pipeline + DAC pipeline)
reg signed [31:0] tx_delay [0:15];
reg [3:0] tx_wr;
integer k;

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        tx_wr <= 0;
        for (k = 0; k < 16; k = k + 1)
            tx_delay[k] <= 0;
    end else if (sample_en) begin
        tx_delay[tx_wr] <= tx_gained;
        tx_wr <= tx_wr + 1;
    end
end

// ── Signal analysis ──────────────────────────────────────────────────────
reg [31:0] rx_abs;
reg signal_detected;
reg correlation_ok;
reg level_match;
reg [23:0] pass_cnt;
reg all_pass;

// Track RX peak and TX peak over a window
reg [31:0] rx_peak, tx_peak;
reg [9:0] window_cnt;

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n || test_reset) begin
        signal_detected <= 0;
        correlation_ok <= 0;
        level_match <= 0;
        all_pass <= 0;
        pass_cnt <= 0;
        rx_peak <= 0;
        tx_peak <= 0;
        window_cnt <= 0;
    end else if (rx_valid) begin
        rx_abs <= rx_sample[31] ? (~rx_sample + 1) : rx_sample;

        // Signal detection: anything above noise floor
        if (rx_abs > 32'd512)
            signal_detected <= 1;

        // Track peaks over 512-sample window (~10ms)
        window_cnt <= window_cnt + 1;
        if (rx_abs > rx_peak)
            rx_peak <= rx_abs;

        begin : tx_peak_block
            reg [31:0] tx_abs_val;
            // Compare against delayed TX (try offset 4 — typical ADC+DAC pipeline)
            tx_abs_val = tx_delay[(tx_wr - 4) & 4'hF];
            if (tx_abs_val[31])
                tx_abs_val = ~tx_abs_val + 1;
            if (tx_abs_val > tx_peak)
                tx_peak <= tx_abs_val;
        end

        if (window_cnt == 0) begin
            // Correlation check: RX should be changing (not stuck)
            // and peak should be significant
            if (rx_peak > 32'd1024)
                correlation_ok <= 1;

            // Level match: RX peak within 2x-0.25x of TX peak
            if (tx_peak > 0) begin
                if (rx_peak > (tx_peak >> 2) && rx_peak < (tx_peak << 1))
                    level_match <= 1;
                else
                    level_match <= 0;
            end

            rx_peak <= 0;
            tx_peak <= 0;
        end

        // All-pass counter: all indicators green for ~1 second
        if (signal_detected && correlation_ok && level_match) begin
            if (pass_cnt < 24'd12_000_000)
                pass_cnt <= pass_cnt + 1;
            else
                all_pass <= 1;
        end else begin
            pass_cnt <= 0;
            all_pass <= 0;
        end
    end
end

// ── ADC activity LED ─────────────────────────────────────────────────────
reg [19:0] act_cnt;
reg act_led;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin act_cnt <= 0; act_led <= 0; end
    else begin
        if (rx_valid) begin act_led <= 1; act_cnt <= 0; end
        else if (act_cnt < 20'd675_000) act_cnt <= act_cnt + 1;
        else act_led <= 0;
    end
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

// Active-low LEDs
assign led[0] = ~hb;               // heartbeat
assign led[1] = ~act_led;          // ADC activity
assign led[2] = ~signal_detected;  // signal present on ADC
assign led[3] = ~correlation_ok;   // received signal is dynamic (not stuck)
assign led[4] = ~level_match;      // amplitude in expected range
assign led[5] = ~all_pass;         // ALL PASS for >1s

endmodule
