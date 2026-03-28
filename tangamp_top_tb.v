// ============================================================================
// tangamp_top_tb.v
// Full-Chain Testbench for tangAmp Top-Level
//
// Generates an I2S bitstream (440Hz/0.5V sine as 24-bit) on adc_dout,
// captures dac_din output, decodes back to PCM, and writes to file.
// ============================================================================

`timescale 1ns/1ps

module tangamp_top_tb;

// ── Clock and Reset ─────────────────────────────────────────────────────────
reg clk_27m;
reg btn_rst_n;
localparam CLK_PERIOD = 37;  // ~27MHz

initial clk_27m = 0;
always #(CLK_PERIOD/2) clk_27m = ~clk_27m;

// ── DUT ─────────────────────────────────────────────────────────────────────
reg  adc_dout_reg;
wire adc_bck, adc_lrck;
wire dac_din, dac_bck, dac_lrck;
wire [5:0] led;

tangamp_top dut (
    .clk_27m    (clk_27m),
    .btn_rst_n  (btn_rst_n),
    .adc_dout   (adc_dout_reg),
    .adc_bck    (adc_bck),
    .adc_lrck   (adc_lrck),
    .dac_din    (dac_din),
    .dac_bck    (dac_bck),
    .dac_lrck   (dac_lrck),
    .led        (led)
);

// ── Sine Wave Table ─────────────────────────────────────────────────────────
// 440Hz at ~46.875kHz sample rate: ~106.5 samples per period
// Using 107 samples. 0.5V amplitude as 24-bit signed.
// 24-bit full scale = 8388607. 0.5V / ~3.3V_ref... but we treat as
// direct Q16.16 input. 0.5V in Q16.16 = 32768. As 24-bit left-justified
// in the I2S frame: we send the Q16.16 value's top 24 bits = value[31:8].
// So 0.5 * 65536 = 32768 -> [31:8] = 128 as 24-bit -> too small.
// Better: generate 24-bit PCM directly, then on RX side it maps to Q16.16.
// 0.5V amplitude in 24-bit: 0.5 * 2^23 = 4194304
localparam SINE_LEN = 107;  // ~46875/440

reg signed [23:0] sine_table [0:SINE_LEN-1];
integer i;
real two_pi = 6.2831853;

initial begin
    for (i = 0; i < SINE_LEN; i = i + 1) begin
        // 0.5V amplitude: half of full-scale 24-bit
        sine_table[i] = $rtoi($sin(two_pi * i * 1.0 / SINE_LEN) * 4194304.0);
    end
end

// ── I2S Transmitter (feed ADC data to DUT) ─────────────────────────────────
// We slave to the DUT's generated BCK and LRCK signals.
// Shift out 24-bit data MSB first with 1 BCK delay after LRCK transition.

reg adc_bck_d, adc_lrck_d;
wire adc_bck_fall = ~adc_bck & adc_bck_d;
wire adc_bck_rise = adc_bck & ~adc_bck_d;
wire adc_lrck_fall = ~adc_lrck & adc_lrck_d;
wire adc_lrck_rise = adc_lrck & ~adc_lrck_d;

always @(posedge clk_27m) begin
    adc_bck_d  <= adc_bck;
    adc_lrck_d <= adc_lrck;
end

integer sine_idx;
reg [31:0] tx_sr;         // 32-bit shift register for I2S frame
reg [5:0]  tx_bit_cnt;
reg        tx_active;
reg        tx_skip_first;

// Sample counter for DC settling
integer tx_sample_count;
localparam DC_SETTLE = 500;

initial begin
    sine_idx = 0;
    tx_sample_count = 0;
    adc_dout_reg = 0;
    tx_sr = 0;
    tx_bit_cnt = 0;
    tx_active = 0;
    tx_skip_first = 0;
end

always @(posedge clk_27m) begin
    if (!btn_rst_n) begin
        tx_sr <= 32'd0;
        tx_bit_cnt <= 6'd0;
        tx_active <= 1'b0;
        tx_skip_first <= 1'b0;
        adc_dout_reg <= 1'b0;
    end else begin
        // On LRCK falling edge: left channel starts, load new sample
        if (adc_lrck_fall) begin
            if (tx_sample_count < DC_SETTLE) begin
                tx_sr <= 32'd0;  // silence during settling
                tx_sample_count <= tx_sample_count + 1;
            end else begin
                // Load 24-bit sample left-justified in 32-bit frame
                tx_sr <= {sine_table[sine_idx], 8'd0};
                if (sine_idx >= SINE_LEN - 1)
                    sine_idx <= 0;
                else
                    sine_idx <= sine_idx + 1;
                tx_sample_count <= tx_sample_count + 1;
            end
            tx_bit_cnt <= 6'd0;
            tx_active <= 1'b1;
            tx_skip_first <= 1'b1;
        end else if (adc_lrck_rise) begin
            // Right channel: send zeros (mono input)
            tx_sr <= 32'd0;
            tx_bit_cnt <= 6'd0;
            tx_active <= 1'b1;
            tx_skip_first <= 1'b1;
        end

        // Shift out on BCK falling edge (DAC samples on rising)
        if (adc_bck_fall && tx_active) begin
            if (tx_skip_first) begin
                // 1-BCK delay per I2S spec
                tx_skip_first <= 1'b0;
                adc_dout_reg <= 1'b0;
            end else if (tx_bit_cnt < 6'd32) begin
                adc_dout_reg <= tx_sr[31];
                tx_sr <= {tx_sr[30:0], 1'b0};
                tx_bit_cnt <= tx_bit_cnt + 6'd1;
            end else begin
                adc_dout_reg <= 1'b0;
            end
        end
    end
end

// ── DAC Output Decoder ─────────────────────────────────────────────────────
// Capture dac_din on BCK rising edges, decode 24-bit I2S

reg dac_bck_d, dac_lrck_d;
wire dac_bck_rise_w = dac_bck & ~dac_bck_d;
wire dac_lrck_fall_w = ~dac_lrck & dac_lrck_d;
wire dac_lrck_rise_w = dac_lrck & ~dac_lrck_d;

always @(posedge clk_27m) begin
    dac_bck_d  <= dac_bck;
    dac_lrck_d <= dac_lrck;
end

reg [23:0] rx_sr;
reg [5:0]  rx_bit_cnt;
reg        rx_skip_first;
reg        rx_capturing;
reg signed [31:0] dac_sample_l;  // decoded left channel (Q16.16)
reg        dac_sample_valid;

initial begin
    rx_sr = 0;
    rx_bit_cnt = 0;
    rx_skip_first = 0;
    rx_capturing = 0;
    dac_sample_l = 0;
    dac_sample_valid = 0;
end

always @(posedge clk_27m) begin
    if (!btn_rst_n) begin
        rx_sr <= 24'd0;
        rx_bit_cnt <= 6'd0;
        rx_skip_first <= 1'b0;
        rx_capturing <= 1'b0;
        dac_sample_valid <= 1'b0;
    end else begin
        dac_sample_valid <= 1'b0;

        // LRCK falling = left channel start
        if (dac_lrck_fall_w) begin
            rx_sr <= 24'd0;
            rx_bit_cnt <= 6'd0;
            rx_skip_first <= 1'b1;
            rx_capturing <= 1'b1;
        end else if (dac_lrck_rise_w) begin
            // Left channel done, output result
            if (rx_capturing && rx_bit_cnt >= 6'd24) begin
                // Convert 24-bit to Q16.16: sign-extend, shift left 8
                dac_sample_l <= {{8{rx_sr[23]}}, rx_sr} <<< 8;
                dac_sample_valid <= 1'b1;
            end
            rx_capturing <= 1'b0;
        end

        // Capture on BCK rising edge
        if (dac_bck_rise_w && rx_capturing) begin
            if (rx_skip_first) begin
                rx_skip_first <= 1'b0;
            end else if (rx_bit_cnt < 6'd24) begin
                rx_sr <= {rx_sr[22:0], dac_din};
                rx_bit_cnt <= rx_bit_cnt + 6'd1;
            end
        end
    end
end

// ── Output Capture ──────────────────────────────────────────────────────────
integer fd;
integer out_sample_count;

// Track what we sent (delayed to match pipeline)
// Store the input sample corresponding to each output
// We use a simple counter since the pipeline has fixed latency
reg signed [31:0] input_log [0:65535];
integer in_log_wr;

initial begin
    fd = $fopen("top_tb_output.txt", "w");
    out_sample_count = 0;
    in_log_wr = 0;
end

// Log input samples (Q16.16 version of what we sent)
always @(posedge clk_27m) begin
    if (adc_lrck_fall && btn_rst_n) begin
        if (tx_sample_count >= DC_SETTLE && sine_idx > 0) begin
            // Convert 24-bit sine to Q16.16 for logging
            input_log[in_log_wr] <= {{8{sine_table[sine_idx > 0 ? sine_idx - 1 : SINE_LEN-1][23]}},
                                      sine_table[sine_idx > 0 ? sine_idx - 1 : SINE_LEN-1]} <<< 8;
        end else begin
            input_log[in_log_wr] <= 32'sd0;
        end
        in_log_wr <= in_log_wr + 1;
    end
end

// Log output samples
always @(posedge clk_27m) begin
    if (dac_sample_valid) begin
        // Format: sample_number input output (Q16.16 integers)
        // Use out_sample_count as index into input_log (rough alignment)
        $fdisplay(fd, "%0d %0d %0d", out_sample_count,
                  (out_sample_count < in_log_wr) ? input_log[out_sample_count] : 0,
                  dac_sample_l);
        out_sample_count = out_sample_count + 1;
    end
end

// ── Run Simulation ──────────────────────────────────────────────────────────
// 500 settling + 4800 audio samples at ~46875Hz
// Each sample = 64 BCK * 9 sys_clk = 576 sys_clk
// Total samples needed: 5300
// Total clocks: 5300 * 576 = 3,052,800
// But triode cascade adds latency, so add margin
// Total time: 5300 * 576 * 37ns = ~113ms

initial begin
    btn_rst_n = 0;
    #(CLK_PERIOD * 10);
    btn_rst_n = 1;

    // Run for (500 + 4800) samples worth of time, with 2x margin for cascade latency
    #((500 + 4800) * 576 * CLK_PERIOD * 2);

    $fclose(fd);
    $display("Simulation complete. %0d output samples written to top_tb_output.txt", out_sample_count);
    $finish;
end

// ── Waveform Dump ───────────────────────────────────────────────────────────
initial begin
    $dumpfile("tangamp_top.vcd");
    $dumpvars(0, tangamp_top_tb);
end

endmodule
