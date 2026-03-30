// ============================================================================
// tangamp_selftest.v
// Self-test top-level for Tang Nano 20K — no external ADC/DAC needed.
//
// Full signal chain with 2x oversampling on the nonlinear triode stage:
//   440Hz sine - nfb -> [upsample 2x] -> triode_engine (runs 2x at 96kHz)
//   -> [downsample 2x] -> tone_stack_iir -> output_transformer
//   -> cabinet_fir -> VU meter LEDs
//                        |
//                   nfb_register <-
//
// Only the triode (nonlinear) runs at 96kHz to reduce aliasing.
// Linear stages (tone stack, transformer, cabinet) remain at 48kHz.
//
// LED[0] = heartbeat (proves clock running)
// LED[1:5] = output level (VU meter, 5 segments)
//
// Tang Nano 20K: Gowin GW2AR-LV18QN88C8/I7 (or GW2A-LV18QN88C8/I7)
// Clock: 27MHz on pin 52
// LEDs: active LOW on pins 15,16,17,18,19,20
// Buttons: S1=pin 88 (active low), S2=pin 87
// ============================================================================

module tangamp_selftest (
    input  wire clk_27m,     // 27MHz crystal, pin 52
    input  wire btn_s1,      // Button S1, pin 88, active low
    output wire [5:0] led    // LEDs, pins 15-20, active low
);

// ── Reset ────────────────────────────────────────────────────────────────
reg [1:0] rst_sync;
reg rst_n;
always @(posedge clk_27m) begin
    rst_sync <= {rst_sync[0], btn_s1};
    rst_n <= rst_sync[1];
end

// ── Sample rate clock (48kHz from 27MHz) ────────────────────────────────
// 27MHz / 562 = 48043Hz (close enough to 48kHz)
reg [9:0] div_cnt;
reg sample_en;

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        div_cnt <= 0;
        sample_en <= 0;
    end else begin
        sample_en <= 0;
        if (div_cnt >= 10'd561) begin
            div_cnt <= 0;
            sample_en <= 1;
        end else begin
            div_cnt <= div_cnt + 1;
        end
    end
end

// ── Sine wave generator (440Hz at 48kHz = ~109 samples/period) ──────────
// 0.5V amplitude in Q16.16 = 32768
reg signed [31:0] sine_table [0:107];
integer i;
initial begin
    for (i = 0; i < 108; i = i + 1)
        sine_table[i] = $rtoi($sin(6.2831853 * i / 108.0) * 0.5 * 65536.0);
end

reg [6:0] sine_idx;
reg signed [31:0] audio_in;
reg [15:0] settle_cnt;
localparam SETTLE_SAMPLES = 16'd11000;  // silence for DC settling

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        sine_idx <= 0;
        audio_in <= 0;
        settle_cnt <= 0;
    end else if (sample_en) begin
        if (settle_cnt < SETTLE_SAMPLES) begin
            audio_in <= 0;  // silence during settling
            settle_cnt <= settle_cnt + 1;
        end else begin
            audio_in <= sine_table[sine_idx];
            sine_idx <= (sine_idx >= 7'd107) ? 0 : sine_idx + 1;
        end
    end
end

// ── Noise Gate (suppress hiss when not playing) ─────────────────────────
wire signed [31:0] gated_in;

noise_gate #(
    .FP_FRAC  (16),
    .FP_WIDTH (32)
) ngate (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .sample_en (sample_en),
    .audio_in  (audio_in),
    .audio_out (gated_in),
    .threshold (8'h10)       // quiet threshold, tune later with pot
);

// ── Negative Feedback Subtraction ────────────────────────────────────────
wire signed [31:0] nfb_signal;
reg  signed [31:0] triode_input;

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n)
        triode_input <= 0;
    else if (sample_en)
        triode_input <= gated_in - nfb_signal;
end

// ── 2x Oversampling + Triode Engine ─────────────────────────────────────
// The oversampler generates two samples per 48kHz period.
// A controller state machine runs the triode engine twice (pass A, pass B)
// then feeds results back to the oversampler for decimation.

wire        up_valid_a, up_valid_b;
wire signed [31:0] audio_up;

// Triode engine I/O
reg         triode_start;
reg  signed [31:0] triode_in;
wire signed [31:0] triode_out;
wire        triode_valid;

// Processed samples for downsampling
reg  signed [31:0] proc_a_sample;
reg         proc_a_valid;
reg  signed [31:0] proc_b_sample;
reg         proc_b_valid;

// Downsampled output
wire signed [31:0] audio_down;
wire        down_valid;

oversample_2x oversamp (
    .clk            (clk_27m),
    .rst_n          (rst_n),
    .sample_en_48k  (sample_en),
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
    .POWER_AMP   (1)
) triode (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .sample_en (triode_start),
    .audio_in  (triode_in),
    .audio_out (triode_out),
    .out_valid (triode_valid)
);

// ── Oversampling Controller State Machine ───────────────────────────────
// Sequence per 48kHz sample:
//   1. Oversampler outputs up_valid_a with first sample
//   2. We start triode pass A
//   3. Triode finishes pass A -> capture result, start pass B with second sample
//   4. Oversampler has already output up_valid_b (second sample held)
//   5. Triode finishes pass B -> feed both results to downsampler

localparam OS_IDLE       = 3'd0;
localparam OS_WAIT_TRI_A = 3'd1;  // Waiting for triode pass A to complete
localparam OS_START_B    = 3'd2;  // Start triode pass B
localparam OS_WAIT_TRI_B = 3'd3;  // Waiting for triode pass B to complete
localparam OS_DECIMATE   = 3'd4;  // Feed results to downsampler

reg [2:0] os_state;
reg signed [31:0] up_b_held;      // Hold second upsampled sample
reg signed [31:0] triode_a_held;  // Hold first triode result

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        os_state      <= OS_IDLE;
        triode_start  <= 1'b0;
        triode_in     <= 32'sd0;
        proc_a_valid  <= 1'b0;
        proc_b_valid  <= 1'b0;
        proc_a_sample <= 32'sd0;
        proc_b_sample <= 32'sd0;
        up_b_held     <= 32'sd0;
        triode_a_held <= 32'sd0;
    end else begin
        triode_start <= 1'b0;
        proc_a_valid <= 1'b0;
        proc_b_valid <= 1'b0;

        case (os_state)

        OS_IDLE: begin
            if (up_valid_a) begin
                // First upsampled sample ready — start triode pass A
                triode_in    <= audio_up;
                triode_start <= 1'b1;
                os_state     <= OS_WAIT_TRI_A;
            end
        end

        OS_WAIT_TRI_A: begin
            // Capture second upsampled sample when it appears
            if (up_valid_b) begin
                up_b_held <= audio_up;
            end

            // Wait for triode to finish pass A
            if (triode_valid) begin
                triode_a_held <= triode_out;
                os_state      <= OS_START_B;
            end
        end

        OS_START_B: begin
            // Start triode pass B with second upsampled sample
            triode_in    <= up_b_held;
            triode_start <= 1'b1;
            os_state     <= OS_WAIT_TRI_B;
        end

        OS_WAIT_TRI_B: begin
            // Wait for triode to finish pass B
            if (triode_valid) begin
                // Both passes done — send to downsampler
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

// ── Stage 2: Tone Stack (3-band biquad EQ) at 48kHz ────────────────────
wire signed [31:0] tone_out;
wire tone_valid;

tone_stack_iir tone (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .preset    (3'd0),
    .sample_en (down_valid),
    .audio_in  (audio_down),
    .audio_out (tone_out),
    .out_valid (tone_valid)
);

// ── Stage 3: Output Transformer (HPF 60Hz + LPF 8kHz + soft clip) ──────
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

// ── Power Supply Sag (B+ droop tracker, no audio modification) ─────────
// Tracks envelope of post-transformer signal; b_plus output is
// informational — will feed back to triode engine as dynamic Vpp later.
wire signed [31:0] b_plus_out;

power_supply_sag #(
    .SAG_SHIFT (3)   // moderate sag
) psu_sag (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .sample_en (xf_valid),
    .audio_in  (xf_out),
    .b_plus    (b_plus_out)
);

// ── Stage 4: Cabinet FIR at 48kHz ──────────────────────────────────────
wire signed [31:0] cab_out;
wire cab_valid;

cabinet_fir #(
    .N_TAPS (129)
) cabinet (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .sample_en (xf_valid),
    .audio_in  (xf_out),
    .audio_out (cab_out),
    .out_valid (cab_valid)
);

// ── Negative Feedback Register ──────────────────────────────────────────
nfb_register #(
    .NFB_SHIFT (3)  // moderate feedback (~Marshall)
) nfb (
    .clk        (clk_27m),
    .rst_n      (rst_n),
    .fb_in      (cab_out),
    .fb_valid   (cab_valid),
    .nfb_signal (nfb_signal)
);

// ── VU Meter (output level -> LED bar) ──────────────────────────────────
// Take absolute value of final output, map to 5 LED thresholds
reg [31:0] abs_out;
reg [4:0] vu;

always @(posedge clk_27m) begin
    if (cab_valid) begin
        abs_out <= (cab_out[31]) ? -cab_out : cab_out;
        // Thresholds in Q16.16 (approximately 1V, 3V, 8V, 15V, 25V)
        vu[0] <= (abs_out > 32'd65536);      // > 1V
        vu[1] <= (abs_out > 32'd196608);     // > 3V
        vu[2] <= (abs_out > 32'd524288);     // > 8V
        vu[3] <= (abs_out > 32'd983040);     // > 15V
        vu[4] <= (abs_out > 32'd1638400);    // > 25V
    end
end

// ── Heartbeat (LED 0 blinks at ~1Hz) ─────────────────────────────────────
reg [24:0] hb_cnt;
reg hb;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        hb_cnt <= 0;
        hb <= 0;
    end else begin
        if (hb_cnt >= 25'd13_500_000) begin  // 27MHz / 2 / 13.5M = 1Hz
            hb_cnt <= 0;
            hb <= ~hb;
        end else begin
            hb_cnt <= hb_cnt + 1;
        end
    end
end

// ── LED output (active low on Tang Nano 20K) ────────────────────────────
assign led[0] = ~hb;
assign led[1] = ~vu[0];
assign led[2] = ~vu[1];
assign led[3] = ~vu[2];
assign led[4] = ~vu[3];
assign led[5] = ~vu[4];

endmodule
