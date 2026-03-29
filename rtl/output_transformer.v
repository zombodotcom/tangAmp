// ============================================================================
// output_transformer.v
// Output transformer model: bandpass filter + soft saturation.
//
// Linear part: 2nd-order Butterworth HPF at 60Hz (primary inductance rolloff)
//            + 2nd-order Butterworth LPF at 8kHz (leakage inductance rolloff)
//
// Nonlinear part: Frequency-dependent soft clip modeling core saturation.
//   Core flux ~ V/(N*omega), so low frequencies saturate first.
//   A 1st-order IIR envelope follower at ~100Hz tracks LF content
//   and dynamically reduces the clip threshold when bass is strong.
//
// Biquad architecture matches tone_stack_iir.v (Direct Form I).
// Coefficients in Q2.14, state in Q16.16.
//
// Latency: 5 clock cycles after sample_en.
// Fixed point: Q16.16 signed throughout.
// ============================================================================

module output_transformer (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sample_en,
    input  wire signed [31:0] audio_in,
    output reg  signed [31:0] audio_out,
    output reg         out_valid
);

// ============================================================================
// Biquad Coefficients (Q2.14 signed 16-bit)
// Computed via: scipy.signal.butter(2, fc, btype, fs=48000)
// ============================================================================

// --- HPF 60Hz Butterworth 2nd order ---
localparam signed [15:0] HPF_B0 =  16'sd16293;   // 0.9945
localparam signed [15:0] HPF_B1 = -16'sd32587;   // -1.9889
localparam signed [15:0] HPF_B2 =  16'sd16293;   // 0.9945
localparam signed [15:0] HPF_A1 = -16'sd32586;   // -1.9889
localparam signed [15:0] HPF_A2 =  16'sd16203;   // 0.9890

// --- LPF 8kHz Butterworth 2nd order ---
localparam signed [15:0] LPF_B0 =  16'sd2540;    // 0.1551
localparam signed [15:0] LPF_B1 =  16'sd5081;    // 0.3101
localparam signed [15:0] LPF_B2 =  16'sd2540;    // 0.1551
localparam signed [15:0] LPF_A1 = -16'sd10161;   // -0.6202
localparam signed [15:0] LPF_A2 =  16'sd3939;    // 0.2404

// ============================================================================
// Frequency-dependent soft-clip (core saturation ~ V/f)
//
// Real transformer cores saturate when flux exceeds Bsat.  Flux is
// proportional to V/(N*omega), so low frequencies produce more flux
// for the same voltage and saturate first.
//
// Implementation:
//   1. Track low-frequency envelope via 1st-order IIR at ~100Hz
//   2. Scale clip threshold inversely with LF content:
//        threshold = BASE - lf_envelope * SAG_FACTOR
//      clamped to a minimum of 10V.
// ============================================================================

localparam FP_WIDTH = 32;
localparam FP_FRAC  = 16;

// LF envelope IIR coefficient: alpha = 2*pi*100/48000 ~ 0.01309
// In Q16.16: 0.01309 * 65536 = 858  (use 853 per spec)
localparam signed [FP_WIDTH-1:0] ENV_ALPHA = 32'sd853;

// Base clip threshold: 25V in Q16.16
localparam signed [FP_WIDTH-1:0] BASE_THRESHOLD = 32'sd1638400;  // 25 << 16
// Hard ceiling offset above dynamic threshold: 5V in Q16.16
localparam signed [FP_WIDTH-1:0] CEILING_MARGIN = 32'sd327680;   // 5 << 16
// Minimum threshold: 10V in Q16.16
localparam signed [FP_WIDTH-1:0] MIN_THRESHOLD  = 32'sd655360;   // 10 << 16
// LF sag factor (Q16.16): how much LF content reduces threshold.
// Value 4.0 means: for each 1V of LF envelope, threshold drops 4V.
localparam signed [FP_WIDTH-1:0] SAG_FACTOR = 32'sd262144;  // 4.0 in Q16.16

// LF envelope state
reg signed [FP_WIDTH-1:0] lf_envelope;

// Dynamic threshold computation
reg signed [FP_WIDTH-1:0] dyn_threshold;
reg signed [FP_WIDTH-1:0] clip_threshold;
reg signed [FP_WIDTH-1:0] clip_ceiling;
wire signed [FP_WIDTH-1:0] neg_threshold = -clip_threshold;
wire signed [FP_WIDTH-1:0] neg_ceiling   = -clip_ceiling;

// ============================================================================
// Coefficient Arrays (indexed by biquad number: 0=HPF, 1=LPF)
// ============================================================================

reg signed [15:0] coeff_b0 [0:1];
reg signed [15:0] coeff_b1 [0:1];
reg signed [15:0] coeff_b2 [0:1];
reg signed [15:0] coeff_a1 [0:1];
reg signed [15:0] coeff_a2 [0:1];

initial begin
    coeff_b0[0] = HPF_B0;  coeff_b1[0] = HPF_B1;  coeff_b2[0] = HPF_B2;
    coeff_a1[0] = HPF_A1;  coeff_a2[0] = HPF_A2;

    coeff_b0[1] = LPF_B0;  coeff_b1[1] = LPF_B1;  coeff_b2[1] = LPF_B2;
    coeff_a1[1] = LPF_A1;  coeff_a2[1] = LPF_A2;
end

// ============================================================================
// Biquad state: x[n-1], x[n-2], y[n-1], y[n-2] per biquad
// ============================================================================

reg signed [31:0] bq_x1 [0:1];
reg signed [31:0] bq_x2 [0:1];
reg signed [31:0] bq_y1 [0:1];
reg signed [31:0] bq_y2 [0:1];

integer init_i;
initial begin
    for (init_i = 0; init_i < 2; init_i = init_i + 1) begin
        bq_x1[init_i] = 0;
        bq_x2[init_i] = 0;
        bq_y1[init_i] = 0;
        bq_y2[init_i] = 0;
    end
end

// ============================================================================
// Processing State Machine
// ============================================================================

localparam S_IDLE    = 3'd0;
localparam S_COMPUTE = 3'd1;
localparam S_STORE   = 3'd2;
localparam S_ENV     = 3'd3;  // LF envelope update + dynamic threshold
localparam S_CLIP    = 3'd4;
localparam S_DONE    = 3'd5;

reg [2:0] proc_state;
reg       bq_idx;              // Current biquad (0=HPF, 1=LPF)
reg signed [31:0] bq_in;      // Input to current biquad

// MAC temporaries
reg signed [63:0] mac_acc;
reg signed [63:0] mac_tmp;
reg signed [31:0] bq_out;

// Envelope temporaries
reg signed [FP_WIDTH-1:0] abs_in;
reg signed [63:0]         env_tmp;
reg signed [63:0]         sag_tmp;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        proc_state     <= S_IDLE;
        out_valid      <= 1'b0;
        audio_out      <= 0;
        bq_idx         <= 0;
        bq_in          <= 0;
        bq_out         <= 0;
        mac_acc        <= 0;
        lf_envelope    <= 0;
        dyn_threshold  <= BASE_THRESHOLD;
        clip_threshold <= BASE_THRESHOLD;
        clip_ceiling   <= BASE_THRESHOLD + CEILING_MARGIN;
    end else begin
        out_valid <= 1'b0;

        case (proc_state)

        S_IDLE: begin
            if (sample_en) begin
                bq_idx <= 0;
                bq_in  <= audio_in;
                proc_state <= S_COMPUTE;
            end
        end

        // Compute one biquad:
        // y[n] = (b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]) >> 14
        S_COMPUTE: begin
            mac_acc = $signed(coeff_b0[bq_idx]) * $signed(bq_in);
            mac_tmp = $signed(coeff_b1[bq_idx]) * $signed(bq_x1[bq_idx]);
            mac_acc = mac_acc + mac_tmp;
            mac_tmp = $signed(coeff_b2[bq_idx]) * $signed(bq_x2[bq_idx]);
            mac_acc = mac_acc + mac_tmp;
            mac_tmp = $signed(coeff_a1[bq_idx]) * $signed(bq_y1[bq_idx]);
            mac_acc = mac_acc - mac_tmp;
            mac_tmp = $signed(coeff_a2[bq_idx]) * $signed(bq_y2[bq_idx]);
            mac_acc = mac_acc - mac_tmp;

            bq_out <= mac_acc >>> 14;
            proc_state <= S_STORE;
        end

        // Store biquad state, advance to next or move to envelope
        S_STORE: begin
            bq_x2[bq_idx] <= bq_x1[bq_idx];
            bq_x1[bq_idx] <= bq_in;
            bq_y2[bq_idx] <= bq_y1[bq_idx];
            bq_y1[bq_idx] <= bq_out;

            if (bq_idx == 1'd1) begin
                // Both biquads done, update envelope then clip
                proc_state <= S_ENV;
            end else begin
                bq_in  <= bq_out;
                bq_idx <= 1;
                proc_state <= S_COMPUTE;
            end
        end

        // LF envelope tracking + dynamic threshold computation
        S_ENV: begin
            // abs(bq_out)
            abs_in = (bq_out < 0) ? -bq_out : bq_out;

            // 1st-order IIR: lf_envelope += (abs_in - lf_envelope) * alpha >> 16
            env_tmp = ($signed(abs_in) - $signed(lf_envelope)) * $signed(ENV_ALPHA);
            lf_envelope <= lf_envelope + (env_tmp >>> FP_FRAC);

            // Dynamic threshold: BASE - lf_envelope * SAG_FACTOR
            sag_tmp = $signed(lf_envelope) * $signed(SAG_FACTOR);
            dyn_threshold <= BASE_THRESHOLD - (sag_tmp >>> FP_FRAC);

            // Clamp to minimum and compute ceiling
            if (dyn_threshold < MIN_THRESHOLD) begin
                clip_threshold <= MIN_THRESHOLD;
                clip_ceiling   <= MIN_THRESHOLD + CEILING_MARGIN;
            end else begin
                clip_threshold <= dyn_threshold;
                clip_ceiling   <= dyn_threshold + CEILING_MARGIN;
            end

            proc_state <= S_CLIP;
        end

        // Piecewise-linear soft clip (frequency-dependent core saturation)
        S_CLIP: begin
            if (bq_out > clip_threshold) begin
                if (bq_out > clip_ceiling)
                    audio_out <= clip_ceiling;
                else
                    audio_out <= clip_threshold + ((bq_out - clip_threshold) >>> 2);
            end else if (bq_out < neg_threshold) begin
                if (bq_out < neg_ceiling)
                    audio_out <= neg_ceiling;
                else
                    audio_out <= neg_threshold + ((bq_out - neg_threshold) >>> 2);
            end else begin
                audio_out <= bq_out;
            end
            out_valid  <= 1'b1;
            proc_state <= S_IDLE;
        end

        default: proc_state <= S_IDLE;

        endcase
    end
end

endmodule
