// ============================================================================
// output_transformer.v
// Output transformer model: bandpass filter + soft saturation.
//
// Linear part: 2nd-order Butterworth HPF at 60Hz (primary inductance rolloff)
//            + 2nd-order Butterworth LPF at 8kHz (leakage inductance rolloff)
//
// Nonlinear part: Piecewise-linear soft clip modeling core saturation.
//
// Biquad architecture matches tone_stack_iir.v (Direct Form I).
// Coefficients in Q2.14, state in Q16.16.
//
// Latency: 4 clock cycles after sample_en.
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
// NOTE: b0+b1+b2 must equal exactly 0 for HPF DC rejection.
// Original b1=-32587 gave sum=-1, causing DC gain of -1 (full DC leak).
localparam signed [15:0] HPF_B0 =  16'sd16293;   // 0.9945
localparam signed [15:0] HPF_B1 = -16'sd32586;   // -(b0+b2) = exact zero DC gain
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
// Soft-clip thresholds (piecewise linear, reused from power_amp_stage.v)
// ============================================================================

// Clip threshold: 25V in Q16.16
localparam signed [31:0] CLIP_THRESHOLD = 32'sd1638400;
// Hard ceiling: 30V in Q16.16
localparam signed [31:0] CLIP_CEILING   = 32'sd1966080;
wire signed [31:0] NEG_THRESHOLD = -CLIP_THRESHOLD;
wire signed [31:0] NEG_CEILING   = -CLIP_CEILING;

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
localparam S_CLIP    = 3'd3;
localparam S_DONE    = 3'd4;

reg [2:0] proc_state;
reg       bq_idx;              // Current biquad (0=HPF, 1=LPF)
reg signed [31:0] bq_in;      // Input to current biquad

// MAC temporaries
reg signed [63:0] mac_acc;
reg signed [63:0] mac_tmp;
reg signed [31:0] bq_out;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        proc_state <= S_IDLE;
        out_valid  <= 1'b0;
        audio_out  <= 0;
        bq_idx     <= 0;
        bq_in      <= 0;
        bq_out     <= 0;
        mac_acc    <= 0;
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

        // Store biquad state, advance to next or move to clipping
        S_STORE: begin
            bq_x2[bq_idx] <= bq_x1[bq_idx];
            bq_x1[bq_idx] <= bq_in;
            bq_y2[bq_idx] <= bq_y1[bq_idx];
            bq_y1[bq_idx] <= bq_out;

            if (bq_idx == 1'd1) begin
                // Both biquads done, apply soft saturation
                proc_state <= S_CLIP;
            end else begin
                bq_in  <= bq_out;
                bq_idx <= 1;
                proc_state <= S_COMPUTE;
            end
        end

        // Piecewise-linear soft clip (transformer core saturation)
        S_CLIP: begin
            if (bq_out > CLIP_THRESHOLD) begin
                if (bq_out > CLIP_CEILING)
                    audio_out <= CLIP_CEILING;
                else
                    audio_out <= CLIP_THRESHOLD + ((bq_out - CLIP_THRESHOLD) >>> 2);
            end else if (bq_out < NEG_THRESHOLD) begin
                if (bq_out < NEG_CEILING)
                    audio_out <= NEG_CEILING;
                else
                    audio_out <= NEG_THRESHOLD + ((bq_out - NEG_THRESHOLD) >>> 2);
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
