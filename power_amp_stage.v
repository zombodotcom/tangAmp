// ============================================================================
// power_amp_stage.v
// Simplified power amp for FPGA: gain + soft clip.
//
// Since BRAM is full with preamp triode LUTs, this module approximates
// the power amp character using:
//   1. Configurable gain (left shift)
//   2. Piecewise-linear soft clipping (approximates transformer saturation)
//
// The soft clip uses two regions:
//   - Linear:     |x| < THRESHOLD  ->  pass through
//   - Compressed: |x| >= THRESHOLD ->  THRESHOLD + (x - THRESHOLD) >>> 2
//
// This gives a smooth-ish transition from clean to compressed, similar
// to transformer saturation in a real power amp.
//
// Fixed point: Q16.16 signed throughout.
// Latency: 2 clock cycles when sample_en asserted.
// ============================================================================

module power_amp_stage #(
    parameter FP_FRAC        = 16,
    parameter FP_WIDTH       = 32,
    parameter POWER_GAIN_SHIFT = 2,  // 4x gain (~12dB)
    // Clip threshold: 25V in Q16.16 = 25 * 65536 = 1,638,400
    parameter signed [FP_WIDTH-1:0] CLIP_THRESHOLD = 32'sd1638400,
    // Hard ceiling: 30V in Q16.16 = 30 * 65536 = 1,966,080
    parameter signed [FP_WIDTH-1:0] CLIP_CEILING   = 32'sd1966080
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sample_en,
    input  wire signed [FP_WIDTH-1:0] audio_in,
    output reg  signed [FP_WIDTH-1:0] audio_out,
    output reg         out_valid
);

// Internal pipeline registers
reg signed [FP_WIDTH-1:0] gained;
reg        pipe_valid;

// Negative thresholds
wire signed [FP_WIDTH-1:0] neg_threshold = -CLIP_THRESHOLD;
wire signed [FP_WIDTH-1:0] neg_ceiling   = -CLIP_CEILING;

// ============================================================================
// Pipeline Stage 1: Apply gain
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        gained     <= 0;
        pipe_valid <= 1'b0;
    end else begin
        pipe_valid <= 1'b0;
        if (sample_en) begin
            gained     <= audio_in <<< POWER_GAIN_SHIFT;
            pipe_valid <= 1'b1;
        end
    end
end

// ============================================================================
// Pipeline Stage 2: Soft clip
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        audio_out <= 0;
        out_valid <= 1'b0;
    end else begin
        out_valid <= 1'b0;
        if (pipe_valid) begin
            if (gained > CLIP_THRESHOLD) begin
                // Positive compression region
                // out = threshold + (gained - threshold) >>> 2
                // Capped at ceiling
                if (gained > CLIP_CEILING)
                    audio_out <= CLIP_CEILING;
                else
                    audio_out <= CLIP_THRESHOLD + ((gained - CLIP_THRESHOLD) >>> 2);
            end else if (gained < neg_threshold) begin
                // Negative compression region
                if (gained < neg_ceiling)
                    audio_out <= neg_ceiling;
                else
                    audio_out <= neg_threshold + ((gained - neg_threshold) >>> 2);
            end else begin
                // Linear region: pass through
                audio_out <= gained;
            end
            out_valid <= 1'b1;
        end
    end
end

endmodule
