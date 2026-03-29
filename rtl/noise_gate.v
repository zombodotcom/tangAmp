// ============================================================================
// noise_gate.v
// Noise gate for input signal — suppresses hiss when guitar is not playing.
//
// Algorithm:
//   1. Envelope follower: asymmetric IIR on |audio_in| (fast attack, slow release)
//   2. Threshold comparator: envelope > threshold -> gate open
//   3. Gain smoothing: IIR ramp on gate gain to avoid clicks
//   4. Output: audio_in * gain (single DSP multiply)
//
// Resource budget: ~50 LUTs, 1 DSP multiply
// ============================================================================

module noise_gate #(
    parameter FP_FRAC      = 16,
    parameter FP_WIDTH     = 32,
    parameter ATTACK_SHIFT  = 6,   // alpha=1/64,  tau~1.3ms  @ 48kHz
    parameter RELEASE_SHIFT = 13,  // alpha=1/8192, tau~170ms @ 48kHz
    parameter SMOOTH_SHIFT  = 8    // alpha=1/256,  tau~5.3ms @ 48kHz
)(
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        sample_en,
    input  wire signed [FP_WIDTH-1:0]  audio_in,
    output reg  signed [FP_WIDTH-1:0]  audio_out,
    input  wire [7:0]                  threshold     // 0-255, from pot or constant
);

    localparam signed [FP_WIDTH-1:0] ONE = 32'sd65536; // 1.0 in Q16.16

    // Envelope follower state
    reg signed [FP_WIDTH-1:0] envelope;

    // Gate gain (0..ONE)
    reg signed [FP_WIDTH-1:0] gain;

    // Absolute value of input
    wire signed [FP_WIDTH-1:0] abs_in = audio_in[FP_WIDTH-1] ? -audio_in : audio_in;

    // Threshold scaled to Q16.16: threshold byte into integer part bits [23:16]
    wire signed [FP_WIDTH-1:0] thresh_fp = {8'b0, threshold, 16'b0};

    // Envelope delta
    wire signed [FP_WIDTH-1:0] env_delta = abs_in - envelope;

    // Gain multiply (64-bit intermediate)
    wire signed [2*FP_WIDTH-1:0] mult = $signed(audio_in) * $signed(gain);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            envelope  <= {FP_WIDTH{1'b0}};
            gain      <= ONE;               // start open so first notes pass
            audio_out <= {FP_WIDTH{1'b0}};
        end else if (sample_en) begin
            // ── Envelope follower (asymmetric attack/release) ───────────
            if (abs_in > envelope)
                envelope <= envelope + (env_delta >>> ATTACK_SHIFT);
            else
                envelope <= envelope + (env_delta >>> RELEASE_SHIFT);

            // ── Gate decision + gain smoothing ──────────────────────────
            if (envelope > thresh_fp)
                gain <= gain + ((ONE - gain) >>> SMOOTH_SHIFT);
            else
                gain <= gain + ((-gain) >>> SMOOTH_SHIFT);

            // ── Apply gain (1 multiply) ─────────────────────────────────
            audio_out <= mult[FP_FRAC +: FP_WIDTH];
        end
    end

endmodule
