// ============================================================================
// power_supply_sag.v
// Models B+ voltage droop under load (tube rectifier sag).
//
// Real tube amps with tube rectifiers (GZ34, 5Y3) have B+ voltage that drops
// when the output stage draws heavy current.  This creates the "squishy"
// compression feel players love.
//
// Algorithm (no multiplies — shifts and adds only):
//   abs_audio = |audio_in|
//   envelope += (abs_audio - envelope) >>> 12    // IIR tau ≈ 85ms @ 48kHz
//   b_plus   = V_IDEAL - (envelope >>> SAG_SHIFT)
//
// Resource usage: ~15-20 LUTs, 0 DSP, 0 BRAM.
// ============================================================================

module power_supply_sag #(
    parameter FP_FRAC   = 16,
    parameter FP_WIDTH  = 32,
    parameter SAG_SHIFT = 3       // 2=aggressive, 3=moderate, 4=subtle
)(
    input  wire                        clk,
    input  wire                        rst_n,
    input  wire                        sample_en,    // pulse at sample rate
    input  wire signed [FP_WIDTH-1:0]  audio_in,     // signal level
    output reg  signed [FP_WIDTH-1:0]  b_plus        // modulated B+ supply voltage
);

// 300V in Q16.16 = 300 * 65536 = 19,660,800
localparam signed [FP_WIDTH-1:0] V_IDEAL = 32'sd19660800;

// Envelope state
reg signed [FP_WIDTH-1:0] envelope;

// Absolute value of input (combinational)
wire signed [FP_WIDTH-1:0] abs_audio = audio_in[FP_WIDTH-1] ? -audio_in : audio_in;

// Envelope error term
wire signed [FP_WIDTH-1:0] env_diff = abs_audio - envelope;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        envelope <= {FP_WIDTH{1'b0}};
        b_plus   <= V_IDEAL;
    end else if (sample_en) begin
        // IIR envelope follower: alpha = 1/4096 -> tau ≈ 85ms @ 48kHz
        envelope <= envelope + (env_diff >>> 12);
        // B+ droops proportional to envelope
        b_plus   <= V_IDEAL - (envelope >>> SAG_SHIFT);
    end
end

endmodule
