// ============================================================================
// triode_cascade.v
// Cascaded WDF Triode Stages
//
// Instantiates NUM_STAGES copies of wdf_triode_wdf chained in series.
// Interstage attenuation via arithmetic right shift (ATTEN_SHIFT=2 -> -12dB).
//
// Fixed point: Q16.16 signed throughout
// ============================================================================

module triode_cascade #(
    parameter NUM_STAGES = 2,
    parameter ATTEN_SHIFT = 2  // -12dB between stages
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sample_en,
    input  wire signed [31:0] audio_in,
    output wire signed [31:0] audio_out,
    output wire        out_valid
);

// ── Inter-stage wires ──────────────────────────────────────────────────────
wire signed [31:0] stage_out [0:NUM_STAGES-1];
wire               stage_valid [0:NUM_STAGES-1];

// ── Generate cascaded stages ───────────────────────────────────────────────
genvar g;
generate
    for (g = 0; g < NUM_STAGES; g = g + 1) begin : stage
        wire signed [31:0] stg_in;
        wire               stg_en;

        if (g == 0) begin : first
            assign stg_in = audio_in;
            assign stg_en = sample_en;
        end else begin : chain
            // Attenuate previous stage output and use its valid as enable
            assign stg_in = stage_out[g-1] >>> ATTEN_SHIFT;
            assign stg_en = stage_valid[g-1];
        end

        wdf_triode_wdf triode_inst (
            .clk       (clk),
            .rst_n     (rst_n),
            .sample_en (stg_en),
            .audio_in  (stg_in),
            .audio_out (stage_out[g]),
            .out_valid (stage_valid[g])
        );
    end
endgenerate

// ── Output from last stage ─────────────────────────────────────────────────
assign audio_out = stage_out[NUM_STAGES-1];
assign out_valid = stage_valid[NUM_STAGES-1];

endmodule
