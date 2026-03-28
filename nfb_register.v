// ============================================================================
// nfb_register.v
// Global negative feedback register for tube amplifier emulation.
//
// Captures a fraction of the output signal (after cabinet/transformer) and
// stores it for subtraction from the input on the next sample. The fraction
// is a power-of-2 shift (no DSP block needed).
//
// NFB_SHIFT=4 -> 1/16 = 0.0625 (light, ~Vox)
// NFB_SHIFT=3 -> 1/8  = 0.125  (moderate, ~Marshall)
// NFB_SHIFT=2 -> 1/4  = 0.25   (heavy, ~Fender)
//
// The top-level module subtracts nfb_signal from the triode engine input.
// 1-sample delay is inherent and physically correct (20.8us at 48kHz).
//
// Cost: 1x 32-bit register + 1x 32-bit arithmetic shift = ~32 LUTs + 32 FFs.
// ============================================================================

module nfb_register #(
    parameter NFB_SHIFT = 4  // feedback amount: output >>> NFB_SHIFT
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire signed [31:0] fb_in,      // signal to feed back (cab output)
    input  wire        fb_valid,           // pulse when fb_in is valid
    output reg  signed [31:0] nfb_signal   // subtract this from input
);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        nfb_signal <= 32'sd0;
    end else begin
        if (fb_valid) begin
            nfb_signal <= fb_in >>> NFB_SHIFT;
        end
    end
end

endmodule
