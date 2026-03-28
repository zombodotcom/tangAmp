// ============================================================================
// tone_stack_iir.v
// 3-band parametric EQ tone stack using cascaded biquad (IIR) filters.
//
// Bands:
//   Bass:   lowshelf at 200Hz
//   Mid:    peaking at 800Hz, Q=0.7
//   Treble: highshelf at 3kHz
//
// Default coefficients are FLAT (bass=5, mid=5, treble=5 = 0dB all bands).
// At 0dB, all shelf/peaking filters reduce to unity (b==a), so this is
// a transparent pass-through by default — ready for knob control later.
//
// Biquad: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
// Coefficients in Q2.14 (signed 16-bit, multiply Q16.16 × Q2.14, shift >>14)
//
// Pipeline: 3 biquads processed sequentially, ~3 clocks each = 9 clocks total
//
// Fixed point: Q16.16 signed throughout
// ============================================================================

module tone_stack_iir (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sample_en,
    input  wire signed [31:0] audio_in,
    output reg  signed [31:0] audio_out,
    output reg         out_valid
);

// ============================================================================
// Biquad Coefficients (Q2.14 signed 16-bit)
// Default: flat (0dB on all bands) — b == a so output == input
//
// Bass: lowshelf 200Hz, 0dB, Q=0.707
// Mid:  peaking 800Hz, 0dB, Q=0.7
// Treble: highshelf 3kHz, 0dB, Q=0.707
// ============================================================================

// Bass biquad
localparam signed [15:0] BASS_B0 =  16'sd16384;   // 1.0
localparam signed [15:0] BASS_B1 = -16'sd32161;   // -1.9630
localparam signed [15:0] BASS_B2 =  16'sd15788;   // 0.9636
localparam signed [15:0] BASS_A1 = -16'sd32161;   // -1.9630
localparam signed [15:0] BASS_A2 =  16'sd15788;   // 0.9636

// Mid biquad
localparam signed [15:0] MID_B0 =  16'sd16384;    // 1.0
localparam signed [15:0] MID_B1 = -16'sd30324;    // -1.8509
localparam signed [15:0] MID_B2 =  16'sd14107;    // 0.8610
localparam signed [15:0] MID_A1 = -16'sd30324;    // -1.8509
localparam signed [15:0] MID_A2 =  16'sd14107;    // 0.8610

// Treble biquad
localparam signed [15:0] TREB_B0 =  16'sd16384;   // 1.0
localparam signed [15:0] TREB_B1 = -16'sd23826;   // -1.4542
localparam signed [15:0] TREB_B2 =  16'sd9405;    // 0.5740
localparam signed [15:0] TREB_A1 = -16'sd23826;   // -1.4542
localparam signed [15:0] TREB_A2 =  16'sd9405;    // 0.5740

// ============================================================================
// Coefficient Arrays (indexed by biquad number)
// ============================================================================

reg signed [15:0] coeff_b0 [0:2];
reg signed [15:0] coeff_b1 [0:2];
reg signed [15:0] coeff_b2 [0:2];
reg signed [15:0] coeff_a1 [0:2];
reg signed [15:0] coeff_a2 [0:2];

initial begin
    coeff_b0[0] = BASS_B0;  coeff_b1[0] = BASS_B1;  coeff_b2[0] = BASS_B2;
    coeff_a1[0] = BASS_A1;  coeff_a2[0] = BASS_A2;

    coeff_b0[1] = MID_B0;   coeff_b1[1] = MID_B1;   coeff_b2[1] = MID_B2;
    coeff_a1[1] = MID_A1;   coeff_a2[1] = MID_A2;

    coeff_b0[2] = TREB_B0;  coeff_b1[2] = TREB_B1;  coeff_b2[2] = TREB_B2;
    coeff_a1[2] = TREB_A1;  coeff_a2[2] = TREB_A2;
end

// ============================================================================
// State per biquad: x[n-1], x[n-2], y[n-1], y[n-2]
// ============================================================================

reg signed [31:0] bq_x1 [0:2];
reg signed [31:0] bq_x2 [0:2];
reg signed [31:0] bq_y1 [0:2];
reg signed [31:0] bq_y2 [0:2];

integer init_i;
initial begin
    for (init_i = 0; init_i < 3; init_i = init_i + 1) begin
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
localparam S_COMPUTE = 3'd1;  // Multiply-accumulate for current biquad
localparam S_STORE   = 3'd2;  // Store biquad state, advance
localparam S_DONE    = 3'd3;

reg [2:0] proc_state;
reg [1:0] bq_idx;           // Current biquad (0, 1, 2)
reg signed [31:0] bq_in;    // Input to current biquad

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

        // Compute one biquad in a single clock:
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

            // Shift right by 14 to convert from Q2.14 multiply result back to Q16.16
            bq_out <= mac_acc >>> 14;

            proc_state <= S_STORE;
        end

        // Store biquad state, advance to next or finish
        S_STORE: begin
            bq_x2[bq_idx] <= bq_x1[bq_idx];
            bq_x1[bq_idx] <= bq_in;
            bq_y2[bq_idx] <= bq_y1[bq_idx];
            bq_y1[bq_idx] <= bq_out;

            if (bq_idx == 2'd2) begin
                // All 3 biquads done
                audio_out  <= bq_out;
                out_valid  <= 1'b1;
                proc_state <= S_IDLE;
            end else begin
                // Feed output of this biquad to next
                bq_in  <= bq_out;
                bq_idx <= bq_idx + 1;
                proc_state <= S_COMPUTE;
            end
        end

        default: proc_state <= S_IDLE;

        endcase
    end
end

endmodule
