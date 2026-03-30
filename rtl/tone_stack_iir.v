// ============================================================================
// tone_stack_iir.v
// Tone stack using cascaded biquad (IIR) filters with 5 switchable presets.
//
// Presets computed from parametric EQ (low shelf + peaking + high shelf)
// using bilinear-transformed biquad coefficients. All coefficients verified
// to fit within Q2.14 signed 16-bit range (max |coeff| < 2.0).
//
// Preset 0: Flat       (0dB / 0dB / 0dB)
// Preset 1: Scooped    (+1.5dB bass / -7.2dB mid / +1.5dB treble)
// Preset 2: Mid Boost  (-3dB bass / +7.2dB mid / -3dB treble)
// Preset 3: Bass Heavy (+1.5dB bass / 0dB mid / -6dB treble)
// Preset 4: Bright     (-6dB bass / 0dB mid / +1.5dB treble)
//
// Biquad: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
// Coefficients in Q2.14 (signed 16-bit, multiply Q16.16 x Q2.14, shift >>14)
//
// Pipeline: 3 biquads processed sequentially, ~3 clocks each = 9 clocks total
//
// Fixed point: Q16.16 signed throughout
// ============================================================================

module tone_stack_iir (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [2:0]  preset,     // 0-4: selects coefficient preset
    input  wire        sample_en,
    input  wire signed [31:0] audio_in,
    output reg  signed [31:0] audio_out,
    output reg         out_valid
);

// ============================================================================
// Preset Coefficient Selection (Q2.14 signed 16-bit)
//
// Bass:   low shelf at 200Hz, Q=0.707
// Mid:    peaking at 800Hz, Q=0.7
// Treble: high shelf at 3kHz, Q=0.707
// ============================================================================

reg signed [15:0] bass_b0, bass_b1, bass_b2, bass_a1, bass_a2;
reg signed [15:0] mid_b0,  mid_b1,  mid_b2,  mid_a1,  mid_a2;
reg signed [15:0] treb_b0, treb_b1, treb_b2, treb_a1, treb_a2;

always @(*) begin
    case (preset)
    3'd0: begin // Flat (0dB / 0dB / 0dB) -- unity passthrough
        bass_b0 =  16'sd16384; bass_b1 = -16'sd32161; bass_b2 =  16'sd15788;
        bass_a1 = -16'sd32161; bass_a2 =  16'sd15788;
        mid_b0  =  16'sd16384; mid_b1  = -16'sd30324; mid_b2  =  16'sd14107;
        mid_a1  = -16'sd30324; mid_a2  =  16'sd14107;
        treb_b0 =  16'sd16384; treb_b1 = -16'sd23826; treb_b2 =  16'sd9405;
        treb_a1 = -16'sd23826; treb_a2 =  16'sd9405;
    end
    3'd1: begin // Scooped (+1.5dB bass / -7.2dB mid / +1.5dB treble)
        bass_b0 =  16'sd16410; bass_b1 = -16'sd32186; bass_b2 =  16'sd15788;
        bass_a1 = -16'sd32187; bass_a2 =  16'sd15813;
        mid_b0  =  16'sd15447; mid_b1  = -16'sd29280; mid_b2  =  16'sd13994;
        mid_a1  = -16'sd29280; mid_a2  =  16'sd13057;
        treb_b0 =  16'sd19019; treb_b1 = -16'sd28074; treb_b2 =  16'sd11169;
        treb_a1 = -16'sd23453; treb_a2 =  16'sd9184;
    end
    3'd2: begin // Mid Boost (-3dB bass / +7.2dB mid / -3dB treble)
        bass_b0 =  16'sd16332; bass_b1 = -16'sd32109; bass_b2 =  16'sd15786;
        bass_a1 = -16'sd32107; bass_a2 =  16'sd15736;
        mid_b0  =  16'sd17378; mid_b1  = -16'sd31056; mid_b2  =  16'sd13849;
        mid_a1  = -16'sd31056; mid_a2  =  16'sd14844;
        treb_b0 =  16'sd12159; treb_b1 = -16'sd17119; treb_b2 =  16'sd6649;
        treb_a1 = -16'sd24530; treb_a2 =  16'sd9835;
    end
    3'd3: begin // Bass Heavy (+1.5dB bass / 0dB mid / -6dB treble)
        bass_b0 =  16'sd16410; bass_b1 = -16'sd32186; bass_b2 =  16'sd15788;
        bass_a1 = -16'sd32187; bass_a2 =  16'sd15813;
        mid_b0  =  16'sd16384; mid_b1  = -16'sd30324; mid_b2  =  16'sd14107;
        mid_a1  = -16'sd30324; mid_a2  =  16'sd14107;
        treb_b0 =  16'sd9027;  treb_b1 = -16'sd12260; treb_b2 =  16'sd4684;
        treb_a1 = -16'sd25183; treb_a2 =  16'sd10251;
    end
    3'd4: begin // Bright (-6dB bass / 0dB mid / +1.5dB treble)
        bass_b0 =  16'sd16279; bass_b1 = -16'sd32051; bass_b2 =  16'sd15780;
        bass_a1 = -16'sd32047; bass_a2 =  16'sd15679;
        mid_b0  =  16'sd16384; mid_b1  = -16'sd30324; mid_b2  =  16'sd14107;
        mid_a1  = -16'sd30324; mid_a2  =  16'sd14107;
        treb_b0 =  16'sd19019; treb_b1 = -16'sd28074; treb_b2 =  16'sd11169;
        treb_a1 = -16'sd23453; treb_a2 =  16'sd9184;
    end
    default: begin // Default to Flat
        bass_b0 =  16'sd16384; bass_b1 = -16'sd32161; bass_b2 =  16'sd15788;
        bass_a1 = -16'sd32161; bass_a2 =  16'sd15788;
        mid_b0  =  16'sd16384; mid_b1  = -16'sd30324; mid_b2  =  16'sd14107;
        mid_a1  = -16'sd30324; mid_a2  =  16'sd14107;
        treb_b0 =  16'sd16384; treb_b1 = -16'sd23826; treb_b2 =  16'sd9405;
        treb_a1 = -16'sd23826; treb_a2 =  16'sd9405;
    end
    endcase
end

// ============================================================================
// Coefficient Arrays (indexed by biquad number)
// ============================================================================

reg signed [15:0] coeff_b0 [0:2];
reg signed [15:0] coeff_b1 [0:2];
reg signed [15:0] coeff_b2 [0:2];
reg signed [15:0] coeff_a1 [0:2];
reg signed [15:0] coeff_a2 [0:2];

always @(*) begin
    coeff_b0[0] = bass_b0;  coeff_b1[0] = bass_b1;  coeff_b2[0] = bass_b2;
    coeff_a1[0] = bass_a1;  coeff_a2[0] = bass_a2;

    coeff_b0[1] = mid_b0;   coeff_b1[1] = mid_b1;   coeff_b2[1] = mid_b2;
    coeff_a1[1] = mid_a1;   coeff_a2[1] = mid_a2;

    coeff_b0[2] = treb_b0;  coeff_b1[2] = treb_b1;  coeff_b2[2] = treb_b2;
    coeff_a1[2] = treb_a1;  coeff_a2[2] = treb_a2;
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
