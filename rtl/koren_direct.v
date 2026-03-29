// ============================================================================
// koren_direct.v
// Hybrid 1D-LUT Koren Triode Equation Evaluator (12AX7)
//
// Computes Ip(Vpk, Vgk) using 3 small 1D LUTs with linear interpolation
// instead of a large 2D LUT. Total memory: 384 bytes vs 32KB.
//
// Pipeline (7 clocks):
//   S_SQRT:     Look up sqrt(kvb + vpk^2) from sqrt_lut, interpolate
//   S_INNER:    Compute inner = kp * (1/mu + vgk / sqrt_val)
//   S_SOFTPLUS: Look up log(1+exp(inner)) from softplus_lut, interpolate
//   S_ED:       Compute Ed = (vpk/kp) * softplus
//   S_POWER:    Look up Ed^1.29 from power_lut, interpolate
//   S_IP:       Compute Ip = power / kg1
//   S_DONE:     Output result
//
// Fixed point: Q16.16 signed 32-bit throughout
// ============================================================================

module koren_direct #(
    parameter FP_FRAC  = 16,
    parameter FP_WIDTH = 32
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire signed [FP_WIDTH-1:0] vpk,    // plate-cathode voltage Q16.16
    input  wire signed [FP_WIDTH-1:0] vgk,    // grid-cathode voltage Q16.16
    output reg  signed [FP_WIDTH-1:0] ip_out, // plate current Q16.16
    output reg         done
);

// ============================================================================
// Constants (12AX7 fitted to RCA datasheet)
// ============================================================================
localparam KLUT_SIZE = 64;
localparam KLUT_BITS = 6;

// Koren constants in Q16.16
localparam signed [FP_WIDTH-1:0] KOREN_MU_FP     = 32'sd6033981;   // 92.08
localparam signed [FP_WIDTH-1:0] KOREN_KP_FP     = 32'sd36769996;  // 561.08
localparam signed [FP_WIDTH-1:0] KOREN_KG1_FP    = 32'sd85492203;  // 1304.71
localparam signed [FP_WIDTH-1:0] KOREN_INV_MU_FP = 32'sd712;       // 1/92.08
localparam signed [FP_WIDTH-1:0] KOREN_INV_KP_FP = 32'sd117;       // 1/561.08

// Sqrt LUT addressing: Vpk 0-300V, 64 entries
// Step = 300/(64-1) = 4.7619V, in Q16.16 = 312015
localparam signed [FP_WIDTH-1:0] SQRT_VPK_MIN_FP  = 32'sd0;
localparam signed [FP_WIDTH-1:0] SQRT_VPK_STEP_FP = 32'sd312015;

// Softplus LUT addressing: inner -6 to +6, 64 entries
// Step = 12/(64-1) = 0.19048, in Q16.16 = 12483
localparam signed [FP_WIDTH-1:0] SOFTPLUS_MIN_FP  = -32'sd393216;  // -6.0
localparam signed [FP_WIDTH-1:0] SOFTPLUS_MAX_FP  = 32'sd393216;   // +6.0
localparam signed [FP_WIDTH-1:0] SOFTPLUS_STEP_FP = 32'sd12483;

// Power LUT addressing: Ed 0 to 0.6, 64 entries
// Step = 0.6/(64-1) = 0.009524, in Q16.16 = 624
localparam signed [FP_WIDTH-1:0] POWER_ED_MIN_FP  = 32'sd0;
localparam signed [FP_WIDTH-1:0] POWER_ED_MAX_FP  = 32'sd39322;    // 0.6
localparam signed [FP_WIDTH-1:0] POWER_ED_STEP_FP = 32'sd624;

// 1.0 in Q16.16
localparam signed [FP_WIDTH-1:0] ONE_FP = 32'sd65536;

// ============================================================================
// 1D LUT Memory (tiny -- fits in distributed RAM / registers)
// ============================================================================
reg [FP_WIDTH-1:0] sqrt_lut     [0:KLUT_SIZE-1];  // sqrt(kvb + vpk^2)
reg [FP_WIDTH-1:0] softplus_lut [0:KLUT_SIZE-1];  // log(1 + exp(x))
reg [FP_WIDTH-1:0] power_lut    [0:KLUT_SIZE-1];  // x^1.29

initial begin
    $readmemh("data/sqrt_lut.hex",     sqrt_lut);
    $readmemh("data/softplus_lut.hex", softplus_lut);
    $readmemh("data/power_lut.hex",    power_lut);
end

// ============================================================================
// State Machine
// ============================================================================
localparam [2:0] S_IDLE     = 3'd0;
localparam [2:0] S_SQRT     = 3'd1;
localparam [2:0] S_INNER    = 3'd2;
localparam [2:0] S_SOFTPLUS = 3'd3;
localparam [2:0] S_ED       = 3'd4;
localparam [2:0] S_POWER    = 3'd5;
localparam [2:0] S_IP       = 3'd6;
localparam [2:0] S_DONE     = 3'd7;

reg [2:0] state;

// ============================================================================
// Pipeline Registers
// ============================================================================
reg signed [FP_WIDTH-1:0] vpk_r;       // latched Vpk input
reg signed [FP_WIDTH-1:0] vgk_r;       // latched Vgk input

// Sqrt LUT result + interpolation
reg [KLUT_BITS-1:0] sqrt_idx;
reg signed [FP_WIDTH-1:0] sqrt_lo;     // LUT[idx]
reg signed [FP_WIDTH-1:0] sqrt_hi;     // LUT[idx+1]
reg signed [FP_WIDTH-1:0] sqrt_frac;   // fractional part Q0.16
reg signed [FP_WIDTH-1:0] sqrt_val;    // interpolated result

// Inner computation
reg signed [FP_WIDTH-1:0] inner_val;

// Softplus LUT result + interpolation
reg [KLUT_BITS-1:0] sp_idx;
reg signed [FP_WIDTH-1:0] sp_lo;
reg signed [FP_WIDTH-1:0] sp_hi;
reg signed [FP_WIDTH-1:0] sp_frac;
reg signed [FP_WIDTH-1:0] softplus_val;

// Ed computation
reg signed [FP_WIDTH-1:0] ed_val;

// Power LUT result + interpolation
reg [KLUT_BITS-1:0] pow_idx;
reg signed [FP_WIDTH-1:0] pow_lo;
reg signed [FP_WIDTH-1:0] pow_hi;
reg signed [FP_WIDTH-1:0] pow_frac;
reg signed [FP_WIDTH-1:0] power_val;

// 64-bit intermediates for multiply
reg signed [63:0] tmp64;
reg signed [63:0] tmp64b;
reg signed [63:0] tmp64c;

// ============================================================================
// LUT Index + Fraction Calculator
// Computes: idx = floor((x - x_min) / step), frac = remainder as Q0.16
// ============================================================================

// Helper: compute index and fraction for a 1D LUT lookup
// Returns idx in lower KLUT_BITS, frac in upper 16 bits
function automatic [FP_WIDTH-1:0] lut_index_frac;
    input signed [FP_WIDTH-1:0] x;
    input signed [FP_WIDTH-1:0] x_min;
    input signed [FP_WIDTH-1:0] step;
    reg signed [63:0] offset;
    reg signed [63:0] div_result;
    reg [KLUT_BITS-1:0] idx;
    reg [FP_FRAC-1:0] frac;
    begin
        offset = x - x_min;
        if (offset < 0) begin
            idx = 0;
            frac = 0;
        end else begin
            // offset is in Q16.16, step is in Q16.16
            // We want: idx.frac = offset / step (result is a fixed-point number)
            // Shift offset left by 16 to get extra precision for fraction
            div_result = (offset <<< FP_FRAC) / step;
            if (div_result >= ((KLUT_SIZE - 1) <<< FP_FRAC)) begin
                idx = KLUT_SIZE - 2;  // clamp to second-to-last for interpolation
                frac = {FP_FRAC{1'b1}};  // 0.9999...
            end else begin
                idx = div_result[FP_FRAC + KLUT_BITS - 1 : FP_FRAC];
                frac = div_result[FP_FRAC-1 : 0];
            end
        end
        lut_index_frac = {frac, {(FP_WIDTH - FP_FRAC - KLUT_BITS){1'b0}}, idx};
    end
endfunction

// ============================================================================
// Main State Machine
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state      <= S_IDLE;
        done       <= 1'b0;
        ip_out     <= 0;
        vpk_r      <= 0;
        vgk_r      <= 0;
        sqrt_val   <= 0;
        inner_val  <= 0;
        softplus_val <= 0;
        ed_val     <= 0;
        power_val  <= 0;
    end else begin
        done <= 1'b0;

        case (state)

        // ── Wait for start ──────────────────────────────────────────
        S_IDLE: begin
            if (start) begin
                // Latch inputs, clamp Vpk >= 0
                vpk_r <= (vpk < 0) ? 0 : vpk;
                vgk_r <= vgk;
                state <= S_SQRT;
            end
        end

        // ── Step 1: sqrt LUT lookup ─────────────────────────────────
        // Compute index into sqrt_lut from Vpk
        S_SQRT: begin
            // Compute LUT index and fractional part
            begin
                reg signed [63:0] offset;
                reg signed [63:0] div_r;

                offset = vpk_r - SQRT_VPK_MIN_FP;
                if (offset < 0) offset = 0;

                div_r = (offset <<< FP_FRAC) / SQRT_VPK_STEP_FP;

                if (div_r >= ((KLUT_SIZE - 1) <<< FP_FRAC)) begin
                    sqrt_idx  <= KLUT_SIZE - 2;
                    sqrt_frac <= ONE_FP - 1;
                end else begin
                    sqrt_idx  <= div_r[FP_FRAC + KLUT_BITS - 1 : FP_FRAC];
                    sqrt_frac <= div_r[FP_FRAC-1 : 0];
                end
            end

            state <= S_INNER;
        end

        // ── Step 2: interpolate sqrt, compute inner ─────────────────
        S_INNER: begin
            // Read LUT entries (available after 1 clock from index computation)
            sqrt_lo <= sqrt_lut[sqrt_idx];
            sqrt_hi <= sqrt_lut[sqrt_idx + 1];

            // Interpolate: result = lo + frac * (hi - lo) >> 16
            tmp64 = $signed({1'b0, sqrt_frac}) *
                    ($signed({1'b0, sqrt_lut[sqrt_idx + 1]}) - $signed({1'b0, sqrt_lut[sqrt_idx]}));
            sqrt_val <= $signed({1'b0, sqrt_lut[sqrt_idx]}) + tmp64[47:16];

            // Compute inner = kp * (1/mu + vgk / sqrt_val)
            // Use the interpolated sqrt_val from this cycle
            // vgk_div_sqrt = vgk / sqrt_val (both Q16.16)
            // But we need the interpolated value... use the non-interpolated LUT[idx] as
            // approximation for the division this cycle, refine would add latency
            // Actually, let's compute in next state for correct data dependency
            state <= S_SOFTPLUS;
        end

        // ── Step 3: compute inner, softplus LUT lookup ──────────────
        S_SOFTPLUS: begin
            // Now sqrt_val is valid from S_INNER
            // inner = kp * (1/mu + vgk / sqrt_val)
            // = kp/mu + kp * vgk / sqrt_val
            // First: vgk / sqrt_val (both Q16.16)
            if (sqrt_val != 0) begin
                tmp64 = ($signed(vgk_r) <<< FP_FRAC) / $signed(sqrt_val);
            end else begin
                tmp64 = 0;
            end
            // tmp64 is vgk/sqrt_val in Q16.16

            // inner = kp * (inv_mu + vgk/sqrt_val)
            // = kp * inv_mu + kp * (vgk/sqrt_val)
            tmp64b = $signed(KOREN_KP_FP) * ($signed(KOREN_INV_MU_FP) + tmp64[31:0]);
            inner_val <= tmp64b[47:16];  // Q16.16 result

            // Compute softplus LUT index from inner_val
            // But inner_val won't be ready until next clock...
            // We need to pipeline this: compute index in next state
            state <= S_ED;
        end

        // ── Step 3b: softplus LUT lookup with inner_val ─────────────
        S_ED: begin
            // inner_val is now valid
            // Check piecewise regions
            if (inner_val < SOFTPLUS_MIN_FP) begin
                // softplus ≈ exp(inner) ≈ 0 for inner < -6
                softplus_val <= 0;
            end else if (inner_val > SOFTPLUS_MAX_FP) begin
                // softplus ≈ inner for inner > 6
                softplus_val <= inner_val;
            end else begin
                // LUT region: compute index
                begin
                    reg signed [63:0] sp_offset;
                    reg signed [63:0] sp_div;
                    reg [KLUT_BITS-1:0] sp_i;
                    reg signed [FP_WIDTH-1:0] sp_f;

                    sp_offset = inner_val - SOFTPLUS_MIN_FP;
                    sp_div = (sp_offset <<< FP_FRAC) / SOFTPLUS_STEP_FP;

                    if (sp_div >= ((KLUT_SIZE - 1) <<< FP_FRAC)) begin
                        sp_i = KLUT_SIZE - 2;
                        sp_f = ONE_FP - 1;
                    end else begin
                        sp_i = sp_div[FP_FRAC + KLUT_BITS - 1 : FP_FRAC];
                        sp_f = sp_div[FP_FRAC-1 : 0];
                    end

                    // Interpolate
                    tmp64c = $signed({1'b0, sp_f}) *
                             ($signed({1'b0, softplus_lut[sp_i + 1]}) -
                              $signed({1'b0, softplus_lut[sp_i]}));
                    softplus_val <= $signed({1'b0, softplus_lut[sp_i]}) + tmp64c[47:16];
                end
            end

            state <= S_POWER;
        end

        // ── Step 4: Ed = (vpk/kp) * softplus, power LUT lookup ─────
        S_POWER: begin
            // ed = vpk * inv_kp * softplus (all Q16.16)
            // vpk * inv_kp first
            tmp64 = $signed(vpk_r) * $signed(KOREN_INV_KP_FP);
            // tmp64[47:16] = vpk/kp in Q16.16

            // Then * softplus
            tmp64b = tmp64[47:16] * $signed(softplus_val);
            ed_val <= (tmp64b[47:16] < 0) ? 0 : tmp64b[47:16];

            // Compute power LUT index from ed_val
            // ed_val won't be ready until next clock, pipeline it
            state <= S_IP;
        end

        // ── Step 5: power LUT lookup + Ip computation ───────────────
        S_IP: begin
            // ed_val is now valid
            if (ed_val <= 0) begin
                ip_out <= 0;
            end else if (ed_val >= POWER_ED_MAX_FP) begin
                // Clamp to max LUT value / kg1
                // power_lut[KLUT_SIZE-1] / kg1
                tmp64 = ($signed({1'b0, power_lut[KLUT_SIZE-1]}) <<< FP_FRAC) / $signed(KOREN_KG1_FP);
                ip_out <= tmp64[31:0];
            end else begin
                // LUT lookup with interpolation
                begin
                    reg signed [63:0] p_offset;
                    reg signed [63:0] p_div;
                    reg [KLUT_BITS-1:0] p_i;
                    reg signed [FP_WIDTH-1:0] p_f;
                    reg signed [FP_WIDTH-1:0] p_interp;
                    reg signed [63:0] p_tmp;

                    p_offset = ed_val - POWER_ED_MIN_FP;
                    p_div = (p_offset <<< FP_FRAC) / POWER_ED_STEP_FP;

                    if (p_div >= ((KLUT_SIZE - 1) <<< FP_FRAC)) begin
                        p_i = KLUT_SIZE - 2;
                        p_f = ONE_FP - 1;
                    end else begin
                        p_i = p_div[FP_FRAC + KLUT_BITS - 1 : FP_FRAC];
                        p_f = p_div[FP_FRAC-1 : 0];
                    end

                    // Interpolate power LUT
                    p_tmp = $signed({1'b0, p_f}) *
                            ($signed({1'b0, power_lut[p_i + 1]}) -
                             $signed({1'b0, power_lut[p_i]}));
                    p_interp = $signed({1'b0, power_lut[p_i]}) + p_tmp[47:16];

                    // Ip = power_interp / kg1
                    tmp64 = ($signed(p_interp) <<< FP_FRAC) / $signed(KOREN_KG1_FP);
                    ip_out <= tmp64[31:0];
                end
            end

            done <= 1'b1;
            state <= S_IDLE;
        end

        default: state <= S_IDLE;

        endcase
    end
end

endmodule
