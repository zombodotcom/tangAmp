// ============================================================================
// koren_direct.v
// Hybrid 1D-LUT Koren Triode Equation Evaluator (12AX7)
//
// Computes Ip(Vpk, Vgk) using 3 small 1D LUTs + 1 inv_sqrt LUT with
// linear interpolation instead of a large 2D LUT.
// Total memory: 768 bytes (3 LUTs x 64 entries x 32-bit) vs 32KB.
//
// SINGLE MULTIPLIER ARCHITECTURE: All multiplies go through one registered
// mul_a * mul_b -> mul_out pipeline. One multiply per clock cycle.
// This ensures the synthesizer infers exactly 1 DSP multiplier block.
//
// Pipeline: 11 clocks from start to done.
// Division-free: uses inv_sqrt LUT and reciprocal constants.
// Fixed point: Q16.16 signed 32-bit throughout.
// ============================================================================

(* syn_sharing = "on" *)
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

localparam signed [FP_WIDTH-1:0] KOREN_INV_MU_FP = 32'sd712;       // 1/92.08
localparam signed [FP_WIDTH-1:0] KOREN_INV_KP_FP = 32'sd117;       // 1/561.08
localparam signed [FP_WIDTH-1:0] KOREN_KP_FP     = 32'sd36769996;  // 561.08

localparam signed [FP_WIDTH-1:0] SOFTPLUS_MIN_FP  = -32'sd393216;  // -6.0
localparam signed [FP_WIDTH-1:0] SOFTPLUS_MAX_FP  = 32'sd393216;   // +6.0

localparam signed [FP_WIDTH-1:0] POWER_ED_MAX_FP  = 32'sd39322;    // 0.6

localparam signed [FP_WIDTH-1:0] ONE_FP = 32'sd65536;

// Reciprocal constants for division-free indexing
// index_fp = (offset * INV) >>> 16  (offset is Q16.16, INV is integer)
localparam signed [FP_WIDTH-1:0] INV_SQRT_STEP    = 32'sd13765;    // 2^32 / 312015
localparam signed [FP_WIDTH-1:0] INV_SOFTPLUS_STEP = 32'sd344065;  // 2^32 / 12483
localparam signed [FP_WIDTH-1:0] INV_POWER_STEP   = 32'sd6882960;  // 2^32 / 624
localparam signed [FP_WIDTH-1:0] INV_KG1          = 32'sd3292405;  // 2^48 / 85492203

// ============================================================================
// 1D LUT Memory
// ============================================================================
reg [FP_WIDTH-1:0] inv_sqrt_lut  [0:KLUT_SIZE-1];
reg [FP_WIDTH-1:0] softplus_lut  [0:KLUT_SIZE-1];
reg [FP_WIDTH-1:0] power_lut     [0:KLUT_SIZE-1];

initial begin
    $readmemh("data/inv_sqrt_lut.hex",  inv_sqrt_lut);
    $readmemh("data/softplus_lut.hex",  softplus_lut);
    $readmemh("data/power_lut.hex",     power_lut);
end

// ============================================================================
// SINGLE SHARED MULTIPLIER
// ============================================================================
reg  signed [FP_WIDTH-1:0] mul_a;
reg  signed [FP_WIDTH-1:0] mul_b;
wire signed [63:0]         mul_out;
assign mul_out = $signed(mul_a) * $signed(mul_b);

// ============================================================================
// State Machine
// ============================================================================
localparam [3:0] S_IDLE     = 4'd0;
localparam [3:0] S1_IDX     = 4'd1;   // inv_sqrt index multiply
localparam [3:0] S2_LERP    = 4'd2;   // inv_sqrt lerp multiply
localparam [3:0] S3_VGK     = 4'd3;   // vgk * inv_sqrt
localparam [3:0] S4_KP      = 4'd4;   // kp * (inv_mu + vgk/sqrt)
localparam [3:0] S5_SPIDX   = 4'd5;   // softplus index multiply
localparam [3:0] S6_SPLERP  = 4'd6;   // softplus lerp multiply
localparam [3:0] S7_ED1     = 4'd7;   // vpk * inv_kp
localparam [3:0] S8_ED2     = 4'd8;   // (vpk/kp) * softplus
localparam [3:0] S9_PIDX    = 4'd9;   // power index multiply
localparam [3:0] S10_IP     = 4'd10;  // power_interp * inv_kg1

reg [3:0] state;

// Pipeline registers
reg signed [FP_WIDTH-1:0] vpk_r, vgk_r;
reg signed [FP_WIDTH-1:0] inv_sqrt_val;
reg signed [FP_WIDTH-1:0] inner_val;
reg signed [FP_WIDTH-1:0] softplus_val;
reg signed [FP_WIDTH-1:0] vpk_div_kp;
reg signed [FP_WIDTH-1:0] ed_val;
reg        skip_to_ed;  // flag for softplus saturation shortcut

// ============================================================================
// Helper: extract LUT index and fraction from index_fp (Q16.16)
// ============================================================================
// (done inline in each state to avoid extra logic)

// ============================================================================
// Main State Machine
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state        <= S_IDLE;
        done         <= 1'b0;
        ip_out       <= 0;
        vpk_r        <= 0;
        vgk_r        <= 0;
        mul_a        <= 0;
        mul_b        <= 0;
        inv_sqrt_val <= 0;
        inner_val    <= 0;
        softplus_val <= 0;
        vpk_div_kp   <= 0;
        ed_val       <= 0;
        skip_to_ed   <= 0;
    end else begin
        done <= 1'b0;

        case (state)

        S_IDLE: begin
            if (start) begin
                vpk_r <= (vpk < 0) ? 0 : vpk;
                vgk_r <= vgk;
                skip_to_ed <= 0;
                // Setup S1: index = vpk * INV_SQRT_STEP
                mul_a <= (vpk < 0) ? 0 : vpk;
                mul_b <= INV_SQRT_STEP;
                state <= S1_IDX;
            end
        end

        // ── S1: Read inv_sqrt index multiply result ─────────────────
        S1_IDX: begin
            // mul_out = vpk * INV_SQRT_STEP; index_fp = mul_out >>> 16
            begin
                reg signed [63:0] idx_fp;
                reg [KLUT_BITS-1:0] idx;
                reg signed [FP_WIDTH-1:0] frac;
                reg signed [FP_WIDTH-1:0] lo_val, hi_val;

                idx_fp = mul_out >>> 16;

                if (idx_fp >= ((KLUT_SIZE - 1) <<< FP_FRAC)) begin
                    idx  = KLUT_SIZE - 2;
                    frac = ONE_FP - 1;
                end else if (idx_fp < 0) begin
                    idx  = 0;
                    frac = 0;
                end else begin
                    idx  = idx_fp[FP_FRAC + KLUT_BITS - 1 : FP_FRAC];
                    frac = idx_fp[FP_FRAC-1 : 0];
                end

                lo_val = $signed({1'b0, inv_sqrt_lut[idx]});
                hi_val = $signed({1'b0, inv_sqrt_lut[idx + 1]});

                // Store base for lerp addition
                inv_sqrt_val <= lo_val;

                // Setup S2: lerp = frac * (hi - lo)
                mul_a <= frac;
                mul_b <= hi_val - lo_val;
            end
            state <= S2_LERP;
        end

        // ── S2: Apply inv_sqrt lerp, setup vgk multiply ────────────
        S2_LERP: begin
            // inv_sqrt_val = lo + (frac * (hi-lo)) >> 16
            inv_sqrt_val <= inv_sqrt_val + mul_out[47:16];

            // Setup S3: vgk * inv_sqrt_val (corrected)
            mul_a <= vgk_r;
            mul_b <= inv_sqrt_val + mul_out[47:16];
            state <= S3_VGK;
        end

        // ── S3: Read vgk*inv_sqrt, setup kp multiply ───────────────
        S3_VGK: begin
            // vgk_div_sqrt = mul_out[47:16] (Q16.16)

            // Setup S4: kp * (inv_mu + vgk/sqrt)
            mul_a <= KOREN_KP_FP;
            mul_b <= KOREN_INV_MU_FP + mul_out[47:16];
            state <= S4_KP;
        end

        // ── S4: Read inner, decide softplus path ────────────────────
        S4_KP: begin
            // inner = mul_out[47:16]
            inner_val <= mul_out[47:16];

            if (mul_out[47:16] < SOFTPLUS_MIN_FP) begin
                // softplus ~ 0 for very negative inner -> Ip = 0
                softplus_val <= 0;
                skip_to_ed <= 1;
                // Setup S7: vpk * inv_kp (skip to Ed computation)
                mul_a <= vpk_r;
                mul_b <= KOREN_INV_KP_FP;
                state <= S7_ED1;
            end else if (mul_out[47:16] > SOFTPLUS_MAX_FP) begin
                // softplus ~ inner for large positive inner
                softplus_val <= mul_out[47:16];
                skip_to_ed <= 1;
                mul_a <= vpk_r;
                mul_b <= KOREN_INV_KP_FP;
                state <= S7_ED1;
            end else begin
                // Normal softplus LUT region
                // Setup S5: sp_offset * INV_SOFTPLUS_STEP
                mul_a <= mul_out[47:16] - SOFTPLUS_MIN_FP;
                mul_b <= INV_SOFTPLUS_STEP;
                state <= S5_SPIDX;
            end
        end

        // ── S5: Read softplus index, setup lerp ─────────────────────
        S5_SPIDX: begin
            begin
                reg signed [63:0] idx_fp;
                reg [KLUT_BITS-1:0] idx;
                reg signed [FP_WIDTH-1:0] frac;
                reg signed [FP_WIDTH-1:0] lo_val, hi_val;

                idx_fp = mul_out >>> 16;

                if (idx_fp >= ((KLUT_SIZE - 1) <<< FP_FRAC)) begin
                    idx  = KLUT_SIZE - 2;
                    frac = ONE_FP - 1;
                end else if (idx_fp < 0) begin
                    idx  = 0;
                    frac = 0;
                end else begin
                    idx  = idx_fp[FP_FRAC + KLUT_BITS - 1 : FP_FRAC];
                    frac = idx_fp[FP_FRAC-1 : 0];
                end

                lo_val = $signed({1'b0, softplus_lut[idx]});
                hi_val = $signed({1'b0, softplus_lut[idx + 1]});

                softplus_val <= lo_val;

                // Setup S6: lerp
                mul_a <= frac;
                mul_b <= hi_val - lo_val;
            end
            state <= S6_SPLERP;
        end

        // ── S6: Apply softplus lerp, setup Ed mul #1 ────────────────
        S6_SPLERP: begin
            softplus_val <= softplus_val + mul_out[47:16];

            // Setup S7: vpk * inv_kp
            mul_a <= vpk_r;
            mul_b <= KOREN_INV_KP_FP;
            state <= S7_ED1;
        end

        // ── S7: Read vpk/kp, setup Ed mul #2 ───────────────────────
        S7_ED1: begin
            vpk_div_kp <= mul_out[47:16];

            // Setup S8: (vpk/kp) * softplus
            mul_a <= mul_out[47:16];
            mul_b <= softplus_val;
            state <= S8_ED2;
        end

        // ── S8: Read Ed, decide power path ──────────────────────────
        S8_ED2: begin
            // ed = mul_out[47:16], clamped >= 0
            ed_val <= (mul_out[47:16] < 0) ? 0 : mul_out[47:16];

            if (mul_out[47:16] <= 0) begin
                ip_out <= 0;
                done   <= 1'b1;
                state  <= S_IDLE;
            end else if (mul_out[47:16] >= POWER_ED_MAX_FP) begin
                // Clamp to max: setup power_max * inv_kg1
                mul_a <= $signed({1'b0, power_lut[KLUT_SIZE-1]});
                mul_b <= INV_KG1;
                state <= S10_IP;
            end else begin
                // Power LUT: compute index
                mul_a <= mul_out[47:16];  // ed_val
                mul_b <= INV_POWER_STEP;
                state <= S9_PIDX;
            end
        end

        // ── S9: Read power index, lookup + setup Ip multiply ────────
        S9_PIDX: begin
            begin
                reg signed [63:0] idx_fp;
                reg [KLUT_BITS-1:0] idx;
                reg signed [FP_WIDTH-1:0] frac;
                reg signed [FP_WIDTH-1:0] lo_val, hi_val, interp;

                idx_fp = mul_out >>> 16;

                if (idx_fp >= ((KLUT_SIZE - 1) <<< FP_FRAC)) begin
                    idx  = KLUT_SIZE - 2;
                    frac = ONE_FP - 1;
                end else if (idx_fp < 0) begin
                    idx  = 0;
                    frac = 0;
                end else begin
                    idx  = idx_fp[FP_FRAC + KLUT_BITS - 1 : FP_FRAC];
                    frac = idx_fp[FP_FRAC-1 : 0];
                end

                lo_val = $signed({1'b0, power_lut[idx]});
                hi_val = $signed({1'b0, power_lut[idx + 1]});

                // Nearest-neighbor interpolation (saves 1 multiply cycle)
                // Max error from skipping lerp: ~0.5 * step = ~0.005 in Ed^1.29
                interp = (frac >= (ONE_FP >>> 1)) ? hi_val : lo_val;

                // Setup S10: interp * inv_kg1
                mul_a <= interp;
                mul_b <= INV_KG1;
            end
            state <= S10_IP;
        end

        // ── S10: Read Ip result ─────────────────────────────────────
        S10_IP: begin
            // Ip = (power_interp * INV_KG1) >>> 32
            // INV_KG1 = 2^48 / kg1_fp, so:
            //   mul_out = power_interp * (2^48 / kg1_fp)
            //   Ip_q16 = mul_out >>> 32 = power_interp / kg1_fp * 2^16
            ip_out <= mul_out[63:32];
            done   <= 1'b1;
            state  <= S_IDLE;
        end

        default: state <= S_IDLE;

        endcase
    end
end

endmodule
