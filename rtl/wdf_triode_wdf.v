// ============================================================================
// wdf_triode_wdf.v
// WDF-based Single Common-Cathode Triode Stage (12AX7)
//
// Ported from validated Python WDF simulation (wdf_triode_sim_wdf.py).
// Uses Newton-Raphson root finding with 2D LUTs for tube nonlinearity.
//
// Algorithm per sample:
//   1. High-pass coupling filter (Cin + Rg)
//   2. Upward pass: leaf reflected waves
//   3. Newton-Raphson (3 iterations) using ip/dip LUTs + numerical dIp/dVpk
//   4. Extract plate voltage via load line, AC-couple output
//
// Fixed point: Q16.16 signed throughout
// Sample rate: 48kHz
// Clock:       27MHz (Tang Nano 20K)
// ============================================================================

module wdf_triode_wdf #(
    parameter FP_FRAC  = 16,
    parameter FP_WIDTH = 32,
    parameter LUT_BITS = 7,
    parameter LUT_SIZE = 128,
    parameter IP_SCALE = 10000,
    parameter DIP_SCALE = 100000,
    parameter DIP_VPK_SCALE = 10000000,
    parameter integer VPK_MIN_MV = 0,
    parameter integer VPK_MAX_MV = 300000,
    parameter integer VGK_MIN_MV = -4000,
    parameter integer VGK_MAX_MV = 0
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sample_en,
    input  wire signed [FP_WIDTH-1:0] audio_in,
    output reg  signed [FP_WIDTH-1:0] audio_out,
    output reg                        out_valid
);

// ============================================================================
// Constants
// ============================================================================

// B+ supply voltage: 200V in Q16.16
localparam signed [FP_WIDTH-1:0] VB_FP = 200 * (1 << FP_FRAC);  // 13,107,200

// Resistor values as plain integers (NOT Q16.16)
localparam integer RP_INT  = 100000;   // Plate resistor (ohms)
localparam integer RK_INT  = 1500;     // Cathode resistor (ohms)
localparam integer RPK_INT = 100000;   // RP + R_cathode (R_cathode ≈ 0.47 ohm, negligible)

// High-pass filter coefficients (Q0.16, stored in lower 16 bits)
// c_hp  = 0.999054 -> round(0.999054 * 65536) = 65474
// hp_gain = 0.999527 -> round(0.999527 * 65536) = 65505
localparam signed [FP_WIDTH-1:0] C_HP    = 32'sd65474;
localparam signed [FP_WIDTH-1:0] HP_GAIN = 32'sd65505;

// 1.0 in Q16.16
localparam signed [FP_WIDTH-1:0] ONE_FP = 32'sd65536;

// Cathode parallel adaptor: Rk(1500) || Ck(22uF)
// R_Ck = 1/(2*48000*22e-6) = 0.4735 ohm
// R_cathode = 1/(1/1500 + 1/0.4735) = 0.47319 ohm
// gamma_cath = (1/1500) / (1/1500 + 1/0.4735) = 0.000316
// In Q16.16: round(0.000316 * 65536) = 20
localparam signed [FP_WIDTH-1:0] GAMMA_CATH = 32'sd20;
// 2*R_cathode = 0.947 ohm. In Q16.16: round(0.947 * 65536) = 62041
localparam signed [FP_WIDTH-1:0] TWO_R_CATH_FP = 32'sd62041;

// DC operating point estimate: ~146V in Q16.16
// Will be refined by exponential moving average
localparam signed [FP_WIDTH-1:0] VP_DC_INIT = 146 * (1 << FP_FRAC);  // 9,568,256

// DC tracking: EMA with alpha = 1/256 (shift by 8)
localparam DC_SHIFT = 8;
localparam DC_SETTLE_SAMPLES = 10000;

// ============================================================================
// LUT Memory
// ============================================================================

reg signed [15:0] ip_lut      [0 : LUT_SIZE*LUT_SIZE - 1];
reg signed [15:0] dip_vgk_lut [0 : LUT_SIZE*LUT_SIZE - 1];
// dip_vpk_lut removed — dIp/dVpk LUT dropped to fit BSRAM (matches triode_engine.v)

initial begin
    $readmemh("data/ip_lut.hex",       ip_lut);
    $readmemh("data/dip_dvgk_lut.hex", dip_vgk_lut);
end

// Grid current LUTs (64 entries, Vgk 0-2V, distributed RAM)
localparam IG_LUT_SIZE = 64;
localparam IG_LUT_BITS = 6;
localparam signed [FP_WIDTH-1:0] IG_VGK_MAX_FP = 32'sd131072;  // 2.0V in Q16.16
localparam signed [FP_WIDTH-1:0] IG_VGK_STEP_FP = 32'sd2080;   // step in Q16.16
localparam integer RG_INT = 1000000;  // Grid resistor (ohms)

reg signed [15:0] ig_lut  [0:IG_LUT_SIZE-1];
reg signed [15:0] dig_lut [0:IG_LUT_SIZE-1];

initial begin
    $readmemh("data/ig_lut.hex", ig_lut);
    $readmemh("data/dig_lut.hex", dig_lut);
end

// ============================================================================
// LUT Address Functions (copied from wdf_triode.v)
// ============================================================================

function automatic [LUT_BITS-1:0] vpk_to_idx;
    input signed [FP_WIDTH-1:0] vpk_fp;
    reg signed [63:0] tmp;
    begin
        tmp = (vpk_fp * 1000) >>> FP_FRAC;
        if (tmp < VPK_MIN_MV) tmp = VPK_MIN_MV;
        if (tmp > VPK_MAX_MV) tmp = VPK_MAX_MV;
        vpk_to_idx = ((tmp - VPK_MIN_MV) * (LUT_SIZE-1)) / (VPK_MAX_MV - VPK_MIN_MV);
    end
endfunction

function automatic [LUT_BITS-1:0] vgk_to_idx;
    input signed [FP_WIDTH-1:0] vgk_fp;
    reg signed [63:0] tmp;
    begin
        tmp = (vgk_fp * 1000) >>> FP_FRAC;
        if (tmp < VGK_MIN_MV) tmp = VGK_MIN_MV;
        if (tmp > VGK_MAX_MV) tmp = VGK_MAX_MV;
        vgk_to_idx = ((tmp - VGK_MIN_MV) * (LUT_SIZE-1)) / (VGK_MAX_MV - VGK_MIN_MV);
    end
endfunction

function automatic [IG_LUT_BITS-1:0] vgk_to_ig_idx;
    input signed [FP_WIDTH-1:0] vgk_fp;
    reg signed [63:0] tmp;
    begin
        if (vgk_fp <= 0)
            vgk_to_ig_idx = 0;
        else if (vgk_fp >= IG_VGK_MAX_FP)
            vgk_to_ig_idx = IG_LUT_SIZE - 1;
        else begin
            tmp = ($signed({vgk_fp, 16'b0})) / $signed(IG_VGK_STEP_FP);
            vgk_to_ig_idx = tmp[IG_LUT_BITS-1:0];
        end
    end
endfunction

// ============================================================================
// State Machine
// ============================================================================

localparam ST_IDLE    = 3'd0;
localparam ST_HP      = 3'd1;
localparam ST_NR_ADDR = 3'd2;
localparam ST_NR_READ = 3'd3;
localparam ST_NR_CONV = 3'd4;
localparam ST_NR_STEP = 3'd5;
localparam ST_OUTPUT  = 3'd6;

reg [2:0] state;

// ============================================================================
// State Registers
// ============================================================================

// High-pass filter state
reg signed [FP_WIDTH-1:0] hp_prev_in;
reg signed [FP_WIDTH-1:0] hp_prev_out;

// Grid voltage after coupling cap
reg signed [FP_WIDTH-1:0] vgrid;

// Cathode bypass capacitor state
reg signed [FP_WIDTH-1:0] ck_z;         // Bypass cap state (WDF delay element)
reg signed [FP_WIDTH-1:0] b_cathode;    // Cathode adaptor upward reflected wave

// Newton-Raphson state
reg signed [FP_WIDTH-1:0] ip_est;       // Current Ip estimate (Q16.16)
reg signed [FP_WIDTH-1:0] ip_prev;      // Previous sample's converged Ip
reg signed [FP_WIDTH-1:0] vpk_est;      // Vpk estimate (Q16.16)
reg signed [FP_WIDTH-1:0] vgk_est;      // Vgk estimate (Q16.16)
reg [1:0] newton_iter;                   // Iteration counter (0..2, 3 iterations)

// LUT address and raw values
reg [LUT_BITS*2-1:0] lut_addr;
reg signed [15:0] ip_raw;
reg signed [15:0] dip_vgk_raw;

// Converted Q16.16 LUT values
reg signed [FP_WIDTH-1:0] ip_model;
reg signed [FP_WIDTH-1:0] dip_vgk_val;

// DC operating point tracking
reg signed [FP_WIDTH-1:0] vp_dc;
reg [15:0] sample_count;
reg        dc_frozen;

// 64-bit intermediates
reg signed [63:0] temp64;
reg signed [63:0] temp64b;
reg signed [63:0] hp_temp;
reg signed [63:0] hp_temp2;

// Newton step intermediates
reg signed [FP_WIDTH-1:0] f_val;
reg signed [FP_WIDTH-1:0] fp_val;
reg signed [63:0] temp_a;
reg signed [63:0] temp_b;
reg signed [47:0] step_num;
reg signed [FP_WIDTH-1:0] step;
reg signed [FP_WIDTH-1:0] step_ig;

// Secant method registers for J11 estimation
reg signed [FP_WIDTH-1:0] f1_prev_iter;   // f1 from previous Newton iteration
reg signed [FP_WIDTH-1:0] ip_prev_iter;   // ip_est from previous Newton iteration
reg signed [63:0] j11_saved;              // J11 estimated by secant, persisted across samples

// 2x2 Newton solver registers
reg signed [FP_WIDTH-1:0] ig_est;
reg signed [FP_WIDTH-1:0] ig_prev;
reg signed [15:0] ig_raw;
reg signed [15:0] dig_raw;
reg signed [FP_WIDTH-1:0] f1_val;
reg signed [FP_WIDTH-1:0] f2_val;
reg signed [63:0] j11, j12, j21, j22;
reg signed [63:0] det;
reg signed [63:0] dIp_num, dIg_num;

// ============================================================================
// Main State Machine
// ============================================================================

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state       <= ST_IDLE;
        out_valid   <= 1'b0;
        audio_out   <= 0;
        hp_prev_in  <= 0;
        hp_prev_out <= 0;
        vgrid       <= 0;
        // Initialize bypass cap near DC equilibrium: Vk ≈ Ip*Rk ≈ 0.81V
        // ck_z_eq ≈ R_cath * Ip_eq / gamma ≈ 53000 in Q16.16
        ck_z        <= 32'sd53000;
        b_cathode   <= 32'sd53000;
        ip_est      <= 0;
        ip_prev     <= 32'sd35;        // ~0.54mA quiescent in Q16.16
        vpk_est     <= 0;
        vgk_est     <= 0;
        newton_iter <= 0;
        lut_addr    <= 0;
        ip_raw      <= 0;
        dip_vgk_raw <= 0;
        ip_model    <= 0;
        dip_vgk_val <= 0;
        vp_dc       <= VP_DC_INIT;
        sample_count <= 0;
        dc_frozen   <= 0;
        f_val       <= 0;
        fp_val      <= 0;
        step        <= 0;
        ig_est      <= 0;
        ig_prev     <= 0;
        ig_raw      <= 0;
        dig_raw     <= 0;
        f1_val      <= 0;
        f2_val      <= 0;
        f1_prev_iter <= 0;
        ip_prev_iter <= 0;
        j11_saved    <= ONE_FP <<< 1;  // initial J11 = 2.0
    end else begin
        out_valid <= 1'b0;

        case (state)

        // ── Wait for sample_en ──────────────────────────────────────────
        ST_IDLE: begin
            if (sample_en) begin
                // Initialize Ip/Ig estimates from previous sample's converged values
                ip_est <= ip_prev;
                ig_est <= ig_prev;
                newton_iter <= 0;
                state <= ST_HP;
            end
        end

        // ── High-pass coupling filter ───────────────────────────────────
        // Two-coefficient form:
        //   hp_y = c_hp * hp_prev_out + hp_gain * (audio_in - hp_prev_in)
        ST_HP: begin
            // c_hp * hp_prev_out: Q0.16 * Q16.16 -> take bits [47:16]
            hp_temp  = $signed(C_HP) * $signed(hp_prev_out);
            // hp_gain * (audio_in - hp_prev_in): Q0.16 * Q16.16 -> take bits [47:16]
            hp_temp2 = $signed(HP_GAIN) * $signed(audio_in - hp_prev_in);

            vgrid <= hp_temp[47:16] + hp_temp2[47:16];

            // Update filter state
            hp_prev_out <= hp_temp[47:16] + hp_temp2[47:16];
            hp_prev_in  <= audio_in;

            // Cathode parallel adaptor upward pass: Rk || Ck
            // b_rk = 0 (resistor), b_ck = ck_z (capacitor state)
            // bDiff = b_ck - b_rk = ck_z
            // b_cathode = b_ck - gamma * bDiff = ck_z - gamma * ck_z
            temp64 = $signed(GAMMA_CATH) * $signed(ck_z);
            b_cathode <= ck_z - temp64[47:16];

            state <= ST_NR_ADDR;
        end

        // ── Newton-Raphson: compute Vpk, Vgk, build LUT address ────────
        ST_NR_ADDR: begin
            // With cathode bypass cap, cathode port resistance R_cathode ≈ 0.47 ohm
            // Vpk = (VB - b_cathode) - (RP + R_cathode) * Ip ≈ (VB - b_cathode) - RP * Ip
            temp64 = $signed(RPK_INT) * $signed(ip_est);
            vpk_est <= (VB_FP - b_cathode) - temp64[31:0];

            // Vgk = vgrid - b_cathode (R_cathode * Ip is negligible)
            vgk_est <= vgrid - b_cathode;

            // Build LUT address from Vpk and Vgk estimates
            lut_addr <= vpk_to_idx((VB_FP - b_cathode) - temp64[31:0]) * LUT_SIZE
                      + vgk_to_idx(vgrid - b_cathode);

            state <= ST_NR_READ;
        end

        // ── BRAM read latency (1 clock) ─────────────────────────────────
        ST_NR_READ: begin
            ip_raw      <= ip_lut[lut_addr];
            dip_vgk_raw <= dip_vgk_lut[lut_addr];
            // Grid current LUT read
            ig_raw  <= ig_lut[vgk_to_ig_idx(vgk_est)];
            dig_raw <= dig_lut[vgk_to_ig_idx(vgk_est)];
            state <= ST_NR_CONV;
        end

        // ── Convert LUT raw int16 to Q16.16 ────────────────────────────
        // ip_raw = Ip * IP_SCALE -> ip_model = (ip_raw << 16) / IP_SCALE
        // dip_vgk_raw = dIp/dVgk * DIP_SCALE -> dip_vgk_val = (raw << 16) / DIP_SCALE
        // dip_vpk_raw = dIp/dVpk * DIP_VPK_SCALE -> dip_vpk_val = (raw << 16) / DIP_VPK_SCALE
        ST_NR_CONV: begin
            ip_model    <= ($signed(ip_raw) <<< FP_FRAC) / IP_SCALE;
            dip_vgk_val <= ($signed(dip_vgk_raw) <<< FP_FRAC) / DIP_SCALE;
            state <= ST_NR_STEP;
        end

        // ── 2x2 Newton step (Ip and Ig coupled) ────────────────────────
        // f1 = Ip_est - Ip_model(Vpk, Vgk)
        // f2 = Ig_est - Ig_model(Vgk)
        // Jacobian: J = [[J11, J12], [J21, J22]]
        // [dIp; dIg] = J^-1 * [f1; f2]
        ST_NR_STEP: begin
            // Residuals
            f1_val = ip_est - ip_model;
            f2_val = ig_est - $signed({{16{ig_raw[15]}}, ig_raw});

            // J11 = 1 + dIp/dVpk * RPK
            // Secant method: estimate J11 from consecutive Newton iterations.
            // iter 0: use j11_saved (persisted from previous sample's secant).
            // iter 1+: compute secant from consecutive iteration values,
            //          and update j11_saved for use in next sample's iter 0.
            if (newton_iter == 0) begin
                // Use J11 estimated by secant from previous sample
                j11 = j11_saved;
            end else begin
                // Secant approximation: df1/dIp from consecutive iterations
                // delta_f1 and delta_ip use temp registers (temp_a reused below)
                if ((ip_est - ip_prev_iter) != 0) begin
                    j11 = ($signed({f1_val - f1_prev_iter, 16'b0})) / $signed(ip_est - ip_prev_iter);
                    // Clamp J11 to reasonable range [0.5, 4.0] in Q16.16
                    if (j11 < 32'sd32768)  j11 = 32'sd32768;   // min 0.5
                    if (j11 > 32'sd262144) j11 = 32'sd262144;  // max 4.0
                    // Persist this secant estimate for next sample's iter 0
                    j11_saved <= j11;
                end else begin
                    j11 = j11_saved;  // no change, reuse previous estimate
                end
            end

            // Store current values for next iteration's secant
            f1_prev_iter <= f1_val;
            ip_prev_iter <= ip_est;

            // J12 = dIp/dVgk * Rg
            temp_b = ($signed(dip_vgk_raw) * $signed(RG_INT)) <<< FP_FRAC;
            j12 = temp_b / DIP_SCALE;

            // J21 = dIg/dVgk * Rk ≈ 0
            j21 = 0;

            // J22 = 1 + dIg/dVgk * (Rg + Rk) ≈ 1 + dIg/dVgk * Rg
            temp_a = ($signed({{16{dig_raw[15]}}, dig_raw}) * $signed(RG_INT));
            j22 = ONE_FP + temp_a[47:16];

            // det = J11*J22 - J12*J21 (J21=0, so det = J11*J22)
            det = (j11 * j22) >>> FP_FRAC;

            // dIp = (J22*f1 - J12*f2) / det
            dIp_num = (j22 * $signed(f1_val) - j12 * $signed(f2_val)) >>> FP_FRAC;

            // dIg = (J11*f2 - J21*f1) / det (J21=0)
            dIg_num = (j11 * $signed(f2_val)) >>> FP_FRAC;

            // Apply Newton step
            if (det != 0) begin
                step = (dIp_num <<< FP_FRAC) / det;
                ip_est <= ip_est - step;
                if ((ip_est - step) < 0) ip_est <= 0;

                step_ig = (dIg_num <<< FP_FRAC) / det;
                ig_est <= ig_est - step_ig;
                if ((ig_est - step_ig) < 0) ig_est <= 0;
            end

            // 3 Newton iterations (was 2): iter 0, 1, 2
            if (newton_iter < 2'd2) begin
                newton_iter <= newton_iter + 1;
                state <= ST_NR_ADDR;  // Loop back for next iteration
            end else begin
                state <= ST_OUTPUT;   // Done with Newton iterations
            end
        end

        // ── Compute output using converged Ip and Ig ────────────────────
        ST_OUTPUT: begin
            // v_plate = VB - RP * Ip (only plate current through plate resistor)
            temp64 = $signed(RP_INT) * $signed(ip_est);
            audio_out <= (VB_FP - temp64[31:0]) - vp_dc;
            out_valid <= 1'b1;

            // Save converged values for next sample
            ip_prev <= ip_est;
            ig_prev <= ig_est;

            // Cathode downward pass: total current = Ip + Ig
            begin
                reg signed [FP_WIDTH-1:0] ip_total;
                ip_total = ip_est + ig_est;
                temp64b = $signed(TWO_R_CATH_FP) * $signed(ip_total);
            end
            ck_z <= (b_cathode + temp64b[47:16]) + b_cathode - ck_z;

            // DC tracking
            if (!dc_frozen) begin
                vp_dc <= vp_dc + (((VB_FP - temp64[31:0]) - vp_dc) >>> DC_SHIFT);
                if (sample_count < DC_SETTLE_SAMPLES)
                    sample_count <= sample_count + 1;
                else
                    dc_frozen <= 1'b1;
            end

            state <= ST_IDLE;
        end

        default: state <= ST_IDLE;

        endcase
    end
end

endmodule
