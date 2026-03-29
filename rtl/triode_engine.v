// ============================================================================
// triode_engine.v
// Time-multiplexed WDF triode processing engine.
//
// Preamp stages use koren_direct 1D LUT evaluator (384 bytes total) instead
// of 2D LUTs (32KB each). Numerical derivative via two koren_direct calls.
// Power amp stage still uses 2D LUTs (separate voltage range).
//
// Clock budget per preamp stage: 16 clocks/iteration x 2 iterations = 32
// Clock budget per power amp stage: 4 clocks/iteration x 3 iterations = 12
// Total (2 preamp + 1 PA, 2x oversample): (32*2 + 12)*2 + overhead = ~172
//
// Fixed point: Q16.16 signed throughout
// ============================================================================

module triode_engine #(
    parameter NUM_STAGES  = 2,        // preamp stages (12AX7)
    parameter POWER_AMP   = 1,        // 1 = add 6L6 power amp as final stage
    parameter FP_FRAC     = 16,
    parameter FP_WIDTH    = 32,
    parameter LUT_BITS    = 7,
    parameter LUT_SIZE    = 128,
    parameter IP_SCALE    = 10000,
    parameter DIP_SCALE   = 100000,
    parameter DIP_VPK_SCALE = 10000000,
    // 12AX7 preamp ranges (used only for power amp LUT indexing now)
    parameter integer VPK_MIN_MV = 0,
    parameter integer VPK_MAX_MV = 300000,
    parameter integer VGK_MIN_MV = -4000,
    parameter integer VGK_MAX_MV = 0,
    // 6L6 power amp ranges
    parameter integer VPK_MIN_MV_PA = 0,
    parameter integer VPK_MAX_MV_PA = 500000,
    parameter integer VGK_MIN_MV_PA = -50000,
    parameter integer VGK_MAX_MV_PA = 0
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sample_en,
    input  wire signed [FP_WIDTH-1:0] audio_in,
    output reg  signed [FP_WIDTH-1:0] audio_out,
    output reg         out_valid
);

// ============================================================================
// Constants (same as wdf_triode_wdf.v)
// ============================================================================

// 12AX7 preamp constants
localparam signed [FP_WIDTH-1:0] VB_FP_PRE = 200 * (1 << FP_FRAC);
localparam integer RP_INT_PRE  = 100000;
localparam integer RPK_INT_PRE = 100000;  // RP + R_cathode (R_cath ~ 0)

// 6L6 power amp constants (triode-connected)
localparam signed [FP_WIDTH-1:0] VB_FP_PA = 400 * (1 << FP_FRAC);
localparam integer RP_INT_PA  = 2000;     // transformer primary
localparam integer RPK_INT_PA = 2000;     // RP + R_cathode

// Total stages including power amp
localparam TOTAL_STAGES = NUM_STAGES + POWER_AMP;

// Active constants (MUXed by is_power_amp)
reg signed [FP_WIDTH-1:0] vb_active;
reg signed [31:0] rp_active;
reg signed [31:0] rpk_active;
wire is_power_amp = POWER_AMP && (current_stage == TOTAL_STAGES - 1);

localparam signed [FP_WIDTH-1:0] C_HP    = 32'sd65474;
localparam signed [FP_WIDTH-1:0] HP_GAIN = 32'sd65505;
localparam signed [FP_WIDTH-1:0] ONE_FP  = 32'sd65536;
localparam signed [FP_WIDTH-1:0] GAMMA_CATH = 32'sd20;
localparam signed [FP_WIDTH-1:0] TWO_R_CATH_FP = 32'sd62041;
localparam signed [FP_WIDTH-1:0] VP_DC_INIT = 146 * (1 << FP_FRAC);
localparam DC_SHIFT = 8;
localparam DC_SETTLE_SAMPLES = 10000;

// Numerical derivative step: h = 0.01V = 655 in Q16.16
// dIp/dVgk ~= (Ip(Vpk, Vgk+h) - Ip(Vpk, Vgk)) / h
// In Q16.16: result = (ip2 - ip1) * INV_H where INV_H = round(2^16 / 655) = 100
localparam signed [FP_WIDTH-1:0] DERIV_H_FP = 32'sd655;  // 0.01V
localparam signed [FP_WIDTH-1:0] INV_H      = 32'sd100;  // 1/h in Q16.16 units

// ============================================================================
// LUT Memory -- only power amp 2D LUTs remain (preamp uses koren_direct)
// ============================================================================

// 6L6 power amp LUTs (kept -- different voltage range from 12AX7)
reg signed [15:0] ip_lut_pa      [0 : LUT_SIZE*LUT_SIZE - 1];
reg signed [15:0] dip_vgk_lut_pa [0 : LUT_SIZE*LUT_SIZE - 1];

initial begin
    $readmemh("data/ip_lut_6l6.hex",       ip_lut_pa);
    $readmemh("data/dip_dvgk_lut_6l6.hex", dip_vgk_lut_pa);
end


// Grid current LUTs (shared across all stages, tiny -- 64 entries each)
localparam IG_LUT_SIZE = 64;
localparam IG_LUT_BITS = 6;
localparam signed [FP_WIDTH-1:0] IG_VGK_MAX_FP = 32'sd131072;
localparam signed [FP_WIDTH-1:0] IG_VGK_STEP_FP = 32'sd2080;
localparam integer RG_INT = 1000000;

reg signed [15:0] ig_lut  [0:IG_LUT_SIZE-1];
reg signed [15:0] dig_lut [0:IG_LUT_SIZE-1];

initial begin
    $readmemh("data/ig_lut.hex", ig_lut);
    $readmemh("data/dig_lut.hex", dig_lut);
end

// ============================================================================
// Koren Direct Evaluator (single instance, shared across all preamp stages)
// ============================================================================

reg         koren_start;
reg  signed [FP_WIDTH-1:0] koren_vpk;
reg  signed [FP_WIDTH-1:0] koren_vgk;
wire signed [FP_WIDTH-1:0] koren_ip;
wire        koren_done;

koren_direct #(
    .FP_FRAC  (FP_FRAC),
    .FP_WIDTH (FP_WIDTH)
) koren_eval (
    .clk    (clk),
    .rst_n  (rst_n),
    .start  (koren_start),
    .vpk    (koren_vpk),
    .vgk    (koren_vgk),
    .ip_out (koren_ip),
    .done   (koren_done)
);

// Saved result from first koren evaluation (for numerical derivative)
reg signed [FP_WIDTH-1:0] koren_ip1_saved;

// ============================================================================
// Power Amp LUT Address Functions (kept for power amp stage)
// ============================================================================

function automatic [LUT_BITS-1:0] vpk_to_idx_pa;
    input signed [FP_WIDTH-1:0] vpk_fp;
    reg signed [63:0] tmp;
    begin
        tmp = (vpk_fp * 1000) >>> FP_FRAC;
        if (tmp < VPK_MIN_MV_PA) tmp = VPK_MIN_MV_PA;
        if (tmp > VPK_MAX_MV_PA) tmp = VPK_MAX_MV_PA;
        vpk_to_idx_pa = ((tmp - VPK_MIN_MV_PA) * (LUT_SIZE-1)) / (VPK_MAX_MV_PA - VPK_MIN_MV_PA);
    end
endfunction

function automatic [LUT_BITS-1:0] vgk_to_idx_pa;
    input signed [FP_WIDTH-1:0] vgk_fp;
    reg signed [63:0] tmp;
    begin
        tmp = (vgk_fp * 1000) >>> FP_FRAC;
        if (tmp < VGK_MIN_MV_PA) tmp = VGK_MIN_MV_PA;
        if (tmp > VGK_MAX_MV_PA) tmp = VGK_MAX_MV_PA;
        vgk_to_idx_pa = ((tmp - VGK_MIN_MV_PA) * (LUT_SIZE-1)) / (VGK_MAX_MV_PA - VGK_MIN_MV_PA);
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

localparam ST_IDLE      = 4'd0;
localparam ST_LOAD      = 4'd1;   // Load state bank for current stage
localparam ST_HP        = 4'd2;
localparam ST_NR_ADDR   = 4'd3;
localparam ST_NR_EVAL1  = 4'd4;   // Koren eval #1: Ip(Vpk, Vgk) -- preamp only
localparam ST_NR_EVAL2  = 4'd5;   // Koren eval #2: Ip(Vpk, Vgk+h) -- preamp only
localparam ST_NR_DERIV  = 4'd6;   // Compute numerical derivative -- preamp only
localparam ST_NR_READ   = 4'd7;   // BRAM read -- power amp only
localparam ST_NR_CONV   = 4'd8;   // Convert raw to Q16.16 -- power amp only
localparam ST_NR_STEP   = 4'd9;
localparam ST_OUTPUT    = 4'd10;
localparam ST_STORE     = 4'd11;  // Store state bank, advance stage

reg [3:0] state;

// ============================================================================
// Stage Counter
// ============================================================================

reg [$clog2(TOTAL_STAGES+1)-1:0] current_stage;
reg signed [FP_WIDTH-1:0] stage_input;  // Input to current stage

// ============================================================================
// Per-Stage State Banks (persistent across samples)
// ============================================================================

reg signed [FP_WIDTH-1:0] bank_hp_prev_in  [0:TOTAL_STAGES-1];
reg signed [FP_WIDTH-1:0] bank_hp_prev_out [0:TOTAL_STAGES-1];
reg signed [FP_WIDTH-1:0] bank_ck_z        [0:TOTAL_STAGES-1];
reg signed [FP_WIDTH-1:0] bank_ip_prev     [0:TOTAL_STAGES-1];
reg signed [FP_WIDTH-1:0] bank_ig_prev     [0:TOTAL_STAGES-1];
reg signed [FP_WIDTH-1:0] bank_vp_dc       [0:TOTAL_STAGES-1];
reg [15:0]                bank_sample_count [0:TOTAL_STAGES-1];
reg                       bank_dc_frozen    [0:TOTAL_STAGES-1];

// ============================================================================
// Working Registers (loaded from / stored to banks)
// ============================================================================

reg signed [FP_WIDTH-1:0] hp_prev_in;
reg signed [FP_WIDTH-1:0] hp_prev_out;
reg signed [FP_WIDTH-1:0] vgrid;
reg signed [FP_WIDTH-1:0] ck_z;
reg signed [FP_WIDTH-1:0] b_cathode;
reg signed [FP_WIDTH-1:0] ip_est;
reg signed [FP_WIDTH-1:0] ip_prev;
reg signed [FP_WIDTH-1:0] vpk_est;
reg signed [FP_WIDTH-1:0] vgk_est;
reg [1:0] newton_iter;
// Max Newton iterations: 2 for preamp (koren_direct is slower), 3 for power amp (2D LUT is fast)
localparam [1:0] PREAMP_MAX_ITER = 2'd1;  // 0,1 = 2 iterations
localparam [1:0] PA_MAX_ITER     = 2'd2;  // 0,1,2 = 3 iterations

reg [LUT_BITS*2-1:0] lut_addr;
reg signed [15:0] ip_raw;
reg signed [15:0] dip_vgk_raw;


reg signed [FP_WIDTH-1:0] ip_model;
reg signed [FP_WIDTH-1:0] dip_vgk_val;

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
reg signed [63:0] temp_b;

reg signed [FP_WIDTH-1:0] vp_dc;
reg [15:0] sample_count;
reg        dc_frozen;

reg signed [63:0] temp64;
reg signed [63:0] temp64b;
reg signed [63:0] hp_temp;
reg signed [63:0] hp_temp2;

reg signed [FP_WIDTH-1:0] f_val;
reg signed [FP_WIDTH-1:0] fp_val;
reg signed [63:0] temp_a;
reg signed [47:0] step_num;
reg signed [FP_WIDTH-1:0] step;
reg signed [FP_WIDTH-1:0] step_ig;

// Stage output (before attenuation)
reg signed [FP_WIDTH-1:0] stage_out;

// ============================================================================
// Bank Initialization
// ============================================================================

integer init_i;
initial begin
    for (init_i = 0; init_i < TOTAL_STAGES; init_i = init_i + 1) begin
        bank_hp_prev_in[init_i]   = 0;
        bank_hp_prev_out[init_i]  = 0;
        bank_ck_z[init_i]         = 32'sd53000;
        bank_ip_prev[init_i]      = 32'sd35;
        bank_ig_prev[init_i]      = 0;
        bank_vp_dc[init_i]        = VP_DC_INIT;
        bank_sample_count[init_i] = 0;
        bank_dc_frozen[init_i]    = 0;
    end
end

// ============================================================================
// Main State Machine
// ============================================================================

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state         <= ST_IDLE;
        out_valid     <= 1'b0;
        audio_out     <= 0;
        current_stage <= 0;
        stage_input   <= 0;
        hp_prev_in    <= 0;
        hp_prev_out   <= 0;
        vgrid         <= 0;
        ck_z          <= 32'sd53000;
        b_cathode     <= 32'sd53000;
        ip_est        <= 0;
        ip_prev       <= 32'sd35;
        ig_est        <= 0;
        ig_prev       <= 0;
        vpk_est       <= 0;
        vgk_est       <= 0;
        newton_iter   <= 0;
        ig_raw        <= 0;
        dig_raw       <= 0;
        f1_val        <= 0;
        f2_val        <= 0;
        lut_addr      <= 0;
        ip_raw        <= 0;
        dip_vgk_raw   <= 0;
        ip_model      <= 0;
        dip_vgk_val   <= 0;
        vp_dc         <= VP_DC_INIT;
        sample_count  <= 0;
        dc_frozen     <= 0;
        f_val         <= 0;
        fp_val        <= 0;
        step          <= 0;
        stage_out     <= 0;
        koren_start   <= 1'b0;
        koren_vpk     <= 0;
        koren_vgk     <= 0;
        koren_ip1_saved <= 0;
    end else begin
        out_valid   <= 1'b0;
        koren_start <= 1'b0;

        case (state)

        // ── Wait for sample_en ─────────────────────────────────────────
        ST_IDLE: begin
            if (sample_en) begin
                current_stage <= 0;
                stage_input   <= audio_in;
                state         <= ST_LOAD;
            end
        end

        // ── Load state bank for current stage ──────────────────────────
        ST_LOAD: begin
            hp_prev_in   <= bank_hp_prev_in[current_stage];
            hp_prev_out  <= bank_hp_prev_out[current_stage];
            ck_z         <= bank_ck_z[current_stage];
            ip_prev      <= bank_ip_prev[current_stage];
            vp_dc        <= bank_vp_dc[current_stage];
            sample_count <= bank_sample_count[current_stage];
            dc_frozen    <= bank_dc_frozen[current_stage];

            // Select constants for preamp vs power amp
            if (is_power_amp) begin
                vb_active  <= VB_FP_PA;
                rp_active  <= RP_INT_PA;
                rpk_active <= RPK_INT_PA;
            end else begin
                vb_active  <= VB_FP_PRE;
                rp_active  <= RP_INT_PRE;
                rpk_active <= RPK_INT_PRE;
            end

            ip_est      <= bank_ip_prev[current_stage];
            ig_prev     <= bank_ig_prev[current_stage];
            ig_est      <= bank_ig_prev[current_stage];
            newton_iter <= 0;
            state       <= ST_HP;
        end

        // ── High-pass coupling filter ──────────────────────────────────
        ST_HP: begin
            hp_temp  = $signed(C_HP) * $signed(hp_prev_out);
            hp_temp2 = $signed(HP_GAIN) * $signed(stage_input - hp_prev_in);

            vgrid <= hp_temp[47:16] + hp_temp2[47:16];
            hp_prev_out <= hp_temp[47:16] + hp_temp2[47:16];
            hp_prev_in  <= stage_input;

            temp64 = $signed(GAMMA_CATH) * $signed(ck_z);
            b_cathode <= ck_z - temp64[47:16];

            state <= ST_NR_ADDR;
        end

        // ── Newton-Raphson: compute Vpk/Vgk estimates ──────────────────
        ST_NR_ADDR: begin
            temp64 = rpk_active * $signed(ip_est);
            vpk_est <= (vb_active - b_cathode) - temp64[31:0];
            vgk_est <= vgrid - b_cathode;

            if (is_power_amp) begin
                // Power amp: use 2D LUT path
                lut_addr <= vpk_to_idx_pa((vb_active - b_cathode) - temp64[31:0]) * LUT_SIZE
                          + vgk_to_idx_pa(vgrid - b_cathode);
                state <= ST_NR_READ;
            end else begin
                // Preamp: start koren_direct eval #1 with Ip(Vpk, Vgk)
                koren_vpk   <= (vb_active - b_cathode) - temp64[31:0];
                koren_vgk   <= vgrid - b_cathode;
                koren_start <= 1'b1;
                state       <= ST_NR_EVAL1;
            end

            // Grid current LUT read (needed for both paths)
            ig_raw  <= ig_lut[vgk_to_ig_idx(vgrid - b_cathode)];
            dig_raw <= dig_lut[vgk_to_ig_idx(vgrid - b_cathode)];
        end

        // ── Preamp: wait for koren eval #1 (Ip at Vgk) ────────────────
        ST_NR_EVAL1: begin
            if (koren_done) begin
                // Save Ip(Vpk, Vgk)
                koren_ip1_saved <= koren_ip;
                ip_model        <= koren_ip;

                // Start eval #2: Ip(Vpk, Vgk + h) for numerical derivative
                koren_vpk   <= vpk_est;
                koren_vgk   <= vgk_est + DERIV_H_FP;
                koren_start <= 1'b1;
                state       <= ST_NR_EVAL2;
            end
        end

        // ── Preamp: wait for koren eval #2 (Ip at Vgk+h) ──────────────
        ST_NR_EVAL2: begin
            if (koren_done) begin
                // koren_ip now has Ip(Vpk, Vgk+h)
                // koren_ip1_saved has Ip(Vpk, Vgk)
                state <= ST_NR_DERIV;
            end
        end

        // ── Preamp: compute numerical derivative dIp/dVgk ──────────────
        ST_NR_DERIV: begin
            // dIp/dVgk = (ip2 - ip1) / h
            // With h = 0.01V (655 in Q16.16):
            //   result in Q16.16 = (ip2 - ip1) * INV_H
            // But INV_H = 100 and we need the result as a raw int16-equivalent
            // for the Jacobian computation. The DIP_SCALE factor is 100000.
            //
            // In the original code: dip_vgk_val = (dip_vgk_raw << 16) / DIP_SCALE
            // We want dip_vgk_val in Q16.16 directly:
            //   dip_vgk_val = (ip2 - ip1) * (2^16 / h)
            //               = (ip2 - ip1) * (65536 / 655)
            //               = (ip2 - ip1) * 100
            temp64 = ($signed(koren_ip) - $signed(koren_ip1_saved)) * $signed(INV_H);
            dip_vgk_val <= temp64[31:0];

            // Also compute dip_vgk_raw equivalent for Jacobian J12 computation
            // Original: j12 = (dip_vgk_raw * RG_INT) << 16 / DIP_SCALE
            // We have dip_vgk_val in Q16.16 already, so:
            //   j12 = dip_vgk_val * RG_INT (in Q16.16)
            // This is handled in ST_NR_STEP

            state <= ST_NR_STEP;
        end

        // ── Power amp: BRAM read latency ───────────────────────────────
        ST_NR_READ: begin
            ip_raw      <= ip_lut_pa[lut_addr];
            dip_vgk_raw <= dip_vgk_lut_pa[lut_addr];
            state <= ST_NR_CONV;
        end

        // ── Power amp: convert LUT raw to Q16.16 ──────────────────────
        ST_NR_CONV: begin
            ip_model    <= ($signed(ip_raw) <<< FP_FRAC) / IP_SCALE;
            dip_vgk_val <= ($signed(dip_vgk_raw) <<< FP_FRAC) / DIP_SCALE;
            state <= ST_NR_STEP;
        end

        // ── Newton step (shared for preamp and power amp) ──────────────
        ST_NR_STEP: begin
            f1_val = ip_est - ip_model;
            f2_val = ig_est - $signed({{16{ig_raw[15]}}, ig_raw});

            // J11 = 1 + dIp/dVpk * RPK
            // Constant approximation: J11 = 2
            j11 = ONE_FP <<< 1;

            // J12 = dIp/dVgk * Rg
            if (is_power_amp) begin
                // Power amp: dip_vgk_raw is raw int16 from LUT
                temp_b = ($signed(dip_vgk_raw) * $signed(RG_INT)) <<< FP_FRAC;
                j12 = temp_b / DIP_SCALE;
            end else begin
                // Preamp: dip_vgk_val is already Q16.16
                // J12 = dip_vgk_val * RG_INT
                temp_b = $signed(dip_vgk_val) * $signed(RG_INT);
                j12 = temp_b[47:16];
            end

            // J21 = 0
            j21 = 0;

            // J22 = 1 + dIg/dVgk * Rg
            temp_a = ($signed({{16{dig_raw[15]}}, dig_raw}) * $signed(RG_INT));
            j22 = ONE_FP + temp_a[47:16];

            det = (j11 * j22) >>> FP_FRAC;
            dIp_num = (j22 * $signed(f1_val) - j12 * $signed(f2_val)) >>> FP_FRAC;
            dIg_num = (j11 * $signed(f2_val)) >>> FP_FRAC;

            if (det != 0) begin
                step = (dIp_num <<< FP_FRAC) / det;
                ip_est <= ip_est - step;
                if ((ip_est - step) < 0) ip_est <= 0;

                step_ig = (dIg_num <<< FP_FRAC) / det;
                ig_est <= ig_est - step_ig;
                if ((ig_est - step_ig) < 0) ig_est <= 0;
            end

            // Preamp: 2 Newton iterations; Power amp: 3 iterations
            begin
                reg [1:0] max_iter;
                max_iter = is_power_amp ? PA_MAX_ITER : PREAMP_MAX_ITER;
                if (newton_iter < max_iter) begin
                    newton_iter <= newton_iter + 1;
                    state <= ST_NR_ADDR;
                end else begin
                    state <= ST_OUTPUT;
                end
            end
        end

        // ── Compute stage output with grid current ─────────────────────
        ST_OUTPUT: begin
            begin
                reg signed [FP_WIDTH-1:0] ip_total;
                ip_total = ip_est + ig_est;

                temp64 = rp_active * $signed(ip_total);
                stage_out <= (vb_active - temp64[31:0]) - vp_dc;
            end

            ip_prev <= ip_est;

            // Update cathode bypass cap using total current (Ip + Ig)
            temp64b = $signed(TWO_R_CATH_FP) * $signed(ip_est + ig_est);
            ck_z <= (b_cathode + temp64b[47:16]) + b_cathode - ck_z;

            // DC tracking
            if (!dc_frozen) begin
                vp_dc <= vp_dc + (((vb_active - temp64[31:0]) - vp_dc) >>> DC_SHIFT);
                if (sample_count < DC_SETTLE_SAMPLES)
                    sample_count <= sample_count + 1;
                else
                    dc_frozen <= 1'b1;
            end

            state <= ST_STORE;
        end

        // ── Store state bank, advance to next stage or finish ──────────
        ST_STORE: begin
            // Store working registers back to bank
            bank_hp_prev_in[current_stage]   <= hp_prev_in;
            bank_hp_prev_out[current_stage]  <= hp_prev_out;
            bank_ck_z[current_stage]         <= ck_z;
            bank_ip_prev[current_stage]      <= ip_prev;
            bank_ig_prev[current_stage]      <= ig_est;
            bank_vp_dc[current_stage]        <= vp_dc;
            bank_sample_count[current_stage] <= sample_count;
            bank_dc_frozen[current_stage]    <= dc_frozen;

            if (current_stage == TOTAL_STAGES - 1) begin
                // All stages done
                audio_out <= stage_out;
                out_valid <= 1'b1;
                state     <= ST_IDLE;
            end else begin
                // Feed to next stage
                stage_input   <= stage_out;
                current_stage <= current_stage + 1;
                state         <= ST_LOAD;
            end
        end

        default: state <= ST_IDLE;

        endcase
    end
end

endmodule
