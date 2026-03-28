// ============================================================================
// triode_engine.v
// Time-multiplexed WDF triode processing engine.
//
// Uses a SINGLE set of 3 LUT BRAMs (shared) and processes NUM_STAGES
// triode stages sequentially per audio sample. Each stage has its own
// persistent state bank stored in registers.
//
// Clock budget: ~14 clocks/stage × NUM_STAGES per sample.
// At NUM_STAGES=3: 42 clocks out of 562 available.
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
    // 12AX7 preamp ranges
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
localparam integer RPK_INT_PRE = 100000;  // RP + R_cathode (R_cath ≈ 0)

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

// ============================================================================
// LUT Memory (shared across all stages)
// ============================================================================

// 12AX7 preamp LUTs (ip + dip/dvgk only — dip/dvpk dropped to fit BSRAM)
reg signed [15:0] ip_lut      [0 : LUT_SIZE*LUT_SIZE - 1];
reg signed [15:0] dip_vgk_lut [0 : LUT_SIZE*LUT_SIZE - 1];

// 6L6 power amp LUTs
reg signed [15:0] ip_lut_pa      [0 : LUT_SIZE*LUT_SIZE - 1];
reg signed [15:0] dip_vgk_lut_pa [0 : LUT_SIZE*LUT_SIZE - 1];

initial begin
    $readmemh("data/ip_lut.hex",           ip_lut);
    $readmemh("data/dip_dvgk_lut.hex",     dip_vgk_lut);
    $readmemh("data/ip_lut_6l6.hex",       ip_lut_pa);
    $readmemh("data/dip_dvgk_lut_6l6.hex", dip_vgk_lut_pa);
end

// Grid current LUTs (shared across all stages)
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
// LUT Address Functions
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

// Power amp LUT address functions (wider voltage ranges)
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

localparam ST_IDLE    = 4'd0;
localparam ST_LOAD    = 4'd1;   // Load state bank for current stage
localparam ST_HP      = 4'd2;
localparam ST_NR_ADDR = 4'd3;
localparam ST_NR_READ = 4'd4;
localparam ST_NR_CONV = 4'd5;
localparam ST_NR_STEP = 4'd6;
localparam ST_OUTPUT  = 4'd7;
localparam ST_STORE   = 4'd8;   // Store state bank, advance stage
localparam ST_DONE    = 4'd9;

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
reg [0:0] newton_iter;

reg [LUT_BITS*2-1:0] lut_addr;
reg signed [15:0] ip_raw;
reg signed [15:0] dip_vgk_raw;
// dip_vpk_raw removed — dIp/dVpk LUT dropped to fit BSRAM

reg signed [FP_WIDTH-1:0] ip_model;
reg signed [FP_WIDTH-1:0] dip_vgk_val;
// dip_vpk_val removed

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
        // dip_vpk_raw removed
        ip_model      <= 0;
        dip_vgk_val   <= 0;
        // dip_vpk_val removed
        vp_dc         <= VP_DC_INIT;
        sample_count  <= 0;
        dc_frozen     <= 0;
        f_val         <= 0;
        fp_val        <= 0;
        step          <= 0;
        stage_out     <= 0;
    end else begin
        out_valid <= 1'b0;

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

        // ── Newton-Raphson: compute LUT address ────────────────────────
        ST_NR_ADDR: begin
            temp64 = rpk_active * $signed(ip_est);
            vpk_est <= (vb_active - b_cathode) - temp64[31:0];
            vgk_est <= vgrid - b_cathode;

            // Use different LUT address functions for preamp vs power amp
            if (is_power_amp)
                lut_addr <= vpk_to_idx_pa((vb_active - b_cathode) - temp64[31:0]) * LUT_SIZE
                          + vgk_to_idx_pa(vgrid - b_cathode);
            else
                lut_addr <= vpk_to_idx((vb_active - b_cathode) - temp64[31:0]) * LUT_SIZE
                          + vgk_to_idx(vgrid - b_cathode);

            state <= ST_NR_READ;
        end

        // ── BRAM read latency ──────────────────────────────────────────
        ST_NR_READ: begin
            if (is_power_amp) begin
                ip_raw      <= ip_lut_pa[lut_addr];
                dip_vgk_raw <= dip_vgk_lut_pa[lut_addr];
            end else begin
                ip_raw      <= ip_lut[lut_addr];
                dip_vgk_raw <= dip_vgk_lut[lut_addr];
            end
            // Grid current LUT read
            ig_raw  <= ig_lut[vgk_to_ig_idx(vgk_est)];
            dig_raw <= dig_lut[vgk_to_ig_idx(vgk_est)];
            state <= ST_NR_CONV;
        end

        // ── Convert LUT raw to Q16.16 ─────────────────────────────────
        ST_NR_CONV: begin
            ip_model    <= ($signed(ip_raw) <<< FP_FRAC) / IP_SCALE;
            dip_vgk_val <= ($signed(dip_vgk_raw) <<< FP_FRAC) / DIP_SCALE;
            state <= ST_NR_STEP;
        end

        // ── Newton step ────────────────────────────────────────────────
        ST_NR_STEP: begin
            f1_val = ip_est - ip_model;
            f2_val = ig_est - $signed({{16{ig_raw[15]}}, ig_raw});

            // J11 ≈ 2 (constant approximation; dIp/dVpk LUT dropped to fit BSRAM)
            // True J11 = 1 + dIp/dVpk * RPK ≈ 1.5-2.5 for 12AX7 at typical operating points.
            // J11=1 diverges because step=f1 overshoots; J11=2 halves the step, converging.
            j11 = ONE_FP <<< 1;

            // J12 = dIp/dVgk * Rg
            temp_b = ($signed(dip_vgk_raw) * $signed(RG_INT)) <<< FP_FRAC;
            j12 = temp_b / DIP_SCALE;

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

            if (newton_iter == 0) begin
                newton_iter <= 1;
                state <= ST_NR_ADDR;
            end else begin
                state <= ST_OUTPUT;
            end
        end

        // ── Compute stage output with grid current ──────────────────────
        ST_OUTPUT: begin
            // Use converged ig_est from 2x2 Newton solver
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
                // Attenuate output and feed to next stage
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
