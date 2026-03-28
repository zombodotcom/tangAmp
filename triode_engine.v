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
    parameter NUM_STAGES  = 2,
    parameter ATTEN_SHIFT = 5,   // right-shift between stages (~-30dB)
    parameter FP_FRAC     = 16,
    parameter FP_WIDTH    = 32,
    parameter LUT_BITS    = 8,
    parameter LUT_SIZE    = 256,
    parameter IP_SCALE    = 10000,
    parameter DIP_SCALE   = 100000,
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
    output reg         out_valid
);

// ============================================================================
// Constants (same as wdf_triode_wdf.v)
// ============================================================================

localparam signed [FP_WIDTH-1:0] VB_FP = 200 * (1 << FP_FRAC);
localparam integer RP_INT  = 100000;
localparam integer RK_INT  = 1500;
localparam integer RPK_INT = 100000;

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

reg signed [15:0] ip_lut      [0 : LUT_SIZE*LUT_SIZE - 1];
reg signed [15:0] dip_vgk_lut [0 : LUT_SIZE*LUT_SIZE - 1];
reg signed [15:0] dip_vpk_lut [0 : LUT_SIZE*LUT_SIZE - 1];

initial begin
    $readmemh("ip_lut.hex",       ip_lut);
    $readmemh("dip_dvgk_lut.hex", dip_vgk_lut);
    $readmemh("dip_dvpk_lut.hex", dip_vpk_lut);
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

reg [$clog2(NUM_STAGES)-1:0] current_stage;
reg signed [FP_WIDTH-1:0] stage_input;  // Input to current stage

// ============================================================================
// Per-Stage State Banks (persistent across samples)
// ============================================================================

reg signed [FP_WIDTH-1:0] bank_hp_prev_in  [0:NUM_STAGES-1];
reg signed [FP_WIDTH-1:0] bank_hp_prev_out [0:NUM_STAGES-1];
reg signed [FP_WIDTH-1:0] bank_ck_z        [0:NUM_STAGES-1];
reg signed [FP_WIDTH-1:0] bank_ip_prev     [0:NUM_STAGES-1];
reg signed [FP_WIDTH-1:0] bank_vp_dc       [0:NUM_STAGES-1];
reg [15:0]                bank_sample_count [0:NUM_STAGES-1];
reg                       bank_dc_frozen    [0:NUM_STAGES-1];

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
reg signed [15:0] dip_vpk_raw;

reg signed [FP_WIDTH-1:0] ip_model;
reg signed [FP_WIDTH-1:0] dip_vgk_val;
reg signed [FP_WIDTH-1:0] dip_vpk_val;

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

// Stage output (before attenuation)
reg signed [FP_WIDTH-1:0] stage_out;

// ============================================================================
// Bank Initialization
// ============================================================================

integer init_i;
initial begin
    for (init_i = 0; init_i < NUM_STAGES; init_i = init_i + 1) begin
        bank_hp_prev_in[init_i]   = 0;
        bank_hp_prev_out[init_i]  = 0;
        bank_ck_z[init_i]         = 32'sd53000;
        bank_ip_prev[init_i]      = 32'sd35;
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
        vpk_est       <= 0;
        vgk_est       <= 0;
        newton_iter   <= 0;
        lut_addr      <= 0;
        ip_raw        <= 0;
        dip_vgk_raw   <= 0;
        dip_vpk_raw   <= 0;
        ip_model      <= 0;
        dip_vgk_val   <= 0;
        dip_vpk_val   <= 0;
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

            ip_est      <= bank_ip_prev[current_stage];
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
            temp64 = $signed(RPK_INT) * $signed(ip_est);
            vpk_est <= (VB_FP - b_cathode) - temp64[31:0];
            vgk_est <= vgrid - b_cathode;

            lut_addr <= vpk_to_idx((VB_FP - b_cathode) - temp64[31:0]) * LUT_SIZE
                      + vgk_to_idx(vgrid - b_cathode);

            state <= ST_NR_READ;
        end

        // ── BRAM read latency ──────────────────────────────────────────
        ST_NR_READ: begin
            ip_raw      <= ip_lut[lut_addr];
            dip_vgk_raw <= dip_vgk_lut[lut_addr];
            dip_vpk_raw <= dip_vpk_lut[lut_addr];
            state <= ST_NR_CONV;
        end

        // ── Convert LUT raw to Q16.16 ─────────────────────────────────
        ST_NR_CONV: begin
            ip_model    <= ($signed(ip_raw) <<< FP_FRAC) / IP_SCALE;
            dip_vgk_val <= ($signed(dip_vgk_raw) <<< FP_FRAC) / DIP_SCALE;
            dip_vpk_val <= ($signed(dip_vpk_raw) <<< FP_FRAC) / DIP_VPK_SCALE;
            state <= ST_NR_STEP;
        end

        // ── Newton step ────────────────────────────────────────────────
        ST_NR_STEP: begin
            f_val = ip_est - ip_model;
            temp_a = ($signed(dip_vpk_raw) * $signed(RPK_INT)) <<< FP_FRAC;
            fp_val = ONE_FP + temp_a / DIP_VPK_SCALE;

            step_num = {f_val, 16'b0};
            if (fp_val != 0)
                step = step_num / $signed(fp_val);
            else
                step = 0;

            ip_est <= ip_est - step;
            if ((ip_est - step) < 0)
                ip_est <= 0;

            if (newton_iter == 0) begin
                newton_iter <= 1;
                state <= ST_NR_ADDR;
            end else begin
                state <= ST_OUTPUT;
            end
        end

        // ── Compute stage output ───────────────────────────────────────
        ST_OUTPUT: begin
            temp64 = $signed(RP_INT) * $signed(ip_est);
            stage_out <= (VB_FP - temp64[31:0]) - vp_dc;

            ip_prev <= ip_est;

            // Update cathode bypass cap
            temp64b = $signed(TWO_R_CATH_FP) * $signed(ip_est);
            ck_z <= (b_cathode + temp64b[47:16]) + b_cathode - ck_z;

            // DC tracking
            if (!dc_frozen) begin
                vp_dc <= vp_dc + (((VB_FP - temp64[31:0]) - vp_dc) >>> DC_SHIFT);
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
            bank_vp_dc[current_stage]        <= vp_dc;
            bank_sample_count[current_stage] <= sample_count;
            bank_dc_frozen[current_stage]    <= dc_frozen;

            if (current_stage == NUM_STAGES - 1) begin
                // All stages done
                audio_out <= stage_out;
                out_valid <= 1'b1;
                state     <= ST_IDLE;
            end else begin
                // Attenuate output and feed to next stage
                stage_input   <= stage_out >>> ATTEN_SHIFT;
                current_stage <= current_stage + 1;
                state         <= ST_LOAD;
            end
        end

        default: state <= ST_IDLE;

        endcase
    end
end

endmodule
