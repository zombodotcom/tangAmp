// ============================================================================
// power_amp_wdf.v
// WDF-based Power Amp Stage (6L6 triode-connected)
//
// Same WDF algorithm as wdf_triode_wdf.v (preamp) but with:
//   - 6L6 tube LUTs (128x128, separate hex files)
//   - Different circuit parameters: VB=400V, RP=2000, RK=250
//   - Wider voltage ranges: VPK 0-500V, VGK -50V to 0V
//   - Soft-clip output stage for transformer saturation
//
// Fixed point: Q16.16 signed throughout
// Sample rate: 48kHz
// ============================================================================

module power_amp_wdf #(
    parameter FP_FRAC  = 16,
    parameter FP_WIDTH = 32,
    parameter LUT_BITS = 7,
    parameter LUT_SIZE = 128,
    parameter IP_SCALE = 10000,
    parameter DIP_SCALE = 100000,
    parameter DIP_VPK_SCALE = 10000000,
    parameter integer VPK_MIN_MV = 0,
    parameter integer VPK_MAX_MV = 500000,    // 0-500V for power tube
    parameter integer VGK_MIN_MV = -50000,    // -50V to 0V for 6L6
    parameter integer VGK_MAX_MV = 0,
    // Soft-clip threshold: 30V in Q16.16
    parameter signed [FP_WIDTH-1:0] CLIP_THRESHOLD = 32'sd1966080,
    // Hard ceiling: 40V in Q16.16
    parameter signed [FP_WIDTH-1:0] CLIP_CEILING   = 32'sd2621440
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sample_en,
    input  wire signed [FP_WIDTH-1:0] audio_in,
    output reg  signed [FP_WIDTH-1:0] audio_out,
    output reg                        out_valid
);

// ============================================================================
// Constants — 6L6 power amp circuit
// ============================================================================

// B+ supply voltage: 400V in Q16.16
localparam signed [FP_WIDTH-1:0] VB_FP = 400 * (1 << FP_FRAC);  // 26,214,400

// Resistor values (plain integers, NOT Q16.16)
localparam integer RP_INT  = 2000;     // Plate load (output transformer primary)
localparam integer RK_INT  = 250;      // Cathode resistor
localparam integer RPK_INT = 2000;     // RP + R_cathode (~2000 + 0.47 ~ 2000)

// High-pass filter coefficients — same as preamp (Cin=22nF, Rg=1M)
localparam signed [FP_WIDTH-1:0] C_HP    = 32'sd65474;
localparam signed [FP_WIDTH-1:0] HP_GAIN = 32'sd65505;

// 1.0 in Q16.16
localparam signed [FP_WIDTH-1:0] ONE_FP = 32'sd65536;

// Cathode parallel adaptor: Rk(250) || Ck(22uF)
// R_Ck = 1/(2*48000*22e-6) = 0.4735 ohm
// R_cathode = 1/(1/250 + 1/0.4735) = 0.47241 ohm
// gamma_cath = (1/250) / (1/250 + 1/0.4735) = 0.001891
// In Q16.16: round(0.001891 * 65536) = 124
localparam signed [FP_WIDTH-1:0] GAMMA_CATH = 32'sd123;
// 2*R_cathode = 0.945 ohm. In Q16.16: round(0.945 * 65536) = 61932
localparam signed [FP_WIDTH-1:0] TWO_R_CATH_FP = 32'sd61943;

// DC operating point: 6L6 at VB=400V, Ip~70mA -> Vp = 400 - 2000*0.07 = 260V
localparam signed [FP_WIDTH-1:0] VP_DC_INIT = 260 * (1 << FP_FRAC);

// DC tracking: EMA with alpha = 1/256 (shift by 8)
localparam DC_SHIFT = 8;
localparam DC_SETTLE_SAMPLES = 10000;

// Negative thresholds for soft clip
wire signed [FP_WIDTH-1:0] neg_threshold = -CLIP_THRESHOLD;
wire signed [FP_WIDTH-1:0] neg_ceiling   = -CLIP_CEILING;

// ============================================================================
// LUT Memory — 6L6 tube data
// ============================================================================

reg signed [15:0] ip_lut      [0 : LUT_SIZE*LUT_SIZE - 1];
reg signed [15:0] dip_vgk_lut [0 : LUT_SIZE*LUT_SIZE - 1];
reg signed [15:0] dip_vpk_lut [0 : LUT_SIZE*LUT_SIZE - 1];

initial begin
    $readmemh("ip_lut_6l6.hex",       ip_lut);
    $readmemh("dip_dvgk_lut_6l6.hex", dip_vgk_lut);
    $readmemh("dip_dvpk_lut_6l6.hex", dip_vpk_lut);
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
localparam ST_HP      = 4'd1;
localparam ST_NR_ADDR = 4'd2;
localparam ST_NR_READ = 4'd3;
localparam ST_NR_CONV = 4'd4;
localparam ST_NR_STEP = 4'd5;
localparam ST_OUTPUT  = 4'd6;
localparam ST_CLIP    = 4'd7;

reg [3:0] state;

// ============================================================================
// State Registers
// ============================================================================

reg signed [FP_WIDTH-1:0] hp_prev_in;
reg signed [FP_WIDTH-1:0] hp_prev_out;
reg signed [FP_WIDTH-1:0] vgrid;
reg signed [FP_WIDTH-1:0] ck_z;
reg signed [FP_WIDTH-1:0] b_cathode;

// Newton-Raphson
reg signed [FP_WIDTH-1:0] ip_est;
reg signed [FP_WIDTH-1:0] ip_prev;
reg signed [FP_WIDTH-1:0] vpk_est;
reg signed [FP_WIDTH-1:0] vgk_est;
reg [0:0] newton_iter;

// LUT
reg [LUT_BITS*2-1:0] lut_addr;
reg signed [15:0] ip_raw;
reg signed [15:0] dip_vgk_raw;
reg signed [15:0] dip_vpk_raw;
reg signed [FP_WIDTH-1:0] ip_model;
reg signed [FP_WIDTH-1:0] dip_vgk_val;
reg signed [FP_WIDTH-1:0] dip_vpk_val;

// DC tracking
reg signed [FP_WIDTH-1:0] vp_dc;
reg [15:0] sample_count;
reg        dc_frozen;

// Intermediates
reg signed [63:0] temp64;
reg signed [63:0] temp64b;
reg signed [63:0] hp_temp;
reg signed [63:0] hp_temp2;
reg signed [FP_WIDTH-1:0] f_val;
reg signed [FP_WIDTH-1:0] fp_val;
reg signed [63:0] temp_a;
reg signed [47:0] step_num;
reg signed [FP_WIDTH-1:0] step;

// Pre-clip output
reg signed [FP_WIDTH-1:0] wdf_out;

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
        // 6L6 quiescent: Ip~70mA, Vk~17.5V, ck_z init ~2300000
        ck_z        <= 32'sd2300000;
        b_cathode   <= 32'sd2300000;
        ip_est      <= 0;
        ip_prev     <= 32'sd459;       // ~7mA in Q16.16 (0.007 * 65536)
        vpk_est     <= 0;
        vgk_est     <= 0;
        newton_iter <= 0;
        lut_addr    <= 0;
        ip_raw      <= 0;
        dip_vgk_raw <= 0;
        dip_vpk_raw <= 0;
        ip_model    <= 0;
        dip_vgk_val <= 0;
        dip_vpk_val <= 0;
        vp_dc       <= VP_DC_INIT;
        sample_count <= 0;
        dc_frozen   <= 0;
        f_val       <= 0;
        fp_val      <= 0;
        step        <= 0;
        wdf_out     <= 0;
    end else begin
        out_valid <= 1'b0;

        case (state)

        // ── Wait for sample_en ──────────────────────────────────────────
        ST_IDLE: begin
            if (sample_en) begin
                ip_est <= ip_prev;
                newton_iter <= 0;
                state <= ST_HP;
            end
        end

        // ── High-pass coupling filter ───────────────────────────────────
        ST_HP: begin
            hp_temp  = $signed(C_HP) * $signed(hp_prev_out);
            hp_temp2 = $signed(HP_GAIN) * $signed(audio_in - hp_prev_in);
            vgrid <= hp_temp[47:16] + hp_temp2[47:16];
            hp_prev_out <= hp_temp[47:16] + hp_temp2[47:16];
            hp_prev_in  <= audio_in;

            // Cathode parallel adaptor upward pass
            temp64 = $signed(GAMMA_CATH) * $signed(ck_z);
            b_cathode <= ck_z - temp64[47:16];

            state <= ST_NR_ADDR;
        end

        // ── Newton-Raphson: compute LUT address ─────────────────────────
        ST_NR_ADDR: begin
            temp64 = $signed(RPK_INT) * $signed(ip_est);
            vpk_est <= (VB_FP - b_cathode) - temp64[31:0];
            vgk_est <= vgrid - b_cathode;

            lut_addr <= vpk_to_idx((VB_FP - b_cathode) - temp64[31:0]) * LUT_SIZE
                      + vgk_to_idx(vgrid - b_cathode);

            state <= ST_NR_READ;
        end

        // ── BRAM read ───────────────────────────────────────────────────
        ST_NR_READ: begin
            ip_raw      <= ip_lut[lut_addr];
            dip_vgk_raw <= dip_vgk_lut[lut_addr];
            dip_vpk_raw <= dip_vpk_lut[lut_addr];
            state <= ST_NR_CONV;
        end

        // ── Convert LUT raw to Q16.16 ───────────────────────────────────
        ST_NR_CONV: begin
            ip_model    <= ($signed(ip_raw) <<< FP_FRAC) / IP_SCALE;
            dip_vgk_val <= ($signed(dip_vgk_raw) <<< FP_FRAC) / DIP_SCALE;
            dip_vpk_val <= ($signed(dip_vpk_raw) <<< FP_FRAC) / DIP_VPK_SCALE;
            state <= ST_NR_STEP;
        end

        // ── Newton step ─────────────────────────────────────────────────
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

        // ── Compute WDF output ──────────────────────────────────────────
        ST_OUTPUT: begin
            temp64 = $signed(RP_INT) * $signed(ip_est);
            wdf_out <= (VB_FP - temp64[31:0]) - vp_dc;

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

            state <= ST_CLIP;
        end

        // ── Soft clip (transformer saturation) ──────────────────────────
        ST_CLIP: begin
            if (wdf_out > CLIP_THRESHOLD) begin
                if (wdf_out > CLIP_CEILING)
                    audio_out <= CLIP_CEILING;
                else
                    audio_out <= CLIP_THRESHOLD + ((wdf_out - CLIP_THRESHOLD) >>> 2);
            end else if (wdf_out < neg_threshold) begin
                if (wdf_out < neg_ceiling)
                    audio_out <= neg_ceiling;
                else
                    audio_out <= neg_threshold + ((wdf_out - neg_threshold) >>> 2);
            end else begin
                audio_out <= wdf_out;
            end
            out_valid <= 1'b1;
            state <= ST_IDLE;
        end

        default: state <= ST_IDLE;

        endcase
    end
end

endmodule
