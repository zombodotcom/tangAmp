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
//   3. Newton-Raphson (2 iterations) using ip/dip LUTs
//   4. Extract plate voltage via load line, AC-couple output
//
// Fixed point: Q16.16 signed throughout
// Sample rate: 48kHz
// Clock:       27MHz (Tang Nano 20K)
// ============================================================================

module wdf_triode_wdf #(
    parameter FP_FRAC  = 16,
    parameter FP_WIDTH = 32,
    parameter LUT_BITS = 8,
    parameter LUT_SIZE = 256,
    parameter IP_SCALE = 10000,
    parameter DIP_SCALE = 100000,
    parameter DIP_VPK_SCALE = 100000,
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
localparam integer RPK_INT = 101500;   // RP + RK

// High-pass filter coefficients (Q0.16, stored in lower 16 bits)
// c_hp  = 0.999054 -> round(0.999054 * 65536) = 65474
// hp_gain = 0.999527 -> round(0.999527 * 65536) = 65505
localparam signed [FP_WIDTH-1:0] C_HP    = 32'sd65474;
localparam signed [FP_WIDTH-1:0] HP_GAIN = 32'sd65505;

// 1.0 in Q16.16
localparam signed [FP_WIDTH-1:0] ONE_FP = 32'sd65536;

// DC operating point estimate: ~146V in Q16.16
// Will be refined by exponential moving average
localparam signed [FP_WIDTH-1:0] VP_DC_INIT = 146 * (1 << FP_FRAC);  // 9,568,256

// DC tracking: EMA with alpha = 1/256 (shift by 8)
localparam DC_SHIFT = 8;
localparam DC_SETTLE_SAMPLES = 2000;

// ============================================================================
// LUT Memory
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

// Newton-Raphson state
reg signed [FP_WIDTH-1:0] ip_est;       // Current Ip estimate (Q16.16)
reg signed [FP_WIDTH-1:0] ip_prev;      // Previous sample's converged Ip
reg signed [FP_WIDTH-1:0] vpk_est;      // Vpk estimate (Q16.16)
reg signed [FP_WIDTH-1:0] vgk_est;      // Vgk estimate (Q16.16)
reg [0:0] newton_iter;                   // Iteration counter (0 or 1)

// LUT address and raw values
reg [LUT_BITS*2-1:0] lut_addr;
reg signed [15:0] ip_raw;
reg signed [15:0] dip_vgk_raw;
reg signed [15:0] dip_vpk_raw;

// Converted Q16.16 LUT values
reg signed [FP_WIDTH-1:0] ip_model;
reg signed [FP_WIDTH-1:0] dip_vgk_val;
reg signed [FP_WIDTH-1:0] dip_vpk_val;

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
        ip_est      <= 0;
        ip_prev     <= 32'sd35;        // ~0.54mA quiescent in Q16.16
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
    end else begin
        out_valid <= 1'b0;

        case (state)

        // ── Wait for sample_en ──────────────────────────────────────────
        ST_IDLE: begin
            if (sample_en) begin
                // Initialize Ip estimate from previous sample's converged value
                ip_est <= ip_prev;
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

            state <= ST_NR_ADDR;
        end

        // ── Newton-Raphson: compute Vpk, Vgk, build LUT address ────────
        ST_NR_ADDR: begin
            // Vpk = VB - (RP + RK) * Ip = VB - 101500 * ip_est
            // RPK_INT is plain int, ip_est is Q16.16 -> result is Q16.16
            temp64 = $signed(RPK_INT) * $signed(ip_est);
            vpk_est <= VB_FP - temp64[31:0];

            // Vgk = vgrid - RK * Ip = vgrid - 1500 * ip_est
            temp64b = $signed(RK_INT) * $signed(ip_est);
            vgk_est <= vgrid - temp64b[31:0];

            // Build LUT address from Vpk and Vgk estimates
            lut_addr <= vpk_to_idx(VB_FP - temp64[31:0]) * LUT_SIZE
                      + vgk_to_idx(vgrid - temp64b[31:0]);

            state <= ST_NR_READ;
        end

        // ── BRAM read latency (1 clock) ─────────────────────────────────
        ST_NR_READ: begin
            ip_raw      <= ip_lut[lut_addr];
            dip_vgk_raw <= dip_vgk_lut[lut_addr];
            dip_vpk_raw <= dip_vpk_lut[lut_addr];
            state <= ST_NR_CONV;
        end

        // ── Convert LUT raw int16 to Q16.16 ────────────────────────────
        // ip_raw = Ip * IP_SCALE -> ip_model = (ip_raw << 16) / IP_SCALE
        // dip_vgk_raw = dIp/dVgk * DIP_SCALE -> dip_vgk_val = (raw << 16) / DIP_SCALE
        // dip_vpk_raw = dIp/dVpk * DIP_VPK_SCALE -> dip_vpk_val = (raw << 16) / DIP_VPK_SCALE
        ST_NR_CONV: begin
            ip_model    <= ($signed(ip_raw) <<< FP_FRAC) / IP_SCALE;
            dip_vgk_val <= ($signed(dip_vgk_raw) <<< FP_FRAC) / DIP_SCALE;
            dip_vpk_val <= ($signed(dip_vpk_raw) <<< FP_FRAC) / DIP_VPK_SCALE;
            state <= ST_NR_STEP;
        end

        // ── Newton step ─────────────────────────────────────────────────
        // f  = Ip - ip_model
        // f' = 1 + dip_vpk * (RP + RK) + dip_vgk * RK
        //    = 1 + dip_vpk * 101500 + dip_vgk * 1500
        // Ip_new = Ip - f / f'
        ST_NR_STEP: begin
            // f = ip_est - ip_model (both Q16.16)
            f_val = ip_est - ip_model;

            // f' = 1.0 + dip_vpk * 101500 + dip_vgk * 1500
            // dip_vpk_val is Q16.16, * plain int -> Q16.16
            temp_a = $signed(dip_vpk_val) * $signed(RPK_INT);
            temp_b = $signed(dip_vgk_val) * $signed(RK_INT);
            fp_val = ONE_FP + temp_a[31:0] + temp_b[31:0];

            // Newton step: Ip = Ip - f / f'
            // f is Q16.16, f' is Q16.16
            // To preserve precision: shift f left by 16 then divide by f'
            // (f << 16) / f' gives Q16.16 result
            step_num = {f_val, 16'b0};  // 48-bit: f << 16
            if (fp_val != 0)
                step = step_num / $signed(fp_val);
            else
                step = 0;

            ip_est <= ip_est - step;

            // Clamp to non-negative
            if ((ip_est - step) < 0)
                ip_est <= 0;

            // Check if more iterations needed
            if (newton_iter == 0) begin
                newton_iter <= 1;
                state <= ST_NR_ADDR;  // Loop back for second iteration
            end else begin
                state <= ST_OUTPUT;   // Done with Newton iterations
            end
        end

        // ── Compute output ──────────────────────────────────────────────
        // v_plate = VB - RP * Ip
        // AC output = v_plate - vp_dc
        ST_OUTPUT: begin
            // v_plate = VB - RP * ip_est
            temp64 = $signed(RP_INT) * $signed(ip_est);
            audio_out <= (VB_FP - temp64[31:0]) - vp_dc;
            out_valid <= 1'b1;

            // Save converged Ip for next sample's initial guess
            ip_prev <= ip_est;

            // Update DC tracking (exponential moving average)
            // vp_dc += (vplate - vp_dc) >> DC_SHIFT
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
