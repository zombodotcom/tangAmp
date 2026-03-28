// ============================================================================
// DEPRECATED - kept for reference only.
// The working WDF triode is in wdf_triode_wdf.v (proper WDF tree with
// Newton-Raphson iteration at the root).
// ============================================================================
// wdf_triode.v
// Single Common-Cathode Triode Stage — Direct Load-Line Model
//
// Models a 12AX7 triode using:
//   - Koren equation via 2D LUT (Ip as function of Vpk, Vgk)
//   - Load line:  V_plate = VB - Rp * Ip
//   - Coupling cap: first-order IIR high-pass (bilinear transform)
//
// This is a simplified model that captures the essential tube nonlinearity
// and load-line interaction. Future version will use full WDF tree for
// more accurate reactive component modelling.
//
// Fixed point: Q16.16 signed throughout
// Sample rate: 48kHz
// Clock:       27MHz (Tang Nano 20K)
// ============================================================================

module wdf_triode #(
    // ── Circuit Parameters ──────────────────────────────────────────────────
    parameter real VB       = 200.0,   // B+ supply voltage (V)
    parameter real RP       = 100000,  // Plate resistor (ohms) — 100k
    parameter real RG       = 1000000, // Grid resistor (ohms)  — 1M
    parameter real RK       = 1500,    // Cathode resistor      — 1.5k
    parameter real CIN_UF   = 0.022,   // Input cap (uF)        — 22nF
    parameter real CK_UF    = 22.0,    // Cathode bypass cap    — 22uF

    // ── Sample Rate / Clock ─────────────────────────────────────────────────
    parameter real FS       = 48000.0, // Sample rate (Hz)
    parameter real FCLK     = 27000000.0, // FPGA clock (Hz)

    // ── Fixed Point Format ──────────────────────────────────────────────────
    parameter FP_FRAC       = 16,
    parameter FP_WIDTH      = 32,

    // ── LUT Parameters (must match tube_lut_gen.py output) ─────────────────
    parameter LUT_BITS      = 8,
    parameter LUT_SIZE      = 256,
    parameter IP_SCALE      = 10000,
    parameter DIP_SCALE     = 100000,

    // Physical ranges for LUT address calculation (millivolts)
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
// Constants in Q16.16
// ============================================================================

// B+ supply voltage
localparam signed [FP_WIDTH-1:0] VB_FP      = 200 * (1 << FP_FRAC);  // 13,107,200
localparam signed [FP_WIDTH-1:0] VB_HALF_FP = 100 * (1 << FP_FRAC);  // 6,553,600

// Plate resistor as plain integer (NOT Q16.16 — too large)
// Used as: Rp * Ip_q16 = plain_int * Q16.16 = Q16.16
localparam integer RP_INT = 100000;

// Coupling capacitor high-pass filter coefficient
// f_c = 1/(2*pi*Rg*Cin) = 7.2 Hz
// alpha = Rg*Cin*2*Fs / (1 + Rg*Cin*2*Fs)
// Rg*Cin = 1e6 * 22e-9 = 0.022s, tau*2*Fs = 0.022*96000 = 2112
// alpha = 2112 / 2113 = 0.999527
// In Q16.16: round(0.999527 * 65536) = 65505
localparam signed [FP_WIDTH-1:0] HP_ALPHA = 32'sd65505;

// Cathode bias voltage estimate: Vk = Rk * Ip_quiescent
// At quiescent Ip ≈ 1mA: Vk = 1500 * 0.001 = 1.5V
// In Q16.16: 1.5 * 65536 = 98304
// We'll compute this dynamically instead

// ============================================================================
// LUT Memory
// ============================================================================

reg signed [15:0] ip_lut [0 : LUT_SIZE*LUT_SIZE - 1];
reg signed [15:0] dip_lut [0 : LUT_SIZE*LUT_SIZE - 1];

initial begin
    $readmemh("ip_lut.hex",      ip_lut);
    $readmemh("dip_dvgk_lut.hex", dip_lut);
end

// ============================================================================
// State Registers
// ============================================================================

// Previous sample estimates
reg signed [FP_WIDTH-1:0] vpk_prev;    // Previous plate-cathode voltage
reg signed [FP_WIDTH-1:0] vgk_prev;    // Previous grid-cathode voltage

// Coupling cap high-pass filter state
reg signed [FP_WIDTH-1:0] hp_prev_in;  // Previous input sample
reg signed [FP_WIDTH-1:0] hp_prev_out; // Previous output sample

// Pipeline registers
reg signed [FP_WIDTH-1:0] vgk_current; // Grid voltage this sample
reg signed [FP_WIDTH-1:0] ip_val;      // Plate current from LUT
reg signed [FP_WIDTH-1:0] vplate;      // Plate voltage from load line
reg signed [FP_WIDTH-1:0] vk;          // Cathode voltage

// LUT address
reg [LUT_BITS*2-1:0] lut_addr;
reg signed [15:0] ip_raw;

// 64-bit intermediates
reg signed [63:0] temp64;
reg signed [63:0] hp_temp;

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
// Pipeline State Machine
//
// ST_IDLE:     Wait for sample pulse
// ST_HP:       Coupling cap high-pass filter → Vgk
// ST_LUT_ADDR: Compute LUT address from Vpk/Vgk estimates
// ST_LUT_READ: BRAM read latency
// ST_CONVERT:  Convert LUT raw → Q16.16 Ip
// ST_LOADLINE: V_plate = VB - Rp*Ip, compute cathode voltage
// ST_OUTPUT:   AC-couple output, update estimates
// ============================================================================

localparam ST_IDLE      = 3'd0;
localparam ST_HP        = 3'd1;
localparam ST_LUT_ADDR  = 3'd2;
localparam ST_LUT_READ  = 3'd3;
localparam ST_CONVERT   = 3'd4;
localparam ST_LOADLINE  = 3'd5;
localparam ST_OUTPUT    = 3'd6;

reg [2:0] state;

// ============================================================================
// Main State Machine
// ============================================================================

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state       <= ST_IDLE;
        out_valid   <= 1'b0;
        audio_out   <= 0;
        vpk_prev    <= 32'sd6422528; // ~98V (realistic operating point)
        vgk_prev    <= -32'sd98304; // ~-1.5V (cathode self-bias)
        hp_prev_in  <= 0;
        hp_prev_out <= 0;
        vgk_current <= 0;
        ip_val      <= 32'sd65;     // ~1mA quiescent current
        vplate      <= 32'sd6553600; // ~100V plate
        vk          <= 0;
        lut_addr    <= 0;
        ip_raw      <= 0;
    end else begin
        out_valid <= 1'b0;

        case (state)

        // ── Wait for sample ───────────────────────────────────────────────
        ST_IDLE: begin
            if (sample_en) begin
                state <= ST_HP;
            end
        end

        // ── Coupling cap high-pass filter ─────────────────────────────────
        // y[n] = alpha * (y[n-1] + x[n] - x[n-1])
        // This models Cin blocking DC while passing audio
        ST_HP: begin
            // High-pass: output = alpha * (prev_out + current_in - prev_in)
            // alpha is Q16.16, (prev_out + in - prev_in) is Q16.16
            // Product is Q32.32, shift right by 16
            hp_temp = $signed(HP_ALPHA) * $signed(hp_prev_out + audio_in - hp_prev_in);
            vgk_current <= hp_temp[47:16];  // Q16.16 result

            // Update filter state
            hp_prev_in  <= audio_in;
            hp_prev_out <= hp_temp[47:16];

            state <= ST_LUT_ADDR;
        end

        // ── Compute LUT address ───────────────────────────────────────────
        // Use previous Vpk and current Vgk (after coupling cap)
        // Vgk should be negative for normal operation (grid below cathode)
        // Subtract cathode voltage: Vgk_actual = V_grid - V_cathode
        ST_LUT_ADDR: begin
            // Cathode voltage: Vk = Rk * Ip_prev
            // Rk = 1500 (plain int), ip_val is Q16.16
            // For first sample ip_val=0 so Vk=0, which is fine
            temp64 = 64'sd1500 * $signed(ip_val);
            vk <= temp64[31:0];  // Q16.16 volts

            // Vgk = V_grid - V_cathode
            // V_grid comes from coupling cap output
            // With cathode at Vk, the grid is biased negative
            lut_addr <= vpk_to_idx(vpk_prev) * LUT_SIZE
                      + vgk_to_idx(vgk_current - temp64[31:0]);

            state <= ST_LUT_READ;
        end

        // ── BRAM read latency ─────────────────────────────────────────────
        ST_LUT_READ: begin
            ip_raw <= ip_lut[lut_addr];
            state <= ST_CONVERT;
        end

        // ── Convert LUT → Q16.16 amps ────────────────────────────────────
        // ip_raw = Ip * IP_SCALE as int16
        // ip_val = (ip_raw << 16) / IP_SCALE
        ST_CONVERT: begin
            ip_val <= ($signed(ip_raw) <<< FP_FRAC) / IP_SCALE;
            state <= ST_LOADLINE;
        end

        // ── Load line: V_plate = VB - Rp * Ip ────────────────────────────
        // Then: Vpk = V_plate - Vk (plate-to-cathode voltage)
        ST_LOADLINE: begin
            // Rp * Ip: plain int * Q16.16 = Q16.16
            // Use 64-bit for safety
            temp64 = 64'sd100000 * $signed(ip_val);
            vplate <= VB_FP - temp64[31:0];

            state <= ST_OUTPUT;
        end

        // ── Output and update estimates ───────────────────────────────────
        ST_OUTPUT: begin
            // AC-coupled output: remove DC bias (approximately VB/2)
            audio_out <= vplate - VB_HALF_FP;
            out_valid <= 1'b1;

            // Update estimates for next sample's LUT lookup
            // Use relaxation (average of old and new) to prevent oscillation
            // Vpk_new = V_plate - V_cathode
            // vpk_prev = (vpk_prev + Vpk_new) / 2
            vpk_prev <= (vpk_prev + (vplate - vk)) >>> 1;

            state <= ST_IDLE;
        end

        default: state <= ST_IDLE;

        endcase
    end
end

endmodule
