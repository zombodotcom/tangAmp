// ============================================================================
// interactive_sine.v
// Button-controlled sine wave generator for testing DAC + buttons together.
//
// S1 = cycle through 4 frequencies (440, 880, 220, 1000 Hz)
// S2 = cycle through 4 volumes (25%, 50%, 75%, 100%)
//
// LED[0] = heartbeat
// LED[1] = always on (sine mode indicator)
// LED[2:3] = frequency select (binary: 00=440, 01=880, 10=220, 11=1000)
// LED[4:5] = volume select  (binary: 00=25%, 01=50%, 10=75%, 11=100%)
// ============================================================================

module interactive_sine (
    input  wire clk_27m,
    input  wire btn_s1,
    input  wire btn_s2,

    output wire adc_bck,
    output wire adc_lrck,
    input  wire adc_dout,
    output wire dac_din,

    output wire sd_clk,
    output wire sd_cmd,
    input  wire sd_dat0,
    output wire sd_dat1,
    output wire sd_dat2,
    output wire sd_dat3,

    output wire [5:0] led
);

// SD card inactive
assign sd_clk  = 1'b0;
assign sd_cmd  = 1'b1;
assign sd_dat1 = 1'b1;
assign sd_dat2 = 1'b1;
assign sd_dat3 = 1'b1;

// ── Power-on reset ───────────────────────────────────────────────────────
reg [9:0] por_cnt;
reg rst_n;
always @(posedge clk_27m) begin
    if (por_cnt < 10'd1023) begin
        por_cnt <= por_cnt + 1;
        rst_n <= 1'b0;
    end else
        rst_n <= 1'b1;
end

// ── Audio Clock Generator ────────────────────────────────────────────────
wire bck, lrck, sample_en;

clk_audio_gen u_clkgen (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .bck       (bck),
    .lrck      (lrck),
    .sample_en (sample_en)
);

assign adc_bck  = bck;
assign adc_lrck = lrck;

// ── Button Debounce ──────────────────────────────────────────────────────
reg [15:0] s1_db, s2_db;
reg s1_prev, s2_prev;
reg [1:0] freq_sel, vol_sel;

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        s1_db <= 0; s2_db <= 0;
        s1_prev <= 1; s2_prev <= 1;
        freq_sel <= 0; vol_sel <= 0;
    end else begin
        s1_prev <= btn_s1;
        s2_prev <= btn_s2;
        if (s1_db > 0) s1_db <= s1_db - 1;
        if (s2_db > 0) s2_db <= s2_db - 1;

        if (!btn_s1 && s1_prev && s1_db == 0) begin
            freq_sel <= freq_sel + 1;
            s1_db <= 16'hFFFF;
        end
        if (!btn_s2 && s2_prev && s2_db == 0) begin
            vol_sel <= vol_sel + 1;
            s2_db <= 16'hFFFF;
        end
    end
end

// ── Phase accumulator DDS ────────────────────────────────────────────────
// Phase increment = freq * 2^32 / sample_rate
// At ~46875 Hz sample rate:
//   440Hz:  inc = 440 * 2^32 / 46875 = 40,307,814
//   880Hz:  inc = 80,615,628
//   220Hz:  inc = 20,153,907
//  1000Hz:  inc = 91,608,669

reg [31:0] phase_inc;
always @(*) begin
    case (freq_sel)
        2'd0: phase_inc = 32'd40_307_814;   // 440 Hz
        2'd1: phase_inc = 32'd80_615_628;   // 880 Hz
        2'd2: phase_inc = 32'd20_153_907;   // 220 Hz
        2'd3: phase_inc = 32'd91_608_669;   // 1000 Hz
    endcase
end

reg [31:0] phase_acc;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n)
        phase_acc <= 0;
    else if (sample_en)
        phase_acc <= phase_acc + phase_inc;
end

// ── Sine lookup (64-entry quarter-wave, mirrored) ────────────────────────
// Full amplitude = 32768 (0.5V in Q16.16)
reg signed [15:0] quarter_sine [0:63];
initial begin
    quarter_sine[ 0] = 16'sd0;
    quarter_sine[ 1] = 16'sd804;
    quarter_sine[ 2] = 16'sd1608;
    quarter_sine[ 3] = 16'sd2410;
    quarter_sine[ 4] = 16'sd3212;
    quarter_sine[ 5] = 16'sd4011;
    quarter_sine[ 6] = 16'sd4808;
    quarter_sine[ 7] = 16'sd5602;
    quarter_sine[ 8] = 16'sd6393;
    quarter_sine[ 9] = 16'sd7179;
    quarter_sine[10] = 16'sd7962;
    quarter_sine[11] = 16'sd8739;
    quarter_sine[12] = 16'sd9512;
    quarter_sine[13] = 16'sd10278;
    quarter_sine[14] = 16'sd11039;
    quarter_sine[15] = 16'sd11793;
    quarter_sine[16] = 16'sd12539;
    quarter_sine[17] = 16'sd13279;
    quarter_sine[18] = 16'sd14010;
    quarter_sine[19] = 16'sd14732;
    quarter_sine[20] = 16'sd15446;
    quarter_sine[21] = 16'sd16151;
    quarter_sine[22] = 16'sd16846;
    quarter_sine[23] = 16'sd17530;
    quarter_sine[24] = 16'sd18204;
    quarter_sine[25] = 16'sd18868;
    quarter_sine[26] = 16'sd19519;
    quarter_sine[27] = 16'sd20159;
    quarter_sine[28] = 16'sd20787;
    quarter_sine[29] = 16'sd21403;
    quarter_sine[30] = 16'sd22005;
    quarter_sine[31] = 16'sd22594;
    quarter_sine[32] = 16'sd23170;
    quarter_sine[33] = 16'sd23731;
    quarter_sine[34] = 16'sd24279;
    quarter_sine[35] = 16'sd24811;
    quarter_sine[36] = 16'sd25329;
    quarter_sine[37] = 16'sd25832;
    quarter_sine[38] = 16'sd26319;
    quarter_sine[39] = 16'sd26790;
    quarter_sine[40] = 16'sd27245;
    quarter_sine[41] = 16'sd27683;
    quarter_sine[42] = 16'sd28105;
    quarter_sine[43] = 16'sd28510;
    quarter_sine[44] = 16'sd28898;
    quarter_sine[45] = 16'sd29268;
    quarter_sine[46] = 16'sd29621;
    quarter_sine[47] = 16'sd29956;
    quarter_sine[48] = 16'sd30273;
    quarter_sine[49] = 16'sd30571;
    quarter_sine[50] = 16'sd30852;
    quarter_sine[51] = 16'sd31113;
    quarter_sine[52] = 16'sd31356;
    quarter_sine[53] = 16'sd31580;
    quarter_sine[54] = 16'sd31785;
    quarter_sine[55] = 16'sd31971;
    quarter_sine[56] = 16'sd32137;
    quarter_sine[57] = 16'sd32285;
    quarter_sine[58] = 16'sd32412;
    quarter_sine[59] = 16'sd32521;
    quarter_sine[60] = 16'sd32609;
    quarter_sine[61] = 16'sd32678;
    quarter_sine[62] = 16'sd32728;
    quarter_sine[63] = 16'sd32757;
end

// Quarter-wave mirror: use top 2 bits of phase for quadrant
wire [5:0] table_idx = phase_acc[31] ^ phase_acc[30]
                       ? (6'd63 - phase_acc[29:24])
                       : phase_acc[29:24];
wire signed [15:0] table_val = quarter_sine[table_idx];
wire signed [31:0] raw_sine = phase_acc[31]
                              ? -{16'd0, table_val}
                              : {16'd0, table_val};

// ── Volume scaling ───────────────────────────────────────────────────────
reg [7:0] vol_mult;
always @(*) begin
    case (vol_sel)
        2'd0: vol_mult = 8'd64;    // 25%
        2'd1: vol_mult = 8'd128;   // 50%
        2'd2: vol_mult = 8'd192;   // 75%
        2'd3: vol_mult = 8'd255;   // 100%
    endcase
end

wire signed [39:0] scaled = raw_sine * $signed({1'b0, vol_mult});
wire signed [31:0] audio_sample = scaled[39:8];

// ── I2S Transmitter ──────────────────────────────────────────────────────
i2s_tx u_tx (
    .clk     (clk_27m),
    .rst_n   (rst_n),
    .bck     (bck),
    .lrck    (lrck),
    .audio_l (audio_sample),
    .audio_r (audio_sample),
    .load    (sample_en),
    .dout    (dac_din)
);

// ── Heartbeat ────────────────────────────────────────────────────────────
reg [24:0] hb_cnt;
reg hb;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin hb_cnt <= 0; hb <= 0; end
    else begin
        if (hb_cnt >= 25'd13_500_000) begin hb_cnt <= 0; hb <= ~hb; end
        else hb_cnt <= hb_cnt + 1;
    end
end

// Active-low LEDs
assign led[0] = ~hb;                    // heartbeat
assign led[1] = 1'b0;                   // always on (sine mode)
assign led[2] = ~freq_sel[0];           // freq bit 0
assign led[3] = ~freq_sel[1];           // freq bit 1
assign led[4] = ~vol_sel[0];            // vol bit 0
assign led[5] = ~vol_sel[1];            // vol bit 1

endmodule
