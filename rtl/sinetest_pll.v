// Sine test using clk_audio_pll (instead of clk_audio_gen).
// If sine plays cleanly: clk_audio_pll is fine, problem is ADC-side.
// If sine is noisy: clk_audio_pll has a timing bug that breaks the DAC too.

module sinetest_pll (
    input  wire clk_27m,
    input  wire btn_s1,
    input  wire btn_s2,
    output wire mclk_out,
    output wire adc_bck,
    output wire adc_lrck,
    output wire dac_din,
    input  wire adc_dout,
    output wire sd_clk, sd_cmd, sd_dat1, sd_dat2, sd_dat3,
    input  wire sd_dat0,
    output wire [5:0] led
);

assign sd_clk=0; assign sd_cmd=1; assign sd_dat1=1; assign sd_dat2=1; assign sd_dat3=1;

reg [9:0] por_cnt; reg rst_n;
always @(posedge clk_27m) begin
    if (por_cnt < 10'd1023) begin por_cnt <= por_cnt + 1; rst_n <= 0; end
    else rst_n <= 1;
end

wire mclk, bck, lrck, sample_en, pll_lock;
clk_audio_pll u_pll (
    .clk_27m(clk_27m), .rst_n(rst_n),
    .mclk(mclk), .bck(bck), .lrck(lrck), .sample_en(sample_en), .pll_lock(pll_lock)
);
assign mclk_out = mclk;
assign adc_bck  = bck;
assign adc_lrck = lrck;

// 440 Hz sine table (108 entries for ~46kHz sample rate)
reg signed [31:0] sine_table [0:107];
initial begin
    sine_table[  0] =  32'sd0;       sine_table[  1] =  32'sd1905;
    sine_table[  2] =  32'sd3804;    sine_table[  3] =  32'sd5690;
    sine_table[  4] =  32'sd7557;    sine_table[  5] =  32'sd9398;
    sine_table[  6] =  32'sd11207;   sine_table[  7] =  32'sd12979;
    sine_table[  8] =  32'sd14706;   sine_table[  9] =  32'sd16384;
    sine_table[ 10] =  32'sd18006;   sine_table[ 11] =  32'sd19567;
    sine_table[ 12] =  32'sd21062;   sine_table[ 13] =  32'sd22486;
    sine_table[ 14] =  32'sd23834;   sine_table[ 15] =  32'sd25101;
    sine_table[ 16] =  32'sd26284;   sine_table[ 17] =  32'sd27378;
    sine_table[ 18] =  32'sd28378;   sine_table[ 19] =  32'sd29282;
    sine_table[ 20] =  32'sd30087;   sine_table[ 21] =  32'sd30791;
    sine_table[ 22] =  32'sd31390;   sine_table[ 23] =  32'sd31884;
    sine_table[ 24] =  32'sd32270;   sine_table[ 25] =  32'sd32546;
    sine_table[ 26] =  32'sd32713;   sine_table[ 27] =  32'sd32768;
    sine_table[ 28] =  32'sd32713;   sine_table[ 29] =  32'sd32546;
    sine_table[ 30] =  32'sd32270;   sine_table[ 31] =  32'sd31884;
    sine_table[ 32] =  32'sd31390;   sine_table[ 33] =  32'sd30791;
    sine_table[ 34] =  32'sd30087;   sine_table[ 35] =  32'sd29282;
    sine_table[ 36] =  32'sd28378;   sine_table[ 37] =  32'sd27378;
    sine_table[ 38] =  32'sd26284;   sine_table[ 39] =  32'sd25101;
    sine_table[ 40] =  32'sd23834;   sine_table[ 41] =  32'sd22486;
    sine_table[ 42] =  32'sd21062;   sine_table[ 43] =  32'sd19567;
    sine_table[ 44] =  32'sd18006;   sine_table[ 45] =  32'sd16384;
    sine_table[ 46] =  32'sd14706;   sine_table[ 47] =  32'sd12979;
    sine_table[ 48] =  32'sd11207;   sine_table[ 49] =  32'sd9398;
    sine_table[ 50] =  32'sd7557;    sine_table[ 51] =  32'sd5690;
    sine_table[ 52] =  32'sd3804;    sine_table[ 53] =  32'sd1905;
    sine_table[ 54] =  32'sd0;       sine_table[ 55] = -32'sd1905;
    sine_table[ 56] = -32'sd3804;    sine_table[ 57] = -32'sd5690;
    sine_table[ 58] = -32'sd7557;    sine_table[ 59] = -32'sd9398;
    sine_table[ 60] = -32'sd11207;   sine_table[ 61] = -32'sd12979;
    sine_table[ 62] = -32'sd14706;   sine_table[ 63] = -32'sd16384;
    sine_table[ 64] = -32'sd18006;   sine_table[ 65] = -32'sd19567;
    sine_table[ 66] = -32'sd21062;   sine_table[ 67] = -32'sd22486;
    sine_table[ 68] = -32'sd23834;   sine_table[ 69] = -32'sd25101;
    sine_table[ 70] = -32'sd26284;   sine_table[ 71] = -32'sd27378;
    sine_table[ 72] = -32'sd28378;   sine_table[ 73] = -32'sd29282;
    sine_table[ 74] = -32'sd30087;   sine_table[ 75] = -32'sd30791;
    sine_table[ 76] = -32'sd31390;   sine_table[ 77] = -32'sd31884;
    sine_table[ 78] = -32'sd32270;   sine_table[ 79] = -32'sd32546;
    sine_table[ 80] = -32'sd32713;   sine_table[ 81] = -32'sd32768;
    sine_table[ 82] = -32'sd32713;   sine_table[ 83] = -32'sd32546;
    sine_table[ 84] = -32'sd32270;   sine_table[ 85] = -32'sd31884;
    sine_table[ 86] = -32'sd31390;   sine_table[ 87] = -32'sd30791;
    sine_table[ 88] = -32'sd30087;   sine_table[ 89] = -32'sd29282;
    sine_table[ 90] = -32'sd28378;   sine_table[ 91] = -32'sd27378;
    sine_table[ 92] = -32'sd26284;   sine_table[ 93] = -32'sd25101;
    sine_table[ 94] = -32'sd23834;   sine_table[ 95] = -32'sd22486;
    sine_table[ 96] = -32'sd21062;   sine_table[ 97] = -32'sd19567;
    sine_table[ 98] = -32'sd18006;   sine_table[ 99] = -32'sd16384;
    sine_table[100] = -32'sd14706;   sine_table[101] = -32'sd12979;
    sine_table[102] = -32'sd11207;   sine_table[103] = -32'sd9398;
    sine_table[104] = -32'sd7557;    sine_table[105] = -32'sd5690;
    sine_table[106] = -32'sd3804;    sine_table[107] = -32'sd1905;
end

reg [6:0] sine_idx;
reg signed [31:0] audio_sample;
always @(posedge mclk or negedge rst_n) begin
    if (!rst_n) begin sine_idx <= 0; audio_sample <= 0; end
    else if (sample_en) begin
        audio_sample <= sine_table[sine_idx];
        sine_idx <= (sine_idx >= 7'd107) ? 0 : sine_idx + 1;
    end
end

i2s_tx u_tx (
    .clk(mclk), .rst_n(rst_n), .bck(bck), .lrck(lrck),
    .audio_l(audio_sample), .audio_r(audio_sample),
    .load(sample_en), .dout(dac_din)
);

reg [24:0] hb_cnt; reg hb;
always @(posedge clk_27m) begin
    if (hb_cnt >= 25'd13_500_000) begin hb_cnt <= 0; hb <= ~hb; end
    else hb_cnt <= hb_cnt + 1;
end

assign led[0] = ~hb;
assign led[1] = ~pll_lock;
assign led[2] = 1; assign led[3] = 1; assign led[4] = 1;
assign led[5] = 1'b0;

endmodule
