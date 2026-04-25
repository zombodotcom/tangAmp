// ============================================================================
// adc_simple.v — minimal ADC test, no PLL, properly registered clock outputs.
//
// Drives MCLK via a divider register so the signal escapes the clock network
// onto an I/O pin. Half clk_27m for clean toggling.
//
// Clocks:
//   SCKI = 27 MHz / 2 = 13.5 MHz
//   BCK  = 27 MHz / 12 = 2.25 MHz
//   LRCK = BCK / 64 = 35.16 kHz
//   SCKI / LRCK = 384  ✓ matches MD0=H MD1=L
//
// LED[0] = heartbeat
// LED[1] = always on
// LED[2] = BCK divider running (slow blink)
// LED[3] = LRCK divider running (faster)
// LED[4] = DOUT toggled at least once (sticky)
// LED[5] = live DOUT mirror
// ============================================================================

module adc_simple (
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
assign dac_din=0;

reg [9:0] por_cnt; reg rst_n;
always @(posedge clk_27m) begin
    if (por_cnt < 10'd1023) begin por_cnt <= por_cnt + 1; rst_n <= 0; end
    else rst_n <= 1;
end

// MCLK = clk_27m / 2 = 13.5 MHz, registered so it reaches the I/O pin
reg mclk_reg;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) mclk_reg <= 0;
    else        mclk_reg <= ~mclk_reg;
end
assign mclk_out = mclk_reg;

// BCK = clk_27m / 12 = 2.25 MHz
reg [2:0] bck_cnt;
reg bck_reg;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin bck_cnt <= 0; bck_reg <= 1; end
    else begin
        if (bck_cnt == 3'd5) begin bck_cnt <= 0; bck_reg <= ~bck_reg; end
        else bck_cnt <= bck_cnt + 1;
    end
end
assign adc_bck = bck_reg;

// BCK rising edge in clk_27m domain
reg bck_d;
wire bck_rise = bck_reg & ~bck_d;
always @(posedge clk_27m) bck_d <= bck_reg;

// LRCK = BCK / 64 (35.16 kHz)
reg [5:0] lrck_cnt;
reg lrck_reg;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin lrck_cnt <= 0; lrck_reg <= 1; end
    else if (bck_rise) begin
        if (lrck_cnt == 6'd31) begin lrck_cnt <= 0; lrck_reg <= ~lrck_reg; end
        else lrck_cnt <= lrck_cnt + 1;
    end
end
assign adc_lrck = lrck_reg;

// DOUT sticky-toggle latch
reg dout_d;
reg dout_ever;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin dout_d <= 0; dout_ever <= 0; end
    else begin
        dout_d <= adc_dout;
        if (adc_dout != dout_d) dout_ever <= 1;
    end
end

// Heartbeat
reg [24:0] hb_cnt; reg hb;
always @(posedge clk_27m) begin
    if (hb_cnt >= 25'd13_500_000) begin hb_cnt <= 0; hb <= ~hb; end
    else hb_cnt <= hb_cnt + 1;
end

// BCK divided way down so we can see it
reg [21:0] bck_div;
always @(posedge bck_reg) bck_div <= bck_div + 1;

// LRCK divided
reg [14:0] lrck_div;
always @(posedge lrck_reg) lrck_div <= lrck_div + 1;

assign led[0] = ~hb;
assign led[1] = 1'b0;
assign led[2] = ~bck_div[21];
assign led[3] = ~lrck_div[14];
assign led[4] = ~dout_ever;
assign led[5] = ~adc_dout;

endmodule
