// Force pin 75 to a static level so we can verify it works at all.
// If multimeter reads 3.3V on PCM1808 SCK pin while this is running,
// the pin/wire is good. Then we know the issue is bitstream-side.

module pin75_test (
    input  wire clk_27m,
    input  wire btn_s1,
    input  wire btn_s2,
    output wire mclk_out,    // we'll drive this high
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
assign adc_bck=0;
assign adc_lrck=0;

// Force pin 75 high — multimeter should read 3.3V
assign mclk_out = 1'b1;

// Heartbeat to prove FPGA is running
reg [24:0] hb_cnt; reg hb;
always @(posedge clk_27m) begin
    if (hb_cnt >= 25'd13_500_000) begin hb_cnt <= 0; hb <= ~hb; end
    else hb_cnt <= hb_cnt + 1;
end

assign led[0] = ~hb;
assign led[1] = 1'b0;
assign led[2] = 1'b1;
assign led[3] = 1'b1;
assign led[4] = 1'b1;
assign led[5] = 1'b1;

endmodule
