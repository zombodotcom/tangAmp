// Toggle pin 75 at 1 Hz so multimeter will alternate between 0V and 3.3V.
// If multimeter swings between values → pin 75 driving works in this bitstream.
// If multimeter is stuck at one value → bitstream not actually loaded.

module pin75_blink (
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
assign adc_bck=0;
assign adc_lrck=0;

reg [24:0] cnt;
reg pin_state;
always @(posedge clk_27m) begin
    if (cnt >= 25'd13_500_000) begin cnt <= 0; pin_state <= ~pin_state; end
    else cnt <= cnt + 1;
end

assign mclk_out = pin_state;

// LEDs: 0 mirrors pin_state so we can SEE pin 75 state visually
assign led[0] = ~pin_state;
assign led[1] = ~pin_state;
assign led[2] = 1'b1;
assign led[3] = 1'b1;
assign led[4] = 1'b1;
assign led[5] = 1'b1;

endmodule
