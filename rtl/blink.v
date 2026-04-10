module blink (
    input  wire clk_27m,
    output wire [5:0] led
);

// Route clock through global buffer (pin 52 is SSPI, not a dedicated clock pin)
wire clk;
BUFG clk_buf (.I(clk_27m), .O(clk));

reg [25:0] cnt;

always @(posedge clk)
    cnt <= cnt + 1;

assign led[0] = ~cnt[25];
assign led[1] = ~cnt[24];
assign led[2] = ~cnt[23];
assign led[3] = ~cnt[22];
assign led[4] = ~cnt[21];
assign led[5] = ~cnt[20];

endmodule
