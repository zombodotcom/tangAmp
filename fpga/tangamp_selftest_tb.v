// ============================================================================
// tangamp_selftest_tb.v
// Testbench for the FPGA self-test design.
// Verifies: sine generation, triode processing, VU meter output.
// ============================================================================

`timescale 1ns/1ps

module tangamp_selftest_tb;

reg clk_27m;
reg btn_s1;
wire [5:0] led;

// 27MHz clock
initial clk_27m = 0;
always #(18.5) clk_27m = ~clk_27m;  // ~37ns period = 27MHz

tangamp_selftest dut (
    .clk_27m (clk_27m),
    .btn_s1  (btn_s1),
    .led     (led)
);

// Monitor internal signals
integer fd;
integer sample_count;
integer led_checks;
integer led_on_count;

initial begin
    fd = $fopen("selftest_output.txt", "w");
    sample_count = 0;
    led_checks = 0;
    led_on_count = 0;

    // Reset
    btn_s1 = 0;  // active low reset
    #(37 * 20);
    btn_s1 = 1;  // release reset

    // Run for 15000 samples (includes 10000 settling + 5000 audio)
    // At 562 clocks per sample, 15000 * 562 * 37ns = ~312ms
    #(15000 * 562 * 37);

    $fclose(fd);
    $display("Self-test simulation complete. %0d samples captured.", sample_count);
    $display("LED checks: %0d, LED[1] on count: %0d", led_checks, led_on_count);

    // Verify: after settling, at least some LEDs should be on
    if (led_on_count > 100) begin
        $display("PASS: Triode is producing output (LEDs active)");
    end else begin
        $display("FAIL: No LED activity — triode may not be processing");
    end

    $finish;
end

// Capture triode output and LED state at each sample
always @(posedge clk_27m) begin
    if (dut.sample_en) begin
        $fdisplay(fd, "%0d %0d %0d %b",
            sample_count,
            dut.audio_in,
            dut.audio_out,
            led);
        sample_count = sample_count + 1;
    end
end

// Check LEDs periodically (after settling)
always @(posedge clk_27m) begin
    if (dut.out_valid && sample_count > 10500) begin
        led_checks = led_checks + 1;
        if (!led[1]) // active low: LED on when pin is 0
            led_on_count = led_on_count + 1;
    end
end

// Waveform dump
initial begin
    $dumpfile("selftest.vcd");
    $dumpvars(0, tangamp_selftest_tb);
end

endmodule
