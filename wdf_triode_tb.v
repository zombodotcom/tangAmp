// ============================================================================
// wdf_triode_tb.v
// Testbench for the WDF triode stage
//
// Feeds a 440Hz sine wave (concert A) into the triode model
// and dumps the output to a file for analysis in Python/Audacity
// ============================================================================

`timescale 1ns/1ps

module wdf_triode_tb;

// ── Clock and Reset ─────────────────────────────────────────────────────────
reg clk, rst_n;
localparam CLK_PERIOD = 37;  // ~27MHz (37ns period)

initial clk = 0;
always #(CLK_PERIOD/2) clk = ~clk;

// ── Sample Rate Enable ───────────────────────────────────────────────────────
// 27MHz / 48kHz = 562.5 → pulse every 562 clocks
localparam SAMPLE_DIV = 562;
reg [9:0] sample_cnt;
reg sample_en;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sample_cnt <= 0;
        sample_en  <= 0;
    end else begin
        sample_en <= 0;
        if (sample_cnt >= SAMPLE_DIV - 1) begin
            sample_cnt <= 0;
            sample_en  <= 1;
        end else begin
            sample_cnt <= sample_cnt + 1;
        end
    end
end

// ── DUT ─────────────────────────────────────────────────────────────────────
wire signed [31:0] audio_out;
wire               out_valid;
reg  signed [31:0] audio_in;

wdf_triode #(
    .VB     (200.0),
    .RP     (100000),
    .RG     (1000000),
    .RK     (1500),
    .CIN_UF (0.022),
    .CK_UF  (22.0),
    .FS     (48000.0),
    .FCLK   (27000000.0)
) dut (
    .clk       (clk),
    .rst_n     (rst_n),
    .sample_en (sample_en),
    .audio_in  (audio_in),
    .audio_out (audio_out),
    .out_valid (out_valid)
);

// ── Sine Wave Generator ──────────────────────────────────────────────────────
// 440Hz sine at 0.5V amplitude (typical guitar pickup level)
// Precomputed 48kHz sine table — 109 samples per period at 440Hz
// Q16.16 format: 0.5V = 0.5 * 65536 = 32768

integer sine_idx;
reg signed [31:0] sine_table [0:107];  // 48000/440 = 109.09 → 108 samples

integer i;
real two_pi = 6.2831853;

initial begin
    for (i = 0; i < 108; i = i + 1) begin
        // 0.5V amplitude in Q16.16
        sine_table[i] = $rtoi($sin(two_pi * i / 108.0) * 0.5 * 65536.0);
    end
    sine_idx = 0;
end

// DC settling period: run 500 samples with zero input before starting sine
localparam DC_SETTLE_SAMPLES = 500;

// Feed sine samples on each sample_en pulse (after DC settling)
always @(posedge clk) begin
    if (sample_en) begin
        if (sample_count < DC_SETTLE_SAMPLES) begin
            audio_in <= 0;  // silence during settling
        end else begin
            audio_in <= sine_table[sine_idx];
            if (sine_idx >= 107)
                sine_idx <= 0;
            else
                sine_idx <= sine_idx + 1;
        end
    end
end

// ── Output Capture ───────────────────────────────────────────────────────────
integer fd;
integer sample_count;

initial begin
    fd = $fopen("tb_output.txt", "w");
    sample_count = 0;
end

always @(posedge clk) begin
    if (out_valid) begin
        // Write: sample_number, input, output (both Q16.16 integers)
        $fdisplay(fd, "%0d %0d %0d", sample_count, audio_in, audio_out);
        sample_count = sample_count + 1;
    end
end

// ── Run Simulation ───────────────────────────────────────────────────────────
initial begin
    rst_n = 0;
    audio_in = 0;
    #(CLK_PERIOD * 10);
    rst_n = 1;

    // Run for 500 settling + 4800 audio = 5300 samples at 48kHz
    #(5300 * SAMPLE_DIV * CLK_PERIOD);

    $fclose(fd);
    $display("Simulation complete. %0d samples written to tb_output.txt", sample_count);
    $finish;
end

// ── Waveform Dump ────────────────────────────────────────────────────────────
initial begin
    $dumpfile("wdf_triode.vcd");
    $dumpvars(0, wdf_triode_tb);
end

endmodule
