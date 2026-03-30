// ============================================================================
// tangamp_fullchain_tb.v
// Testbench for the full audio chain:
//   sine -> triode_engine(2) -> tone_stack_iir -> cabinet_fir
//
// Runs 12000 samples (11000 settle + 1000 active) at 48kHz.
// Dumps output to fullchain_output.txt for analysis.
// ============================================================================

`timescale 1ns / 1ps

module tangamp_fullchain_tb;

// ── Clock generation (27MHz = 37.037ns period) ──────────────────────────
reg clk;
initial clk = 0;
always #18.518 clk = ~clk;  // ~27MHz

// ── Reset ───────────────────────────────────────────────────────────────
reg rst_n;
initial begin
    rst_n = 0;
    #200;
    rst_n = 1;
end

// ── Sample rate divider (27MHz / 562 = ~48kHz) ─────────────────────────
reg [9:0] div_cnt;
reg sample_en;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        div_cnt <= 0;
        sample_en <= 0;
    end else begin
        sample_en <= 0;
        if (div_cnt >= 10'd561) begin
            div_cnt <= 0;
            sample_en <= 1;
        end else begin
            div_cnt <= div_cnt + 1;
        end
    end
end

// ── Sine wave generator (440Hz, 0.5V amplitude in Q16.16) ──────────────
reg signed [31:0] sine_table [0:107];
integer i;
initial begin
    for (i = 0; i < 108; i = i + 1)
        sine_table[i] = $rtoi($sin(6.2831853 * i / 108.0) * 0.5 * 65536.0);
end

reg [6:0] sine_idx;
reg signed [31:0] audio_in;
reg [15:0] sample_cnt;
localparam SETTLE = 16'd11000;
localparam TOTAL  = 16'd12000;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sine_idx   <= 0;
        audio_in   <= 0;
        sample_cnt <= 0;
    end else if (sample_en) begin
        if (sample_cnt < TOTAL)
            sample_cnt <= sample_cnt + 1;
        if (sample_cnt < SETTLE) begin
            audio_in <= 0;
        end else begin
            audio_in <= sine_table[sine_idx];
            sine_idx <= (sine_idx >= 7'd107) ? 0 : sine_idx + 1;
        end
    end
end

// ── Stage 1: Triode Engine ──────────────────────────────────────────────
wire signed [31:0] triode_out;
wire triode_valid;

triode_engine #(
    .NUM_STAGES  (2)
) u_triode (
    .clk       (clk),
    .rst_n     (rst_n),
    .sample_en (sample_en),
    .audio_in  (audio_in),
    .audio_out (triode_out),
    .out_valid (triode_valid)
);

// ── Stage 2: Tone Stack ────────────────────────────────────────────────
wire signed [31:0] tone_out;
wire tone_valid;

tone_stack_iir u_tone (
    .clk       (clk),
    .rst_n     (rst_n),
    .preset    (3'd0),
    .sample_en (triode_valid),
    .audio_in  (triode_out),
    .audio_out (tone_out),
    .out_valid (tone_valid)
);

// ── Stage 3: Cabinet FIR ───────────────────────────────────────────────
wire signed [31:0] cab_out;
wire cab_valid;

cabinet_fir #(
    .N_TAPS (129)
) u_cab (
    .clk       (clk),
    .rst_n     (rst_n),
    .sample_en (tone_valid),
    .audio_in  (tone_out),
    .audio_out (cab_out),
    .out_valid (cab_valid)
);

// ── Output logging ─────────────────────────────────────────────────────
integer fout;
integer sample_num;

initial begin
    fout = $fopen("fullchain_output.txt", "w");
    sample_num = 0;
end

always @(posedge clk) begin
    if (cab_valid) begin
        $fwrite(fout, "%d %d %d %d\n", audio_in, triode_out, tone_out, cab_out);
        sample_num = sample_num + 1;
        if (sample_num % 1000 == 0)
            $display("Sample %0d: in=%0d triode=%0d tone=%0d cab=%0d",
                     sample_num, audio_in, triode_out, tone_out, cab_out);
    end
end

// ── Simulation control ─────────────────────────────────────────────────
// Run for enough time to process TOTAL samples
// Each sample = 562 clocks × ~37ns = ~20.8us
// TOTAL samples × 20.8us = ~250ms, but triode engine takes longer (2 stages)
// Budget conservatively: 12000 × 562 × 2 × 37ns ≈ 500ms = 500_000_000 ns
// But that's too long. Let's use $finish after enough samples.

initial begin
    $dumpfile("fullchain.vcd");
    $dumpvars(0, tangamp_fullchain_tb);

    // Wait for all samples to process
    // 12000 samples × 562 clocks × 37ns ≈ 250ms sim time
    // Add extra margin for multi-stage pipeline
    #300_000_000;  // 300ms should be enough

    $display("\n=== Simulation Complete ===");
    $display("Processed %0d output samples", sample_num);
    $fclose(fout);
    $finish;
end

// Watchdog: finish early if all samples output
always @(posedge clk) begin
    if (sample_num >= TOTAL && cab_valid) begin
        #1000;
        $display("\n=== All %0d samples processed ===", sample_num);
        $fclose(fout);
        $finish;
    end
end

endmodule
