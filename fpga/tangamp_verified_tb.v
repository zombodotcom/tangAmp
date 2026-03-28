// ============================================================================
// tangamp_verified_tb.v
// Definitive end-to-end testbench for tangamp_selftest.
//
// Tests full signal chain: 440Hz sine -> triode_engine (2+PA) ->
//   tone_stack_iir -> cabinet_fir -> VU meter LEDs
//
// Runs 15000 samples (11000 settle + 4000 audio), writes output file,
// checks heartbeat and VU LEDs, reports PASS/FAIL.
// ============================================================================

`timescale 1ns/1ps

module tangamp_verified_tb;

// ── Clock & reset ───────────────────────────────────────────────────────
reg clk_27m;
reg btn_s1;
wire [5:0] led;

// 27MHz clock: period = 37.037ns, half-period = 18.519ns
initial clk_27m = 0;
always #(18.519) clk_27m = ~clk_27m;

// ── DUT ─────────────────────────────────────────────────────────────────
tangamp_selftest dut (
    .clk_27m (clk_27m),
    .btn_s1  (btn_s1),
    .led     (led)
);

// ── File output and counters ────────────────────────────────────────────
integer fd;
integer sample_count;
integer vu_on_count;      // samples where at least one VU LED is on
integer vu_check_count;   // total samples checked for VU (after settle)
reg     hb_initial;       // heartbeat state after reset
reg     hb_toggled;       // heartbeat has changed at least once
reg     hb_counting;      // heartbeat counter is advancing

// Track peak values for gain analysis
integer peak_audio_in;
integer peak_triode_out;
integer peak_cab_out;
integer abs_val;

// ── Main test sequence ──────────────────────────────────────────────────
initial begin
    $dumpfile("verified.vcd");
    $dumpvars(0, tangamp_verified_tb);

    fd = $fopen("verified_output.txt", "w");
    if (fd == 0) begin
        $display("ERROR: Could not open verified_output.txt for writing");
        $finish;
    end

    // Header
    $fdisplay(fd, "# sample_num  audio_in  triode_out  cab_out");

    sample_count    = 0;
    vu_on_count     = 0;
    vu_check_count  = 0;
    hb_toggled      = 0;
    hb_counting     = 0;
    peak_audio_in   = 0;
    peak_triode_out = 0;
    peak_cab_out    = 0;

    // ── Assert reset (btn_s1 active low) ────────────────────────────────
    btn_s1 = 0;
    #(37 * 30);  // hold reset for 30 clock cycles
    btn_s1 = 1;  // release reset

    // Capture initial heartbeat state after a few clocks
    #(37 * 10);
    hb_initial = led[0];

    // ── Run for 15000 audio samples ─────────────────────────────────────
    // Each sample = 562 clocks, each clock ~37ns
    // Total: 15000 * 562 * 37 = ~312ms of simulated time
    // Extra margin: add 200 extra clocks per sample for pipeline latency
    #(15000 * 562 * 37 + 200 * 37);

    $fclose(fd);

    // ── Results ─────────────────────────────────────────────────────────
    $display("");
    $display("============================================================");
    $display("  tangamp_selftest Verification Results");
    $display("============================================================");
    $display("Total samples captured: %0d", sample_count);
    $display("VU checks (post-settle): %0d, VU on: %0d", vu_check_count, vu_on_count);
    $display("Heartbeat toggled: %s", hb_toggled ? "YES" : "NO");
    $display("Peak |audio_in|:   %0d (Q16.16)", peak_audio_in);
    $display("Peak |triode_out|: %0d (Q16.16)", peak_triode_out);
    $display("Peak |cab_out|:    %0d (Q16.16)", peak_cab_out);

    if (peak_audio_in > 0) begin
        $display("Triode gain: ~%0dx", peak_triode_out / (peak_audio_in > 0 ? peak_audio_in : 1));
        $display("Overall gain (in->cab): ~%0dx", peak_cab_out / (peak_audio_in > 0 ? peak_audio_in : 1));
    end

    $display("------------------------------------------------------------");

    // ── Check 1: Heartbeat ──────────────────────────────────────────────
    // Heartbeat toggles at 1Hz (13.5M clocks). Sim runs ~8.4M clocks,
    // so the counter should be well above 0 even if it hasn't toggled yet.
    hb_counting = (dut.hb_cnt > 25'd1000);
    if (hb_toggled || hb_counting) begin
        $display("CHECK 1: PASS - Heartbeat active (counter=%0d, toggled=%s)", dut.hb_cnt, hb_toggled ? "yes" : "no");
    end else begin
        $display("CHECK 1: FAIL - Heartbeat counter not advancing (counter=%0d)", dut.hb_cnt);
    end

    // ── Check 2: VU LEDs active during audio ────────────────────────────
    if (vu_on_count > 50) begin
        $display("CHECK 2: PASS - VU LEDs active during audio (%0d/%0d samples)", vu_on_count, vu_check_count);
    end else begin
        $display("CHECK 2: FAIL - VU LEDs not active (only %0d/%0d samples)", vu_on_count, vu_check_count);
    end

    // ── Check 3: Triode producing output ────────────────────────────────
    if (peak_triode_out > 0) begin
        $display("CHECK 3: PASS - Triode producing output (peak=%0d)", peak_triode_out);
    end else begin
        $display("CHECK 3: FAIL - Triode output is zero");
    end

    // ── Overall ─────────────────────────────────────────────────────────
    if ((hb_toggled || hb_counting) && vu_on_count > 50 && peak_triode_out > 0) begin
        $display("============================================================");
        $display("  OVERALL: PASS");
        $display("============================================================");
    end else begin
        $display("============================================================");
        $display("  OVERALL: FAIL");
        $display("============================================================");
    end

    $display("");
    $finish;
end

// ── Capture data at each sample_en tick ─────────────────────────────────
always @(posedge clk_27m) begin
    if (dut.sample_en) begin
        // Write to output file: sample_num, audio_in, triode_out, cab_out
        $fdisplay(fd, "%0d %0d %0d %0d",
            sample_count,
            dut.audio_in,
            dut.triode_out,
            dut.cab_out);

        // Track peak absolute values (for audio phase, sample > 11000)
        if (sample_count > 11000) begin
            // |audio_in|
            abs_val = dut.audio_in;
            if (abs_val < 0) abs_val = -abs_val;
            if (abs_val > peak_audio_in) peak_audio_in = abs_val;

            // |triode_out|
            abs_val = dut.triode_out;
            if (abs_val < 0) abs_val = -abs_val;
            if (abs_val > peak_triode_out) peak_triode_out = abs_val;

            // |cab_out|
            abs_val = dut.cab_out;
            if (abs_val < 0) abs_val = -abs_val;
            if (abs_val > peak_cab_out) peak_cab_out = abs_val;
        end

        sample_count = sample_count + 1;
    end
end

// ── Monitor heartbeat (check if LED[0] toggles) ────────────────────────
always @(posedge clk_27m) begin
    if (btn_s1 && !hb_toggled) begin
        if (led[0] != hb_initial) begin
            hb_toggled = 1;
        end
    end
end

// ── Monitor VU LEDs after settling (sample > 11500) ─────────────────────
always @(posedge clk_27m) begin
    if (dut.sample_en && sample_count > 11500) begin
        vu_check_count = vu_check_count + 1;
        // Active low: any of LED[1:5] being 0 means VU is on
        if (led[5:1] != 5'b11111) begin
            vu_on_count = vu_on_count + 1;
        end
    end
end

endmodule
