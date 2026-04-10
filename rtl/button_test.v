// ============================================================================
// button_test.v
// Interactive button + LED test for Tang Nano 20K.
//
// LED[0] = heartbeat (proves clock running)
// LED[1] = S1 held (lights while pressed)
// LED[2] = S1 toggle (press to toggle on/off)
// LED[3] = S2 held (lights while pressed)
// LED[4] = S2 toggle (press to toggle on/off)
// LED[5] = both buttons held simultaneously
// ============================================================================

module button_test (
    input  wire clk_27m,
    input  wire btn_s1,    // pin 88, active low
    input  wire btn_s2,    // pin 87, active low
    output wire [5:0] led
);

// ── Heartbeat ────────────────────────────────────────────────────────────
reg [24:0] hb_cnt;
reg hb;
always @(posedge clk_27m) begin
    if (hb_cnt >= 25'd13_500_000) begin
        hb_cnt <= 0;
        hb <= ~hb;
    end else
        hb_cnt <= hb_cnt + 1;
end

// ── Debounce S1 ──────────────────────────────────────────────────────────
reg [15:0] s1_db;
reg s1_prev, s1_toggle;
always @(posedge clk_27m) begin
    s1_prev <= btn_s1;
    if (!btn_s1 && s1_prev) begin
        if (s1_db == 0) begin
            s1_toggle <= ~s1_toggle;
            s1_db <= 16'hFFFF;
        end
    end
    if (s1_db > 0)
        s1_db <= s1_db - 1;
end

// ── Debounce S2 ──────────────────────────────────────────────────────────
reg [15:0] s2_db;
reg s2_prev, s2_toggle;
always @(posedge clk_27m) begin
    s2_prev <= btn_s2;
    if (!btn_s2 && s2_prev) begin
        if (s2_db == 0) begin
            s2_toggle <= ~s2_toggle;
            s2_db <= 16'hFFFF;
        end
    end
    if (s2_db > 0)
        s2_db <= s2_db - 1;
end

// Active-low LEDs
assign led[0] = ~hb;                        // heartbeat
assign led[1] = btn_s1;                     // S1 held (active low -> LED on when pressed)
assign led[2] = ~s1_toggle;                 // S1 toggle
assign led[3] = btn_s2;                     // S2 held
assign led[4] = ~s2_toggle;                 // S2 toggle
assign led[5] = btn_s1 | btn_s2;            // both held (NOR: on when both low)

endmodule
