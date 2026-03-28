// ============================================================================
// tangamp_selftest.v
// Self-test top-level for Tang Nano 20K — no external ADC/DAC needed.
//
// Generates a 440Hz sine internally, processes through WDF triode,
// and displays output level on the 6 onboard LEDs as a VU meter.
//
// LED[0] = heartbeat (proves clock running)
// LED[1:5] = output level (VU meter, 5 segments)
//
// Tang Nano 20K: Gowin GW2AR-LV18QN88C8/I7 (or GW2A-LV18QN88C8/I7)
// Clock: 27MHz on pin 52
// LEDs: active LOW on pins 15,16,17,18,19,20
// Buttons: S1=pin 88 (active low), S2=pin 87
// ============================================================================

module tangamp_selftest (
    input  wire clk_27m,     // 27MHz crystal, pin 52
    input  wire btn_s1,      // Button S1, pin 88, active low
    output wire [5:0] led    // LEDs, pins 15-20, active low
);

// ── Reset ────────────────────────────────────────────────────────────────
reg [1:0] rst_sync;
reg rst_n;
always @(posedge clk_27m) begin
    rst_sync <= {rst_sync[0], btn_s1};
    rst_n <= rst_sync[1];
end

// ── Sample rate clock (48kHz from 27MHz) ───��─────────────────────────────
// 27MHz / 562 = 48043Hz (close enough to 48kHz)
reg [9:0] div_cnt;
reg sample_en;

always @(posedge clk_27m or negedge rst_n) begin
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

// ── Sine wave generator (440Hz at 48kHz = ~109 samples/period) ──────────
// 0.5V amplitude in Q16.16 = 32768
reg signed [31:0] sine_table [0:107];
integer i;
initial begin
    for (i = 0; i < 108; i = i + 1)
        sine_table[i] = $rtoi($sin(6.2831853 * i / 108.0) * 0.5 * 65536.0);
end

reg [6:0] sine_idx;
reg signed [31:0] audio_in;
reg [15:0] settle_cnt;
localparam SETTLE_SAMPLES = 16'd11000;  // silence for DC settling

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        sine_idx <= 0;
        audio_in <= 0;
        settle_cnt <= 0;
    end else if (sample_en) begin
        if (settle_cnt < SETTLE_SAMPLES) begin
            audio_in <= 0;  // silence during settling
            settle_cnt <= settle_cnt + 1;
        end else begin
            audio_in <= sine_table[sine_idx];
            sine_idx <= (sine_idx >= 7'd107) ? 0 : sine_idx + 1;
        end
    end
end

// ── WDF Triode Stage ────────────────���─────────────────────���──────────────
wire signed [31:0] audio_out;
wire out_valid;

wdf_triode_wdf triode (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .sample_en (sample_en),
    .audio_in  (audio_in),
    .audio_out (audio_out),
    .out_valid (out_valid)
);

// ── VU Meter (output level → LED bar) ─────────────────���──────────────────
// Take absolute value of output, map to 5 LED thresholds
reg [31:0] abs_out;
reg [4:0] vu;

always @(posedge clk_27m) begin
    if (out_valid) begin
        abs_out <= (audio_out[31]) ? -audio_out : audio_out;
        // Thresholds in Q16.16 (approximately 1V, 3V, 8V, 15V, 25V)
        vu[0] <= (abs_out > 32'd65536);      // > 1V
        vu[1] <= (abs_out > 32'd196608);     // > 3V
        vu[2] <= (abs_out > 32'd524288);     // > 8V
        vu[3] <= (abs_out > 32'd983040);     // > 15V
        vu[4] <= (abs_out > 32'd1638400);    // > 25V
    end
end

// ── Heartbeat (LED 0 blinks at ~1Hz) ─────────────────────────────────────
reg [24:0] hb_cnt;
reg hb;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        hb_cnt <= 0;
        hb <= 0;
    end else begin
        if (hb_cnt >= 25'd13_500_000) begin  // 27MHz / 2 / 13.5M = 1Hz
            hb_cnt <= 0;
            hb <= ~hb;
        end else begin
            hb_cnt <= hb_cnt + 1;
        end
    end
end

// ── LED output (active low on Tang Nano 20K) ──────���──────────────────────
assign led[0] = ~hb;
assign led[1] = ~vu[0];
assign led[2] = ~vu[1];
assign led[3] = ~vu[2];
assign led[4] = ~vu[3];
assign led[5] = ~vu[4];

endmodule
