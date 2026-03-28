// ============================================================================
// i2s_tx.v
// I2S Transmitter for PCM5102 DAC
//
// Converts parallel Q16.16 audio samples to I2S serial bitstream.
// I2S format: MSB first, 1 BCK delay after LRCK transition, 24-bit
// left-justified in 32-bit frame.
//
// Q16.16 -> 24-bit conversion: saturate to ±8388607 then send [23:0].
// 24-bit signed range covers ±128V in Q16.16; values beyond are clamped.
// ============================================================================

module i2s_tx (
    input  wire        clk,         // 27MHz
    input  wire        rst_n,
    input  wire        bck,         // bit clock
    input  wire        lrck,        // word clock
    input  wire signed [31:0] audio_l,  // left channel Q16.16
    input  wire signed [31:0] audio_r,  // right channel Q16.16
    input  wire        load,        // pulse to latch new samples
    output reg         dout         // serial data to DAC
);

// ── Sample latches ─────────────────────────────────────────────────────────
// Convert Q16.16 to 24-bit I2S: saturate to ±8388607 then left-justify.
// 24-bit signed range: -8388608 .. +8388607 (±128V in Q16.16).
// If the value exceeds this range, clamp to avoid wrap-around distortion.
reg [31:0] shift_l;
reg [31:0] shift_r;

// Saturation helper: clamp Q16.16 to 24-bit range and left-justify
function [31:0] saturate_24;
    input signed [31:0] val;
    reg signed [31:0] clamped;
    begin
        if (val > 32'sd8388607)
            clamped = 32'sd8388607;
        else if (val < -32'sd8388608)
            clamped = -32'sd8388608;
        else
            clamped = val;
        saturate_24 = {clamped[23:0], 8'd0};
    end
endfunction

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        shift_l <= 32'd0;
        shift_r <= 32'd0;
    end else if (load) begin
        shift_l <= saturate_24(audio_l);
        shift_r <= saturate_24(audio_r);
    end
end

// ── BCK edge detection ─────────────────────────────────────────────────────
reg bck_d;
wire bck_fall = ~bck & bck_d;
wire bck_rise = bck & ~bck_d;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        bck_d <= 1'b0;
    else
        bck_d <= bck;
end

// ── LRCK edge detection ───────────────────────────────────────────────────
reg lrck_d;
wire lrck_fall = ~lrck & lrck_d;
wire lrck_rise_w = lrck & ~lrck_d;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        lrck_d <= 1'b0;
    else
        lrck_d <= lrck;
end

// ── Bit counter and shift-out ──────────────────────────────────────────────
// I2S: 1 BCK delay after LRCK transition, then MSB first for 32 bits
// Strategy: on LRCK edge, load SR and reset. Count BCK falls from there.
// bit_cnt=0: LRCK edge just happened, output 0 (1-BCK delay).
// bit_cnt=1..32: shift out MSB first.
reg [5:0] bit_cnt;
reg [31:0] sr;
reg       active;   // channel is active (have data to shift)

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        bit_cnt <= 6'd0;
        sr      <= 32'd0;
        dout    <= 1'b0;
        active  <= 1'b0;
    end else begin
        // LRCK edges: load new channel data, reset bit counter
        if (lrck_fall) begin
            // Left channel starting
            sr      <= shift_l;
            bit_cnt <= 6'd0;
            active  <= 1'b1;
            dout    <= 1'b0;  // 1-BCK delay: output 0
        end else if (lrck_rise_w) begin
            // Right channel starting
            sr      <= shift_r;
            bit_cnt <= 6'd0;
            active  <= 1'b1;
            dout    <= 1'b0;
        end else if (bck_fall && active) begin
            if (bit_cnt == 6'd0) begin
                // First BCK fall after LRCK edge: still in 1-BCK delay period.
                // The RX skips the BCK rise that follows this fall, so we must
                // not output data yet. Keep dout=0 and advance counter.
                dout    <= 1'b0;
                bit_cnt <= 6'd1;
            end else if (bit_cnt < 6'd33) begin
                dout    <= sr[31];
                sr      <= {sr[30:0], 1'b0};
                bit_cnt <= bit_cnt + 6'd1;
            end else begin
                dout <= 1'b0;
            end
        end
    end
end

endmodule
