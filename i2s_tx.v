// ============================================================================
// i2s_tx.v
// I2S Transmitter for PCM5102 DAC
//
// Converts parallel Q16.16 audio samples to I2S serial bitstream.
// I2S format: MSB first, 1 BCK delay after LRCK transition, 24-bit
// left-justified in 32-bit frame.
//
// Q16.16 -> 24-bit conversion: take bits [31:8] (sign-extend top 8 bits
// are already there since Q16.16 integer part is 16 bits, we want the
// top 24 significant bits).
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
// Convert Q16.16 to 24-bit: saturate/clamp then take top 24 bits
// For a Q16.16 value, the integer range is -32768..32767
// 24-bit range is -8388608..8388607
// Shift Q16.16 left by 8 to get 24-bit left-justified in 32-bit frame:
//   audio[31:8] gives us the 24 MSBs (16 integer + 8 fraction bits)
reg [31:0] shift_l;
reg [31:0] shift_r;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        shift_l <= 32'd0;
        shift_r <= 32'd0;
    end else if (load) begin
        // Q16.16 to 24-bit left-justified in 32-bit:
        // Take bits [31:8] as 24-bit value, pad 8 LSBs with 0
        shift_l <= {audio_l[31:8], 8'd0};
        shift_r <= {audio_r[31:8], 8'd0};
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
                // First BCK fall after LRCK edge = end of 1-BCK delay
                // Output MSB now
                dout    <= sr[31];
                sr      <= {sr[30:0], 1'b0};
                bit_cnt <= 6'd1;
            end else if (bit_cnt < 6'd32) begin
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
