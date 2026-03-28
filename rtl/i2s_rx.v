// ============================================================================
// i2s_rx.v
// I2S Receiver for PCM1802 ADC
//
// Receives I2S serial bitstream and converts to Q16.16 parallel samples.
// I2S format: MSB first, 1 BCK delay after LRCK transition, 24-bit data.
//
// 24-bit to Q16.16 conversion: sign-extend to 32 bits (no shifting).
// The I2S frame carries Q16.16 values directly in the lower 24 bits,
// which covers ±128V — more than enough for guitar-level signals.
// ============================================================================

module i2s_rx (
    input  wire        clk,         // 27MHz
    input  wire        rst_n,
    input  wire        bck,         // bit clock
    input  wire        lrck,        // word clock
    input  wire        din,         // serial data from ADC
    output reg  signed [31:0] audio_l,  // Q16.16 left channel
    output reg         valid        // pulse when new left sample ready
);

// ── BCK rising edge detection (2-FF synchronizer) ──────────────────────────
reg bck_r1, bck_r2;
wire bck_rise = bck_r1 & ~bck_r2;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        bck_r1 <= 1'b0;
        bck_r2 <= 1'b0;
    end else begin
        bck_r1 <= bck;
        bck_r2 <= bck_r1;
    end
end

// ── LRCK edge detection ───────────────────────────────────────────────────
reg lrck_r1, lrck_r2;
wire lrck_fall = ~lrck_r1 & lrck_r2;  // falling edge = left channel start
wire lrck_rise = lrck_r1 & ~lrck_r2;  // rising edge = right channel start

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        lrck_r1 <= 1'b0;
        lrck_r2 <= 1'b0;
    end else begin
        lrck_r1 <= lrck;
        lrck_r2 <= lrck_r1;
    end
end

// ── DIN synchronizer ───────────────────────────────────────────────────────
reg din_r1, din_r2;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        din_r1 <= 1'b0;
        din_r2 <= 1'b0;
    end else begin
        din_r1 <= din;
        din_r2 <= din_r1;
    end
end

// ── Bit capture state machine ──────────────────────────────────────────────
reg [5:0] bit_cnt;        // bit position counter
reg       capturing;      // actively capturing bits
reg       skip_first;     // skip first BCK for I2S 1-bit delay
reg [23:0] shift_reg;     // 24-bit shift register
reg       is_left;        // currently capturing left channel

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        bit_cnt    <= 6'd0;
        capturing  <= 1'b0;
        skip_first <= 1'b0;
        shift_reg  <= 24'd0;
        is_left    <= 1'b0;
        audio_l    <= 32'sd0;
        valid      <= 1'b0;
    end else begin
        valid <= 1'b0;

        // Detect LRCK transitions (synchronized)
        if (lrck_fall) begin
            // Left channel starting
            // If we were capturing right channel, ignore it (we only output left)
            capturing  <= 1'b1;
            skip_first <= 1'b1;
            bit_cnt    <= 6'd0;
            shift_reg  <= 24'd0;
            is_left    <= 1'b1;
        end else if (lrck_rise) begin
            // Right channel starting - output left channel result
            if (is_left && bit_cnt >= 6'd24) begin
                // Sign-extend 24-bit to 32-bit Q16.16
                // I2S frame carries Q16.16 values directly (fits in 24 bits
                // for guitar-level signals up to ±128V)
                audio_l <= {{8{shift_reg[23]}}, shift_reg};
                valid   <= 1'b1;
            end
            capturing  <= 1'b0;
            is_left    <= 1'b0;
        end

        // Shift in data on BCK rising edge
        if (bck_rise && capturing) begin
            if (skip_first) begin
                // Skip first BCK for I2S 1-bit delay
                skip_first <= 1'b0;
            end else if (bit_cnt < 6'd24) begin
                shift_reg <= {shift_reg[22:0], din_r2};
                bit_cnt   <= bit_cnt + 6'd1;
            end
        end
    end
end

endmodule
