// ============================================================================
// clk_audio_gen.v
// Audio clock generator for I2S interface
//
// Divides 27MHz system clock to produce:
//   BCK  = ~3MHz bit clock (27MHz / 9)
//   LRCK = ~46.875kHz word clock (BCK / 64)
//   sample_en = one pulse per LRCK falling edge (left channel start)
//
// Target: Tang Nano 20K (Gowin GW2A), 27MHz oscillator
// ============================================================================

module clk_audio_gen (
    input  wire clk,          // 27MHz
    input  wire rst_n,
    output reg  bck,           // ~3MHz bit clock (27/9 = 3MHz)
    output reg  lrck,          // ~46.875kHz word clock (3M/64)
    output wire sample_en      // one pulse per LRCK falling edge
);

// ── BCK generation: divide 27MHz by 9 ──────────────────────────────────────
// Toggle pattern: high for 5 clocks, low for 4 clocks (period = 9 clocks)
reg [3:0] bck_cnt;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        bck_cnt <= 4'd0;
        bck     <= 1'b1;
    end else begin
        if (bck_cnt == 4'd8) begin
            bck_cnt <= 4'd0;
            bck     <= 1'b1;
        end else begin
            bck_cnt <= bck_cnt + 4'd1;
            if (bck_cnt == 4'd4)
                bck <= 1'b0;
        end
    end
end

// ── Detect BCK rising edge ─────────────────────────────────────────────────
reg bck_d;
wire bck_rise = bck & ~bck_d;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        bck_d <= 1'b0;
    else
        bck_d <= bck;
end

// ── LRCK generation: count 64 BCK rising edges ────────────────────────────
// 32 BCK per channel, toggle LRCK every 32 BCK edges
reg [5:0] lrck_cnt;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        lrck_cnt <= 6'd0;
        lrck     <= 1'b1;
    end else if (bck_rise) begin
        if (lrck_cnt == 6'd31) begin
            lrck_cnt <= 6'd0;
            lrck     <= ~lrck;
        end else begin
            lrck_cnt <= lrck_cnt + 6'd1;
        end
    end
end

// ── sample_en: pulse on LRCK falling edge ──────────────────────────────────
reg lrck_d;
wire lrck_fall = ~lrck & lrck_d;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        lrck_d <= 1'b0;
    else
        lrck_d <= lrck;
end

assign sample_en = lrck_fall;

endmodule
