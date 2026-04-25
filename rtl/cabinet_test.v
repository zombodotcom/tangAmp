// ============================================================================
// cabinet_test.v
// Cabinet IR isolation test — ADC -> cabinet_fir -> DAC
// No triode, no tone stack. Hear what the cab sim does to raw guitar.
//
// S1 = cycle through cab IRs (SD card: sd_spi_reader + sd_ir_loader)
// S2 = bypass toggle (direct vs cab IR)
//
// LED[0] = heartbeat (1Hz)
// LED[1] = ADC activity
// LED[2:4] = IR index (binary)
// LED[5] = bypass indicator (lit = bypassed)
// ============================================================================

module cabinet_test (
    input  wire clk_27m,
    input  wire btn_s1,
    input  wire btn_s2,

    output wire adc_bck,
    output wire adc_lrck,
    input  wire adc_dout,
    output wire dac_din,
    output wire mclk_out,    // 18MHz MCLK to PCM1808 SCK (pin 75)

    output wire sd_clk,
    output wire sd_cmd,
    input  wire sd_dat0,
    output wire sd_dat1,
    output wire sd_dat2,
    output wire sd_dat3,

    output wire [5:0] led
);

// SD card unused data lines held high (SPI mode)
assign sd_dat1 = 1'b1;
assign sd_dat2 = 1'b1;

// ── Power-on reset ───────────────────────────────────────────────────────
reg [9:0] por_cnt;
reg rst_n;

always @(posedge clk_27m) begin
    if (por_cnt < 10'd1023) begin
        por_cnt <= por_cnt + 1;
        rst_n <= 1'b0;
    end else
        rst_n <= 1'b1;
end

// ── Audio Clocks ─────────────────────────────────────────────────────────
wire mclk, bck, lrck, sample_en, pll_lock;

clk_audio_pll u_clkgen (
    .clk_27m   (clk_27m),
    .rst_n     (rst_n),
    .mclk      (mclk),
    .bck       (bck),
    .lrck      (lrck),
    .sample_en (sample_en),
    .pll_lock  (pll_lock)
);

assign mclk_out = mclk;

assign adc_bck  = bck;
assign adc_lrck = lrck;

// ── I2S Receiver (ADC) ──────────────────────────────────────────────────
wire signed [31:0] adc_audio;
wire               adc_valid;

i2s_rx u_rx (
    .clk     (clk_27m),
    .rst_n   (rst_n),
    .bck     (bck),
    .lrck    (lrck),
    .din     (adc_dout),
    .audio_l (adc_audio),
    .valid   (adc_valid)
);

// ── S1 debounce: IR cycling ─────────────────────────────────────────────
reg [15:0] s1_debounce;
reg s1_prev;
reg ir_next_pulse;

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        s1_debounce <= 0;
        s1_prev <= 1;
        ir_next_pulse <= 0;
    end else begin
        ir_next_pulse <= 0;
        s1_prev <= btn_s1;
        if (!btn_s1 && s1_prev && s1_debounce == 0) begin
            ir_next_pulse <= 1;
            s1_debounce <= 16'hFFFF;
        end
        if (s1_debounce > 0)
            s1_debounce <= s1_debounce - 1;
    end
end

// ── S2 debounce: bypass toggle ──────────────────────────────────────────
reg [15:0] s2_debounce;
reg s2_prev;
reg bypass_mode;

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        s2_debounce <= 0;
        s2_prev <= 1;
        bypass_mode <= 0;
    end else begin
        s2_prev <= btn_s2;
        if (!btn_s2 && s2_prev && s2_debounce == 0) begin
            bypass_mode <= ~bypass_mode;
            s2_debounce <= 16'hFFFF;
        end
        if (s2_debounce > 0)
            s2_debounce <= s2_debounce - 1;
    end
end

// ── SD Card SPI Reader + IR Loader ──────────────────────────────────────
wire        sd_start_read;
wire [31:0] sd_sector_addr;
wire        sd_init_done;
wire        sd_read_done;
wire        sd_error;
wire        sd_byte_valid;
wire [7:0]  sd_byte_data;
wire [9:0]  sd_byte_index;
wire        sd_tap_wr_en;
wire [7:0]  sd_tap_wr_addr;
wire [15:0] sd_tap_wr_data;
wire        ir_sector_active;

sd_spi_reader u_sd_reader (
    .clk            (clk_27m),
    .rst_n          (rst_n),
    .start_read     (sd_start_read),
    .sector_addr    (sd_sector_addr),
    .sd_clk_o       (sd_clk),
    .sd_mosi        (sd_cmd),
    .sd_miso        (sd_dat0),
    .sd_cs_n        (sd_dat3),
    .tap_wr_en      (sd_tap_wr_en),
    .tap_wr_addr    (sd_tap_wr_addr),
    .tap_wr_data    (sd_tap_wr_data),
    .byte_out_valid (sd_byte_valid),
    .byte_out_data  (sd_byte_data),
    .byte_out_index (sd_byte_index),
    .init_done      (sd_init_done),
    .read_done      (sd_read_done),
    .error          (sd_error)
);

wire [3:0] current_ir;
wire [3:0] sd_num_irs;
wire       ir_loaded;
wire       ir_loading;
wire       header_ok;

sd_ir_loader u_ir_loader (
    .clk              (clk_27m),
    .rst_n            (rst_n),
    .btn_next         (~ir_next_pulse),
    .sd_start_read    (sd_start_read),
    .sd_sector_addr   (sd_sector_addr),
    .sd_init_done     (sd_init_done),
    .sd_read_done     (sd_read_done),
    .sd_error         (sd_error),
    .sd_byte_valid    (sd_byte_valid),
    .sd_byte_data     (sd_byte_data),
    .sd_byte_index    (sd_byte_index),
    .ir_sector_active (ir_sector_active),
    .current_ir       (current_ir),
    .num_irs          (sd_num_irs),
    .ir_loaded        (ir_loaded),
    .loading          (ir_loading),
    .header_ok        (header_ok)
);

// Gate tap writes: only during IR sector reads
wire        cab_tap_wr_en   = sd_tap_wr_en & ir_sector_active;
wire [7:0]  cab_tap_wr_addr = sd_tap_wr_addr;
wire [15:0] cab_tap_wr_data = sd_tap_wr_data;

// ── Cabinet IR (256-tap FIR) ────────────────────────────────────────────
wire signed [31:0] cab_out;
wire cab_valid;

cabinet_fir #(
    .N_TAPS (256)
) cabinet (
    .clk         (clk_27m),
    .rst_n       (rst_n),
    .sample_en   (adc_valid),
    .audio_in    (adc_audio),
    .audio_out   (cab_out),
    .out_valid   (cab_valid),
    .tap_wr_en   (cab_tap_wr_en),
    .tap_wr_addr (cab_tap_wr_addr),
    .tap_wr_data (cab_tap_wr_data)
);

// ── Output mux ──────────────────────────────────────────────────────────
wire signed [31:0] dac_audio = bypass_mode ? adc_audio : cab_out;
wire               dac_load  = bypass_mode ? adc_valid : cab_valid;

// ── I2S Transmitter (DAC) ───────────────────────────────────────────────
i2s_tx u_tx (
    .clk     (clk_27m),
    .rst_n   (rst_n),
    .bck     (bck),
    .lrck    (lrck),
    .audio_l (dac_audio),
    .audio_r (dac_audio),
    .load    (dac_load),
    .dout    (dac_din)
);

// ── LED[0]: Heartbeat (~1Hz) ────────────────────────────────────────────
reg [24:0] hb_cnt;
reg hb;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin hb_cnt <= 0; hb <= 0; end
    else begin
        if (hb_cnt >= 25'd13_500_000) begin hb_cnt <= 0; hb <= ~hb; end
        else hb_cnt <= hb_cnt + 1;
    end
end

// ── LED[1]: ADC activity ────────────────────────────────────────────────
reg [19:0] act_cnt;
reg act_led;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin act_cnt <= 0; act_led <= 0; end
    else begin
        if (adc_valid) begin act_led <= 1; act_cnt <= 0; end
        else if (act_cnt < 20'd675_000) act_cnt <= act_cnt + 1;
        else act_led <= 0;
    end
end

// Active-low LEDs
assign led[0] = ~hb;
assign led[1] = ~act_led;
assign led[2] = ~current_ir[0];
assign led[3] = ~current_ir[1];
assign led[4] = ~current_ir[2];
assign led[5] = ~bypass_mode;

endmodule
