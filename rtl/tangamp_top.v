// ============================================================================
// tangamp_top.v
// Top-Level Module for tangAmp FPGA Tube Amp Emulator
//
// Full signal chain with real I2S ADC/DAC:
//   PCM1802 ADC -> noise_gate -> NFB subtract -> [2x oversample] ->
//   triode_engine (2-stage 12AX7 + 6L6 power amp) -> [downsample] ->
//   tone_stack_iir -> output_transformer -> cabinet_fir -> PCM5102 DAC
//                                              |
//                                    nfb_register <---+
//                                    power_supply_sag <-+
//
// Target: Tang Nano 20K (Gowin GW2AR-LV18QN88C8/I7, 27MHz)
// I2S: PCM1802 ADC (24-bit) + PCM5102 DAC (32-bit)
// ============================================================================

module tangamp_top (
    input  wire clk_27m,       // 27MHz crystal, pin 52
    input  wire btn_s1,        // Button S1, pin 88, active low (reset)
    input  wire btn_s2,        // Button S2, pin 87, active low (bypass toggle)

    // I2S clocks (shared between ADC and DAC — wire both to these pins)
    output wire adc_bck,       // bit clock  (pin 76 → PCM1802.BCK + PCM5102.BCK)
    output wire adc_lrck,      // word clock (pin 77 → PCM1802.LRCK + PCM5102.LRCK)

    // I2S data
    input  wire adc_dout,      // serial data from PCM1802 ADC (pin 48)
    output wire dac_din,       // serial data to PCM5102 DAC (pin 49)

    // SD card SPI (onboard MicroSD slot)
    output wire sd_clk,
    output wire sd_cmd,
    input  wire sd_dat0,
    output wire sd_dat1,
    output wire sd_dat2,
    output wire sd_dat3,

    // Status LEDs
    output wire [5:0] led
);

// ── SD card unused data lines held high (SPI mode) ────────────────────────
assign sd_dat1 = 1'b1;
assign sd_dat2 = 1'b1;

// ── Reset synchronizer (2-FF) ──────────────────────────────────────────────
reg [1:0] rst_sync;
reg rst_n;
always @(posedge clk_27m) begin
    rst_sync <= {rst_sync[0], btn_s1};
    rst_n <= rst_sync[1];
end

// ── Bypass Mode Toggle (S2 debounce) ──────────────────────────────────────
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
        if (!btn_s2 && s2_prev) begin
            // Falling edge detected — toggle bypass
            if (s2_debounce == 0) begin
                bypass_mode <= ~bypass_mode;
                s2_debounce <= 16'hFFFF;  // ~2.4ms debounce at 27MHz
            end
        end
        if (s2_debounce > 0)
            s2_debounce <= s2_debounce - 1;
    end
end

// ── Audio Clock Generator ──────────────────────────────────────────────────
wire bck, lrck, sample_en;

clk_audio_gen u_clkgen (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .bck       (bck),
    .lrck      (lrck),
    .sample_en (sample_en)
);

// BCK/LRCK outputs — physically wired to both ADC and DAC
assign adc_bck  = bck;
assign adc_lrck = lrck;

// ── I2S Receiver (ADC -> Q16.16) ───────────────────────────────────────────
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

// ── Input gain scaling ─────────────────────────────────────────────────────
// Guitar pickups output ~100-500mV peak. ADC full scale maps to ±1.0 in Q16.16.
// Tube preamp expects ~0.5-2V input. Scale by 4x (shift left 2).
wire signed [31:0] scaled_in = adc_audio <<< 2;

// ── Noise Gate ─────────────────────────────────────────────────────────────
wire signed [31:0] gated_in;

noise_gate #(
    .FP_FRAC  (16),
    .FP_WIDTH (32)
) ngate (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .sample_en (adc_valid),
    .audio_in  (scaled_in),
    .audio_out (gated_in),
    .threshold (8'h08)       // low threshold — tune with pot later
);

// ── Negative Feedback Subtraction ──────────────────────────────────────────
wire signed [31:0] nfb_signal;
reg  signed [31:0] triode_input;

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n)
        triode_input <= 0;
    else if (adc_valid)
        triode_input <= gated_in - nfb_signal;
end

// ── 2x Oversampling + Triode Engine ────────────────────────────────────────
wire        up_valid_a, up_valid_b;
wire signed [31:0] audio_up;

reg         triode_start;
reg  signed [31:0] triode_in;
wire signed [31:0] triode_out;
wire        triode_valid;

reg  signed [31:0] proc_a_sample;
reg         proc_a_valid;
reg  signed [31:0] proc_b_sample;
reg         proc_b_valid;

wire signed [31:0] audio_down;
wire        down_valid;

oversample_2x oversamp (
    .clk            (clk_27m),
    .rst_n          (rst_n),
    .sample_en_48k  (adc_valid),
    .audio_in       (triode_input),
    .up_valid_a     (up_valid_a),
    .up_valid_b     (up_valid_b),
    .audio_up       (audio_up),
    .processed_a    (proc_a_sample),
    .proc_valid_a   (proc_a_valid),
    .processed_b    (proc_b_sample),
    .proc_valid_b   (proc_b_valid),
    .audio_down     (audio_down),
    .down_valid     (down_valid)
);

triode_engine #(
    .NUM_STAGES  (2),
    .POWER_AMP   (1)
) triode (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .sample_en (triode_start),
    .audio_in  (triode_in),
    .audio_out (triode_out),
    .out_valid (triode_valid)
);

// ── Oversampling Controller ────────────────────────────────────────────────
localparam OS_IDLE       = 3'd0;
localparam OS_WAIT_TRI_A = 3'd1;
localparam OS_START_B    = 3'd2;
localparam OS_WAIT_TRI_B = 3'd3;

reg [2:0] os_state;
reg signed [31:0] up_b_held;
reg signed [31:0] triode_a_held;

always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        os_state      <= OS_IDLE;
        triode_start  <= 1'b0;
        triode_in     <= 0;
        proc_a_valid  <= 1'b0;
        proc_b_valid  <= 1'b0;
        proc_a_sample <= 0;
        proc_b_sample <= 0;
        up_b_held     <= 0;
        triode_a_held <= 0;
    end else begin
        triode_start <= 1'b0;
        proc_a_valid <= 1'b0;
        proc_b_valid <= 1'b0;

        case (os_state)
        OS_IDLE: begin
            if (up_valid_a) begin
                triode_in    <= audio_up;
                triode_start <= 1'b1;
                os_state     <= OS_WAIT_TRI_A;
            end
        end
        OS_WAIT_TRI_A: begin
            if (up_valid_b)
                up_b_held <= audio_up;
            if (triode_valid) begin
                triode_a_held <= triode_out;
                os_state      <= OS_START_B;
            end
        end
        OS_START_B: begin
            triode_in    <= up_b_held;
            triode_start <= 1'b1;
            os_state     <= OS_WAIT_TRI_B;
        end
        OS_WAIT_TRI_B: begin
            if (triode_valid) begin
                proc_a_sample <= triode_a_held;
                proc_a_valid  <= 1'b1;
                proc_b_sample <= triode_out;
                proc_b_valid  <= 1'b1;
                os_state      <= OS_IDLE;
            end
        end
        default: os_state <= OS_IDLE;
        endcase
    end
end

// ── Tone Stack (3-band biquad EQ) ──────────────────────────────────────────
wire signed [31:0] tone_out;
wire tone_valid;

tone_stack_iir tone (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .sample_en (down_valid),
    .audio_in  (audio_down),
    .audio_out (tone_out),
    .out_valid (tone_valid)
);

// ── Output Transformer (HPF + LPF + soft clip) ────────────────────────────
wire signed [31:0] xf_out;
wire xf_valid;

output_transformer xformer (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .sample_en (tone_valid),
    .audio_in  (tone_out),
    .audio_out (xf_out),
    .out_valid (xf_valid)
);

// ── Power Supply Sag ───────────────────────────────────────────────────────
wire signed [31:0] b_plus_out;

power_supply_sag #(
    .SAG_SHIFT (3)
) psu_sag (
    .clk       (clk_27m),
    .rst_n     (rst_n),
    .sample_en (xf_valid),
    .audio_in  (xf_out),
    .b_plus    (b_plus_out)
);

// ── SD Card SPI Reader + IR Loader ─────────────────────────────────────────
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
    .btn_next         (1'b1),    // No button cycling yet (active low, held high = no press)
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

// Gate tap writes: only pass through during IR sector reads (not header)
wire        cab_tap_wr_en   = sd_tap_wr_en & ir_sector_active;
wire [7:0]  cab_tap_wr_addr = sd_tap_wr_addr;
wire [15:0] cab_tap_wr_data = sd_tap_wr_data;

// ── Cabinet IR (256-tap FIR) ───────────────────────────────────────────────
wire signed [31:0] cab_out;
wire cab_valid;

cabinet_fir #(
    .N_TAPS (129)
) cabinet (
    .clk         (clk_27m),
    .rst_n       (rst_n),
    .sample_en   (xf_valid),
    .audio_in    (xf_out),
    .audio_out   (cab_out),
    .out_valid   (cab_valid),
    .tap_wr_en   (cab_tap_wr_en),
    .tap_wr_addr (cab_tap_wr_addr),
    .tap_wr_data (cab_tap_wr_data)
);

// ── Negative Feedback Register ─────────────────────────────────────────────
nfb_register #(
    .NFB_SHIFT (3)
) nfb (
    .clk        (clk_27m),
    .rst_n      (rst_n),
    .fb_in      (cab_out),
    .fb_valid   (cab_valid),
    .nfb_signal (nfb_signal)
);

// ── Output scaling for DAC ─────────────────────────────────────────────────
// cab_out is in Q16.16 with typical range ±50V.
// DAC expects values near full scale. Shift right to avoid clipping,
// then the DAC's own output stage handles the analog level.
// In bypass mode, send scaled ADC input directly to DAC
// In normal mode, send processed cab_out
wire signed [31:0] dac_audio = bypass_mode ? (adc_audio <<< 2) : (cab_out >>> 2);

// ── I2S Transmitter (DAC) ──────────────────────────────────────────────────
i2s_tx u_tx (
    .clk     (clk_27m),
    .rst_n   (rst_n),
    .bck     (bck),              // internal bck (same signal driving adc_bck pin)
    .lrck    (lrck),             // internal lrck (same signal driving adc_lrck pin)
    .audio_l (dac_audio),
    .audio_r (dac_audio),        // mono: same signal both channels
    .load    (bypass_mode ? adc_valid : cab_valid),
    .dout    (dac_din)
);

// ── LED Indicators ─────────────────────────────────────────────────────────

// LED[0]: Heartbeat (~1Hz)
reg [24:0] hb_cnt;
reg hb;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        hb_cnt <= 0;
        hb <= 0;
    end else begin
        if (hb_cnt >= 25'd13_500_000) begin
            hb_cnt <= 0;
            hb <= ~hb;
        end else
            hb_cnt <= hb_cnt + 1;
    end
end

// LED[1]: ADC activity (blinks when receiving samples)
reg [19:0] act_cnt;
reg act_led;
always @(posedge clk_27m or negedge rst_n) begin
    if (!rst_n) begin
        act_cnt <= 0;
        act_led <= 0;
    end else begin
        if (adc_valid) begin
            act_led <= 1'b1;
            act_cnt <= 0;
        end else if (act_cnt < 20'd675_000)
            act_cnt <= act_cnt + 1;
        else
            act_led <= 1'b0;
    end
end

// LED[2:5]: VU meter (output level)
reg [31:0] abs_out;
reg [3:0] vu;
always @(posedge clk_27m) begin
    if (cab_valid) begin
        abs_out <= cab_out[31] ? -cab_out : cab_out;
        vu[0] <= (abs_out > 32'd65536);     // > 1V
        vu[1] <= (abs_out > 32'd196608);    // > 3V
        vu[2] <= (abs_out > 32'd524288);    // > 8V
        vu[3] <= (abs_out > 32'd983040);    // > 15V
    end
end

// Tang Nano 20K LEDs are active-low
assign led[0] = ~hb;
assign led[1] = ~act_led;
assign led[2] = ~vu[0];
assign led[3] = ~vu[1];
assign led[4] = ~vu[2];
assign led[5] = ~bypass_mode;  // lit when bypass active

endmodule
