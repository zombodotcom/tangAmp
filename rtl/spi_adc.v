// ============================================================================
// spi_adc.v
// SPI master for MCP3008 10-bit 8-channel ADC on Tang Nano 20K.
//
// Reads 4 potentiometers (gain, bass, mid, treble) at ~1kHz update rate.
// Values are exposed as 10-bit registers, updated continuously.
//
// MCP3008 SPI Protocol (Mode 0: CPOL=0, CPHA=0):
//   - CS low
//   - Send: start(1), single-ended(1), D2, D1, D0 = 5 bits on MOSI
//   - Receive: null bit + 10-bit result on MISO
//   - CS high
//   Total: 17 SPI clock cycles per conversion
//
// SPI clock: 27MHz / 256 = ~105kHz (well within MCP3008 limits)
// 4 channels x 17 clocks / 105kHz = ~650us per full scan (~1.5kHz update)
//
// Pin mapping (directly to MCP3008):
//   spi_clk  = SPI SCLK
//   spi_mosi = SPI DIN
//   spi_miso = SPI DOUT
//   spi_cs_n = SPI CS (active low)
// ============================================================================

module spi_adc #(
    parameter NUM_CHANNELS = 4,
    parameter CLK_DIV      = 256    // 27MHz/256 = ~105kHz SPI
)(
    input  wire        clk,         // 27 MHz system clock
    input  wire        rst_n,

    // SPI pins (directly to MCP3008)
    output reg         spi_clk,
    output reg         spi_mosi,
    input  wire        spi_miso,
    output reg         spi_cs_n,

    // Channel values (updated continuously)
    output reg [9:0]   ch0_val,     // gain pot
    output reg [9:0]   ch1_val,     // bass pot
    output reg [9:0]   ch2_val,     // mid pot
    output reg [9:0]   ch3_val      // treble pot
);

    // -------------------------------------------------------------------------
    // State machine
    // -------------------------------------------------------------------------
    localparam S_IDLE    = 3'd0;
    localparam S_CS_LOW  = 3'd1;
    localparam S_SEND    = 3'd2;
    localparam S_RECV    = 3'd3;
    localparam S_CS_HIGH = 3'd4;

    reg [2:0]  state;
    reg [1:0]  ch_idx;              // Current channel (0..NUM_CHANNELS-1)
    reg [4:0]  bit_cnt;             // Bit counter within send/recv phases

    // SPI clock divider
    reg [7:0]  clk_div_cnt;
    wire       spi_tick = (clk_div_cnt == CLK_DIV/2 - 1);
    wire       spi_tock = (clk_div_cnt == CLK_DIV - 1);

    // Shift registers
    reg [4:0]  tx_shift;            // MOSI: start(1), single(1), D2, D1, D0
    reg [9:0]  rx_shift;            // MISO: 10-bit result

    // Scan timer: ~1ms at 27MHz = 27000 clocks
    // Using 15-bit counter, overflow at 27000
    reg [14:0] scan_timer;
    wire       scan_trigger = (scan_timer == 15'd26999);

    // -------------------------------------------------------------------------
    // Clock divider — free-running
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            clk_div_cnt <= 8'd0;
        else if (clk_div_cnt == CLK_DIV - 1)
            clk_div_cnt <= 8'd0;
        else
            clk_div_cnt <= clk_div_cnt + 8'd1;
    end

    // -------------------------------------------------------------------------
    // Scan timer — counts up to ~1ms, resets when a scan starts
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            scan_timer <= 15'd0;
        else if (state != S_IDLE)
            scan_timer <= 15'd0;
        else if (scan_trigger)
            scan_timer <= scan_timer;           // Hold at trigger value
        else
            scan_timer <= scan_timer + 15'd1;
    end

    // -------------------------------------------------------------------------
    // Build TX word for current channel: {start=1, single=1, D2, D1, D0}
    // MCP3008 channel select: D2=0, D1=ch[1], D0=ch[0] for channels 0-3
    // -------------------------------------------------------------------------
    wire [4:0] tx_data = {2'b11, 1'b0, ch_idx[1:0]};

    // -------------------------------------------------------------------------
    // Main state machine — transitions on SPI clock edges
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            ch_idx   <= 2'd0;
            bit_cnt  <= 5'd0;
            spi_clk  <= 1'b0;
            spi_mosi <= 1'b0;
            spi_cs_n <= 1'b1;
            tx_shift <= 5'd0;
            rx_shift <= 10'd0;
            ch0_val  <= 10'd0;
            ch1_val  <= 10'd0;
            ch2_val  <= 10'd0;
            ch3_val  <= 10'd0;
        end else begin
            case (state)
                // ---------------------------------------------------------
                S_IDLE: begin
                    spi_clk  <= 1'b0;
                    spi_cs_n <= 1'b1;
                    spi_mosi <= 1'b0;
                    if (scan_trigger) begin
                        ch_idx <= 2'd0;
                        state  <= S_CS_LOW;
                    end
                end

                // ---------------------------------------------------------
                S_CS_LOW: begin
                    spi_cs_n <= 1'b0;
                    tx_shift <= tx_data;
                    bit_cnt  <= 5'd0;
                    // Wait one full SPI period for CS setup time
                    if (spi_tock)
                        state <= S_SEND;
                end

                // ---------------------------------------------------------
                // Shift out 5 bits MSB-first on rising edge of SPI clock
                S_SEND: begin
                    if (spi_tick) begin
                        // Rising edge: drive MOSI, raise SPI clock
                        spi_clk  <= 1'b1;
                        spi_mosi <= tx_shift[4];
                    end
                    if (spi_tock) begin
                        // Falling edge: lower SPI clock, advance
                        spi_clk  <= 1'b0;
                        tx_shift <= {tx_shift[3:0], 1'b0};
                        bit_cnt  <= bit_cnt + 5'd1;
                        if (bit_cnt == 5'd4) begin
                            // Done sending 5 bits, move to receive
                            bit_cnt <= 5'd0;
                            state   <= S_RECV;
                        end
                    end
                end

                // ---------------------------------------------------------
                // Receive null bit + 10 data bits = 11 SPI clocks
                // Sample MISO on rising edge, data valid after null bit
                S_RECV: begin
                    if (spi_tick) begin
                        // Rising edge: sample MISO, raise SPI clock
                        spi_clk <= 1'b1;
                        if (bit_cnt > 5'd0) begin
                            // After null bit, shift in data
                            rx_shift <= {rx_shift[8:0], spi_miso};
                        end
                        // bit_cnt 0 = null bit (discard)
                        // bit_cnt 1..10 = D9..D0
                    end
                    if (spi_tock) begin
                        // Falling edge: lower SPI clock, advance
                        spi_clk <= 1'b0;
                        spi_mosi <= 1'b0;
                        bit_cnt <= bit_cnt + 5'd1;
                        if (bit_cnt == 5'd10) begin
                            // Done receiving 11 bits (null + 10 data)
                            state <= S_CS_HIGH;
                        end
                    end
                end

                // ---------------------------------------------------------
                S_CS_HIGH: begin
                    spi_cs_n <= 1'b1;
                    spi_clk  <= 1'b0;
                    // Store result
                    case (ch_idx)
                        2'd0: ch0_val <= rx_shift;
                        2'd1: ch1_val <= rx_shift;
                        2'd2: ch2_val <= rx_shift;
                        2'd3: ch3_val <= rx_shift;
                    endcase
                    // Advance channel or finish scan
                    if (ch_idx == NUM_CHANNELS - 1) begin
                        state <= S_IDLE;
                    end else begin
                        ch_idx <= ch_idx + 2'd1;
                        state  <= S_CS_LOW;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
