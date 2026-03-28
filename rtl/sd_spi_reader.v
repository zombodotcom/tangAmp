// ============================================================================
// sd_spi_reader.v
// Minimal SPI master for reading SD card sectors on Tang Nano 20K.
//
// Implements:
//   - SD card SPI mode initialization (CMD0, CMD8, CMD55+ACMD41, CMD58)
//   - Single 512-byte sector read (CMD17)
//   - Direct write of cabinet IR tap data into cabinet_fir tap RAM
//
// Designed for loading cabinet IRs: 256 taps x 16-bit = 512 bytes = 1 sector.
//
// Pin mapping (Tang Nano 20K onboard MicroSD slot):
//   sd_clk  = pin 83 (SPI SCLK)
//   sd_cmd  = pin 82 (SPI MOSI)
//   sd_dat0 = pin 84 (SPI MISO)
//   sd_dat3 = pin 81 (SPI CS, active low)
//
// Clock: 27 MHz system clock
// SPI clock: ~396 kHz during init (27M/68), ~6.75 MHz after init (27M/4)
// ============================================================================

module sd_spi_reader (
    input  wire        clk,          // 27 MHz system clock
    input  wire        rst_n,
    input  wire        start_read,   // Pulse to begin reading a sector
    input  wire [31:0] sector_addr,  // Sector number to read

    // SPI pins (directly to SD card)
    output reg         sd_clk_o,     // SPI clock
    output reg         sd_mosi,      // SPI MOSI (directly drives sd_cmd pin)
    input  wire        sd_miso,      // SPI MISO (from sd_dat0 pin)
    output reg         sd_cs_n,      // SPI CS (directly drives sd_dat3 pin)

    // Cabinet IR tap write interface (directly to cabinet_fir)
    output reg         tap_wr_en,
    output reg  [7:0]  tap_wr_addr,
    output reg  [15:0] tap_wr_data,

    // Status
    output reg         init_done,    // SD card initialized successfully
    output reg         read_done,    // Sector read complete
    output reg         error         // Initialization or read failed
);

// ============================================================================
// SPI Clock Divider
// ============================================================================
// Init: 27MHz / 68 = ~397kHz (must be <= 400kHz)
// Fast: 27MHz /  4 = 6.75MHz (well within SD spec 25MHz limit)

localparam SLOW_DIV = 7'd34;   // half-period: 27M / (2*34) = ~397kHz
localparam FAST_DIV = 7'd2;    // half-period: 27M / (2*2)  = 6.75MHz

reg [6:0] clk_div;
reg [6:0] clk_div_target;
reg       spi_clk_en;          // Pulse when SPI clock should toggle
reg       spi_clk_phase;       // 0 = rising edge next, 1 = falling edge next

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        clk_div       <= 0;
        spi_clk_en    <= 0;
        spi_clk_phase <= 0;
    end else begin
        spi_clk_en <= 0;
        if (clk_div == clk_div_target - 1) begin
            clk_div    <= 0;
            spi_clk_en <= 1;
            spi_clk_phase <= ~spi_clk_phase;
        end else begin
            clk_div <= clk_div + 1;
        end
    end
end

// ============================================================================
// SPI Shift Register
// ============================================================================

reg  [7:0]  spi_tx_byte;       // Byte to transmit
reg  [7:0]  spi_rx_byte;       // Received byte
reg  [2:0]  spi_bit_cnt;       // Bit counter (7 downto 0)
reg         spi_busy;
reg         spi_byte_done;     // Pulses when byte transfer complete
reg         spi_start;         // Pulse to begin a byte transfer

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sd_clk_o      <= 0;
        sd_mosi       <= 1;
        spi_rx_byte   <= 8'hFF;
        spi_bit_cnt   <= 0;
        spi_busy      <= 0;
        spi_byte_done <= 0;
    end else begin
        spi_byte_done <= 0;

        if (spi_start && !spi_busy) begin
            spi_busy    <= 1;
            spi_bit_cnt <= 3'd7;
            sd_mosi     <= spi_tx_byte[7];  // MSB first
        end else if (spi_busy && spi_clk_en) begin
            if (!spi_clk_phase) begin
                // Rising edge: sample MISO
                sd_clk_o <= 1;
                spi_rx_byte <= {spi_rx_byte[6:0], sd_miso};
            end else begin
                // Falling edge: shift out next MOSI bit
                sd_clk_o <= 0;
                if (spi_bit_cnt == 0) begin
                    spi_busy      <= 0;
                    spi_byte_done <= 1;
                    sd_mosi       <= 1;  // Idle high
                end else begin
                    spi_bit_cnt <= spi_bit_cnt - 1;
                    sd_mosi     <= spi_tx_byte[spi_bit_cnt - 1];
                end
            end
        end
    end
end

// ============================================================================
// SD Command Sender
// ============================================================================
// 6-byte command format: [0x40|cmd_idx, arg[31:24], arg[23:16],
//                         arg[15:8], arg[7:0], crc|0x01]

reg [47:0] cmd_buf;            // 6-byte command buffer
reg [2:0]  cmd_byte_idx;       // Which byte of the 6 we're sending
reg        cmd_sending;
reg        cmd_send_start;

// Response waiting
reg [7:0]  cmd_response;       // R1 response byte
reg        cmd_resp_valid;
reg [3:0]  resp_wait_cnt;      // Wait up to 8 bytes for response

// ============================================================================
// Main State Machine
// ============================================================================

localparam S_RESET       = 5'd0;
localparam S_POWERUP     = 5'd1;   // Wait >= 1ms after power
localparam S_SEND_CLOCKS = 5'd2;   // 80 clocks with CS high
localparam S_CMD0        = 5'd3;   // GO_IDLE_STATE
localparam S_CMD0_RESP   = 5'd4;
localparam S_CMD8        = 5'd5;   // SEND_IF_COND
localparam S_CMD8_RESP   = 5'd6;
localparam S_CMD8_TAIL   = 5'd7;   // Read 4 extra bytes of R7 response
localparam S_CMD55       = 5'd8;   // APP_CMD prefix
localparam S_CMD55_RESP  = 5'd9;
localparam S_ACMD41      = 5'd10;  // SD_SEND_OP_COND
localparam S_ACMD41_RESP = 5'd11;
localparam S_CMD58       = 5'd12;  // READ_OCR (check SDHC)
localparam S_CMD58_RESP  = 5'd13;
localparam S_CMD58_TAIL  = 5'd14;  // Read 4 OCR bytes
localparam S_INIT_DONE   = 5'd15;
localparam S_IDLE        = 5'd16;  // Waiting for read request
localparam S_CMD17       = 5'd17;  // READ_SINGLE_BLOCK
localparam S_CMD17_RESP  = 5'd18;
localparam S_WAIT_TOKEN  = 5'd19;  // Wait for data token 0xFE
localparam S_READ_DATA   = 5'd20;  // Read 512 bytes
localparam S_READ_CRC    = 5'd21;  // Read 2 CRC bytes (discard)
localparam S_READ_DONE   = 5'd22;
localparam S_ERROR       = 5'd23;

reg [4:0]  state;
reg [15:0] wait_cnt;           // General-purpose wait counter
reg [9:0]  byte_cnt;           // Byte counter for data read (0-511)
reg [9:0]  retry_cnt;          // Retry counter for ACMD41
reg        sdhc_mode;          // 1 = SDHC (sector addressing)
reg [3:0]  dummy_cnt;          // Counter for extra response bytes
reg [7:0]  data_lo;            // Low byte of 16-bit tap value

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state          <= S_RESET;
        sd_cs_n        <= 1'b1;
        clk_div_target <= SLOW_DIV;
        init_done      <= 1'b0;
        read_done      <= 1'b0;
        error          <= 1'b0;
        spi_start      <= 1'b0;
        spi_tx_byte    <= 8'hFF;
        tap_wr_en      <= 1'b0;
        tap_wr_addr    <= 8'd0;
        tap_wr_data    <= 16'd0;
        wait_cnt       <= 16'd0;
        byte_cnt       <= 10'd0;
        retry_cnt      <= 10'd0;
        sdhc_mode      <= 1'b0;
        dummy_cnt      <= 4'd0;
        data_lo        <= 8'd0;
    end else begin
        spi_start  <= 1'b0;
        tap_wr_en  <= 1'b0;
        read_done  <= 1'b0;

        case (state)

        // ── Power-up delay (~1ms = 27000 clocks) ──────────────────────
        S_RESET: begin
            sd_cs_n        <= 1'b1;
            clk_div_target <= SLOW_DIV;
            wait_cnt       <= 16'd0;
            state          <= S_POWERUP;
        end

        S_POWERUP: begin
            if (wait_cnt == 16'd27000) begin
                wait_cnt <= 16'd0;
                state    <= S_SEND_CLOCKS;
            end else begin
                wait_cnt <= wait_cnt + 1;
            end
        end

        // ── 80 clocks with CS high, MOSI high ────────────────────────
        S_SEND_CLOCKS: begin
            sd_cs_n <= 1'b1;  // CS high
            if (!spi_busy && !spi_byte_done) begin
                if (wait_cnt < 16'd10) begin
                    spi_tx_byte <= 8'hFF;
                    spi_start   <= 1'b1;
                    wait_cnt    <= wait_cnt + 1;
                end else begin
                    wait_cnt <= 16'd0;
                    state    <= S_CMD0;
                end
            end
        end

        // ── CMD0: GO_IDLE_STATE ───────────────────────────────────────
        S_CMD0: begin
            sd_cs_n <= 1'b0;
            if (!spi_busy) begin
                case (wait_cnt[2:0])
                    3'd0: begin spi_tx_byte <= 8'h40; spi_start <= 1; end  // CMD0
                    3'd1: begin spi_tx_byte <= 8'h00; spi_start <= 1; end  // arg[31:24]
                    3'd2: begin spi_tx_byte <= 8'h00; spi_start <= 1; end  // arg[23:16]
                    3'd3: begin spi_tx_byte <= 8'h00; spi_start <= 1; end  // arg[15:8]
                    3'd4: begin spi_tx_byte <= 8'h00; spi_start <= 1; end  // arg[7:0]
                    3'd5: begin spi_tx_byte <= 8'h95; spi_start <= 1; end  // CRC
                    default: ;
                endcase
                if (spi_byte_done)
                    wait_cnt <= wait_cnt + 1;
                if (wait_cnt == 16'd6) begin
                    wait_cnt      <= 16'd0;
                    resp_wait_cnt <= 4'd0;
                    state         <= S_CMD0_RESP;
                end
            end
        end

        // ── Wait for R1 response ──────────────────────────────────────
        S_CMD0_RESP: begin
            if (!spi_busy && !spi_start) begin
                spi_tx_byte <= 8'hFF;
                spi_start   <= 1'b1;
            end
            if (spi_byte_done) begin
                if (spi_rx_byte[7] == 1'b0) begin
                    // Got valid R1 response
                    cmd_response <= spi_rx_byte;
                    sd_cs_n      <= 1'b1;
                    if (spi_rx_byte == 8'h01) begin
                        // Card is in idle state — proceed to CMD8
                        wait_cnt <= 16'd0;
                        state    <= S_CMD8;
                    end else begin
                        state <= S_ERROR;
                    end
                end else begin
                    resp_wait_cnt <= resp_wait_cnt + 1;
                    if (resp_wait_cnt == 4'd15)
                        state <= S_ERROR;  // No response
                end
            end
        end

        // ── CMD8: SEND_IF_COND (0x01AA) ──────────────────────────────
        S_CMD8: begin
            sd_cs_n <= 1'b0;
            if (!spi_busy) begin
                case (wait_cnt[2:0])
                    3'd0: begin spi_tx_byte <= 8'h48; spi_start <= 1; end  // CMD8
                    3'd1: begin spi_tx_byte <= 8'h00; spi_start <= 1; end
                    3'd2: begin spi_tx_byte <= 8'h00; spi_start <= 1; end
                    3'd3: begin spi_tx_byte <= 8'h01; spi_start <= 1; end  // 2.7-3.6V
                    3'd4: begin spi_tx_byte <= 8'hAA; spi_start <= 1; end  // Check pattern
                    3'd5: begin spi_tx_byte <= 8'h87; spi_start <= 1; end  // CRC
                    default: ;
                endcase
                if (spi_byte_done)
                    wait_cnt <= wait_cnt + 1;
                if (wait_cnt == 16'd6) begin
                    wait_cnt      <= 16'd0;
                    resp_wait_cnt <= 4'd0;
                    state         <= S_CMD8_RESP;
                end
            end
        end

        S_CMD8_RESP: begin
            if (!spi_busy && !spi_start) begin
                spi_tx_byte <= 8'hFF;
                spi_start   <= 1'b1;
            end
            if (spi_byte_done) begin
                if (spi_rx_byte[7] == 1'b0) begin
                    cmd_response <= spi_rx_byte;
                    // R7 has 4 more bytes — read and discard
                    dummy_cnt <= 4'd0;
                    state     <= S_CMD8_TAIL;
                end else begin
                    resp_wait_cnt <= resp_wait_cnt + 1;
                    if (resp_wait_cnt == 4'd15)
                        state <= S_ERROR;
                end
            end
        end

        S_CMD8_TAIL: begin
            if (!spi_busy && !spi_start) begin
                spi_tx_byte <= 8'hFF;
                spi_start   <= 1'b1;
            end
            if (spi_byte_done) begin
                if (dummy_cnt == 4'd3) begin
                    sd_cs_n    <= 1'b1;
                    wait_cnt   <= 16'd0;
                    retry_cnt  <= 10'd0;
                    state      <= S_CMD55;
                end else begin
                    dummy_cnt <= dummy_cnt + 1;
                end
            end
        end

        // ── CMD55: APP_CMD prefix ─────────────────────────────────────
        S_CMD55: begin
            sd_cs_n <= 1'b0;
            if (!spi_busy) begin
                case (wait_cnt[2:0])
                    3'd0: begin spi_tx_byte <= 8'h77; spi_start <= 1; end  // CMD55
                    3'd1: begin spi_tx_byte <= 8'h00; spi_start <= 1; end
                    3'd2: begin spi_tx_byte <= 8'h00; spi_start <= 1; end
                    3'd3: begin spi_tx_byte <= 8'h00; spi_start <= 1; end
                    3'd4: begin spi_tx_byte <= 8'h00; spi_start <= 1; end
                    3'd5: begin spi_tx_byte <= 8'h65; spi_start <= 1; end  // CRC
                    default: ;
                endcase
                if (spi_byte_done)
                    wait_cnt <= wait_cnt + 1;
                if (wait_cnt == 16'd6) begin
                    wait_cnt      <= 16'd0;
                    resp_wait_cnt <= 4'd0;
                    state         <= S_CMD55_RESP;
                end
            end
        end

        S_CMD55_RESP: begin
            if (!spi_busy && !spi_start) begin
                spi_tx_byte <= 8'hFF;
                spi_start   <= 1'b1;
            end
            if (spi_byte_done) begin
                if (spi_rx_byte[7] == 1'b0) begin
                    sd_cs_n  <= 1'b1;
                    wait_cnt <= 16'd0;
                    state    <= S_ACMD41;
                end else begin
                    resp_wait_cnt <= resp_wait_cnt + 1;
                    if (resp_wait_cnt == 4'd15)
                        state <= S_ERROR;
                end
            end
        end

        // ── ACMD41: SD_SEND_OP_COND (HCS=1) ──────────────────────────
        S_ACMD41: begin
            sd_cs_n <= 1'b0;
            if (!spi_busy) begin
                case (wait_cnt[2:0])
                    3'd0: begin spi_tx_byte <= 8'h69; spi_start <= 1; end  // ACMD41
                    3'd1: begin spi_tx_byte <= 8'h40; spi_start <= 1; end  // HCS=1
                    3'd2: begin spi_tx_byte <= 8'h00; spi_start <= 1; end
                    3'd3: begin spi_tx_byte <= 8'h00; spi_start <= 1; end
                    3'd4: begin spi_tx_byte <= 8'h00; spi_start <= 1; end
                    3'd5: begin spi_tx_byte <= 8'h77; spi_start <= 1; end  // CRC (dummy OK)
                    default: ;
                endcase
                if (spi_byte_done)
                    wait_cnt <= wait_cnt + 1;
                if (wait_cnt == 16'd6) begin
                    wait_cnt      <= 16'd0;
                    resp_wait_cnt <= 4'd0;
                    state         <= S_ACMD41_RESP;
                end
            end
        end

        S_ACMD41_RESP: begin
            if (!spi_busy && !spi_start) begin
                spi_tx_byte <= 8'hFF;
                spi_start   <= 1'b1;
            end
            if (spi_byte_done) begin
                if (spi_rx_byte[7] == 1'b0) begin
                    sd_cs_n <= 1'b1;
                    if (spi_rx_byte == 8'h00) begin
                        // Card initialized — check SDHC
                        wait_cnt <= 16'd0;
                        state    <= S_CMD58;
                    end else if (spi_rx_byte == 8'h01) begin
                        // Still initializing — retry
                        retry_cnt <= retry_cnt + 1;
                        if (retry_cnt == 10'd1000)
                            state <= S_ERROR;  // Timeout
                        else begin
                            wait_cnt <= 16'd0;
                            state    <= S_CMD55;  // Retry CMD55+ACMD41
                        end
                    end else begin
                        state <= S_ERROR;
                    end
                end else begin
                    resp_wait_cnt <= resp_wait_cnt + 1;
                    if (resp_wait_cnt == 4'd15)
                        state <= S_ERROR;
                end
            end
        end

        // ── CMD58: READ_OCR (check CCS for SDHC) ─────────────────────
        S_CMD58: begin
            sd_cs_n <= 1'b0;
            if (!spi_busy) begin
                case (wait_cnt[2:0])
                    3'd0: begin spi_tx_byte <= 8'h7A; spi_start <= 1; end  // CMD58
                    3'd1: begin spi_tx_byte <= 8'h00; spi_start <= 1; end
                    3'd2: begin spi_tx_byte <= 8'h00; spi_start <= 1; end
                    3'd3: begin spi_tx_byte <= 8'h00; spi_start <= 1; end
                    3'd4: begin spi_tx_byte <= 8'h00; spi_start <= 1; end
                    3'd5: begin spi_tx_byte <= 8'hFD; spi_start <= 1; end  // CRC (dummy)
                    default: ;
                endcase
                if (spi_byte_done)
                    wait_cnt <= wait_cnt + 1;
                if (wait_cnt == 16'd6) begin
                    wait_cnt      <= 16'd0;
                    resp_wait_cnt <= 4'd0;
                    state         <= S_CMD58_RESP;
                end
            end
        end

        S_CMD58_RESP: begin
            if (!spi_busy && !spi_start) begin
                spi_tx_byte <= 8'hFF;
                spi_start   <= 1'b1;
            end
            if (spi_byte_done) begin
                if (spi_rx_byte[7] == 1'b0) begin
                    cmd_response <= spi_rx_byte;
                    dummy_cnt    <= 4'd0;
                    state        <= S_CMD58_TAIL;
                end else begin
                    resp_wait_cnt <= resp_wait_cnt + 1;
                    if (resp_wait_cnt == 4'd15)
                        state <= S_ERROR;
                end
            end
        end

        S_CMD58_TAIL: begin
            if (!spi_busy && !spi_start) begin
                spi_tx_byte <= 8'hFF;
                spi_start   <= 1'b1;
            end
            if (spi_byte_done) begin
                // First byte of OCR has CCS in bit 6
                if (dummy_cnt == 4'd0)
                    sdhc_mode <= spi_rx_byte[6];  // CCS bit
                if (dummy_cnt == 4'd3) begin
                    sd_cs_n   <= 1'b1;
                    state     <= S_INIT_DONE;
                end else begin
                    dummy_cnt <= dummy_cnt + 1;
                end
            end
        end

        // ── Initialization complete ───────────────────────────────────
        S_INIT_DONE: begin
            init_done      <= 1'b1;
            clk_div_target <= FAST_DIV;  // Switch to fast SPI clock
            state          <= S_IDLE;
        end

        // ── Idle: wait for read request ───────────────────────────────
        S_IDLE: begin
            if (start_read) begin
                wait_cnt <= 16'd0;
                state    <= S_CMD17;
            end
        end

        // ── CMD17: READ_SINGLE_BLOCK ──────────────────────────────────
        S_CMD17: begin
            sd_cs_n <= 1'b0;
            if (!spi_busy) begin
                case (wait_cnt[2:0])
                    3'd0: begin spi_tx_byte <= 8'h51; spi_start <= 1; end  // CMD17
                    // Address: SDHC uses sector number, standard uses byte address
                    3'd1: begin
                        spi_tx_byte <= sdhc_mode ? sector_addr[31:24] : sector_addr[22:15];
                        spi_start   <= 1;
                    end
                    3'd2: begin
                        spi_tx_byte <= sdhc_mode ? sector_addr[23:16] : sector_addr[14:7];
                        spi_start   <= 1;
                    end
                    3'd3: begin
                        spi_tx_byte <= sdhc_mode ? sector_addr[15:8] : {sector_addr[6:0], 1'b0};
                        spi_start   <= 1;
                    end
                    3'd4: begin
                        spi_tx_byte <= sdhc_mode ? sector_addr[7:0] : 8'h00;
                        spi_start   <= 1;
                    end
                    3'd5: begin spi_tx_byte <= 8'hFF; spi_start <= 1; end  // CRC (dummy)
                    default: ;
                endcase
                if (spi_byte_done)
                    wait_cnt <= wait_cnt + 1;
                if (wait_cnt == 16'd6) begin
                    wait_cnt      <= 16'd0;
                    resp_wait_cnt <= 4'd0;
                    state         <= S_CMD17_RESP;
                end
            end
        end

        S_CMD17_RESP: begin
            if (!spi_busy && !spi_start) begin
                spi_tx_byte <= 8'hFF;
                spi_start   <= 1'b1;
            end
            if (spi_byte_done) begin
                if (spi_rx_byte[7] == 1'b0) begin
                    if (spi_rx_byte == 8'h00) begin
                        // Command accepted — wait for data token
                        wait_cnt <= 16'd0;
                        state    <= S_WAIT_TOKEN;
                    end else begin
                        sd_cs_n <= 1'b1;
                        state   <= S_ERROR;
                    end
                end else begin
                    resp_wait_cnt <= resp_wait_cnt + 1;
                    if (resp_wait_cnt == 4'd15) begin
                        sd_cs_n <= 1'b1;
                        state   <= S_ERROR;
                    end
                end
            end
        end

        // ── Wait for data token 0xFE ──────────────────────────────────
        S_WAIT_TOKEN: begin
            if (!spi_busy && !spi_start) begin
                spi_tx_byte <= 8'hFF;
                spi_start   <= 1'b1;
            end
            if (spi_byte_done) begin
                if (spi_rx_byte == 8'hFE) begin
                    // Data token received — start reading 512 bytes
                    byte_cnt    <= 10'd0;
                    tap_wr_addr <= 8'd0;
                    state       <= S_READ_DATA;
                end else begin
                    wait_cnt <= wait_cnt + 1;
                    if (wait_cnt == 16'd10000) begin
                        // Timeout waiting for data token
                        sd_cs_n <= 1'b1;
                        state   <= S_ERROR;
                    end
                end
            end
        end

        // ── Read 512 data bytes ───────────────────────────────────────
        // Each cabinet IR tap is 16-bit little-endian: [lo, hi]
        // Byte 0,1 = tap 0; byte 2,3 = tap 1; ... byte 510,511 = tap 255
        S_READ_DATA: begin
            if (!spi_busy && !spi_start) begin
                spi_tx_byte <= 8'hFF;
                spi_start   <= 1'b1;
            end
            if (spi_byte_done) begin
                if (byte_cnt[0] == 1'b0) begin
                    // Even byte = low byte of 16-bit tap
                    data_lo <= spi_rx_byte;
                end else begin
                    // Odd byte = high byte — write complete tap
                    tap_wr_data <= {spi_rx_byte, data_lo};  // {hi, lo}
                    tap_wr_addr <= byte_cnt[8:1];            // tap index
                    tap_wr_en   <= 1'b1;
                end

                if (byte_cnt == 10'd511) begin
                    byte_cnt <= 10'd0;
                    state    <= S_READ_CRC;
                end else begin
                    byte_cnt <= byte_cnt + 1;
                end
            end
        end

        // ── Read and discard 2 CRC bytes ──────────────────────────────
        S_READ_CRC: begin
            if (!spi_busy && !spi_start) begin
                spi_tx_byte <= 8'hFF;
                spi_start   <= 1'b1;
            end
            if (spi_byte_done) begin
                if (byte_cnt == 10'd1) begin
                    sd_cs_n <= 1'b1;
                    state   <= S_READ_DONE;
                end else begin
                    byte_cnt <= byte_cnt + 1;
                end
            end
        end

        // ── Read complete ─────────────────────────────────────────────
        S_READ_DONE: begin
            read_done <= 1'b1;
            state     <= S_IDLE;
        end

        // ── Error state ───────────────────────────────────────────────
        S_ERROR: begin
            error   <= 1'b1;
            sd_cs_n <= 1'b1;
            // Stay in error state until reset
        end

        default: state <= S_RESET;

        endcase
    end
end

endmodule
