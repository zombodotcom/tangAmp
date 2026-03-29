// ============================================================================
// sd_ir_loader.v
// Orchestrates loading cabinet IR data from SD card into cabinet_fir taps.
//
// Boot-time flow:
//   1. Wait for sd_spi_reader to finish SD card initialization
//   2. Read sector 0 (header) — validate "TAIR" magic, extract num_irs
//      and default_ir index
//   3. Read sector (default_ir + 1) — sd_spi_reader writes taps directly
//      to cabinet_fir via tap_wr interface
//   4. Go idle — IR is loaded, amp is ready
//
// The sd_spi_reader already assembles 16-bit little-endian tap values from
// the raw byte stream and outputs tap_wr_en/addr/data. During header reads,
// this module captures the raw bytes via byte_out/byte_out_valid from
// sd_spi_reader, and gates tap_wr_en so header data doesn't corrupt taps.
//
// During IR sector reads, tap_wr_en flows through to cabinet_fir directly.
// ============================================================================

module sd_ir_loader (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        btn_next,       // pulse: load next IR (active low, debounced externally)

    // SD SPI reader control interface
    output reg         sd_start_read,
    output reg  [31:0] sd_sector_addr,
    input  wire        sd_init_done,
    input  wire        sd_read_done,
    input  wire        sd_error,

    // Byte-level output from sd_spi_reader (for header parsing)
    input  wire        sd_byte_valid,
    input  wire [7:0]  sd_byte_data,
    input  wire [9:0]  sd_byte_index,  // 0-511 within sector

    // Tap write gating: when ir_sector_active=1, let tap_wr_en through
    output reg         ir_sector_active,

    // Status
    output reg  [3:0]  current_ir,     // which IR is loaded (0-15)
    output reg  [3:0]  num_irs,        // total IRs on card
    output reg         ir_loaded,      // an IR has been successfully loaded
    output reg         loading,        // currently loading an IR
    output reg         header_ok       // header was valid
);

// ============================================================================
// State Machine
// ============================================================================

localparam S_WAIT_INIT   = 3'd0;
localparam S_READ_HEADER = 3'd1;
localparam S_PARSE_HDR   = 3'd2;
localparam S_LOAD_IR     = 3'd3;
localparam S_WAIT_IR     = 3'd4;
localparam S_IDLE        = 3'd5;
localparam S_ERROR       = 3'd6;

reg [2:0] state;

// Header capture registers
reg [7:0] hdr_magic [0:3];       // "TAIR"
reg [7:0] hdr_num_irs_lo;
reg [7:0] hdr_num_irs_hi;
reg [7:0] hdr_default_lo;
reg [7:0] hdr_default_hi;

// Button edge detection
reg btn_prev;
wire btn_pulse = btn_prev & ~btn_next;  // falling edge (active low button)

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state           <= S_WAIT_INIT;
        sd_start_read   <= 1'b0;
        sd_sector_addr  <= 32'd0;
        ir_sector_active <= 1'b0;
        current_ir      <= 4'd0;
        num_irs         <= 4'd0;
        ir_loaded       <= 1'b0;
        loading         <= 1'b0;
        header_ok       <= 1'b0;
        btn_prev        <= 1'b1;
        hdr_magic[0]    <= 8'd0;
        hdr_magic[1]    <= 8'd0;
        hdr_magic[2]    <= 8'd0;
        hdr_magic[3]    <= 8'd0;
        hdr_num_irs_lo  <= 8'd0;
        hdr_num_irs_hi  <= 8'd0;
        hdr_default_lo  <= 8'd0;
        hdr_default_hi  <= 8'd0;
    end else begin
        sd_start_read <= 1'b0;
        btn_prev      <= btn_next;

        case (state)

        // ── Wait for SD card initialization ──────────────────────────
        S_WAIT_INIT: begin
            loading <= 1'b1;
            if (sd_error) begin
                // SD init failed — keep using $readmemh defaults
                loading <= 1'b0;
                state   <= S_ERROR;
            end else if (sd_init_done) begin
                // Request header sector
                sd_sector_addr  <= 32'd0;
                sd_start_read   <= 1'b1;
                ir_sector_active <= 1'b0;  // don't write header bytes as taps
                state           <= S_READ_HEADER;
            end
        end

        // ── Reading header sector 0 ──────────────────────────────────
        S_READ_HEADER: begin
            // Capture header bytes as they arrive
            if (sd_byte_valid) begin
                case (sd_byte_index)
                    10'd0: hdr_magic[0]   <= sd_byte_data;
                    10'd1: hdr_magic[1]   <= sd_byte_data;
                    10'd2: hdr_magic[2]   <= sd_byte_data;
                    10'd3: hdr_magic[3]   <= sd_byte_data;
                    // bytes 4-5: version (ignored)
                    10'd6: hdr_num_irs_lo <= sd_byte_data;
                    10'd7: hdr_num_irs_hi <= sd_byte_data;
                    // bytes 8-11: taps_per_ir, bits_per_tap (known: 256, 16)
                    10'd12: hdr_default_lo <= sd_byte_data;
                    10'd13: hdr_default_hi <= sd_byte_data;
                    default: ;  // ignore rest
                endcase
            end

            if (sd_read_done)
                state <= S_PARSE_HDR;

            if (sd_error)
                state <= S_ERROR;
        end

        // ── Validate header and extract fields ───────────────────────
        S_PARSE_HDR: begin
            // Check magic "TAIR" = 0x54 0x41 0x49 0x52
            if (hdr_magic[0] == 8'h54 && hdr_magic[1] == 8'h41 &&
                hdr_magic[2] == 8'h49 && hdr_magic[3] == 8'h52) begin
                header_ok  <= 1'b1;
                // Clamp num_irs to 1-15
                if (hdr_num_irs_lo == 8'd0 && hdr_num_irs_hi == 8'd0) begin
                    state <= S_ERROR;  // no IRs
                end else begin
                    num_irs    <= (hdr_num_irs_lo > 4'd15) ? 4'd15 : hdr_num_irs_lo[3:0];
                    // Clamp default IR to valid range
                    current_ir <= (hdr_default_lo[3:0] >= hdr_num_irs_lo[3:0])
                                  ? 4'd0 : hdr_default_lo[3:0];
                    state      <= S_LOAD_IR;
                end
            end else begin
                // Invalid magic — keep $readmemh defaults
                state <= S_ERROR;
            end
        end

        // ── Start reading IR sector ──────────────────────────────────
        S_LOAD_IR: begin
            sd_sector_addr   <= {28'd0, current_ir} + 32'd1;  // sector = ir_index + 1
            sd_start_read    <= 1'b1;
            ir_sector_active <= 1'b1;  // allow tap writes through
            state            <= S_WAIT_IR;
        end

        // ── Wait for IR sector read to complete ──────────────────────
        S_WAIT_IR: begin
            if (sd_read_done) begin
                ir_sector_active <= 1'b0;
                ir_loaded        <= 1'b1;
                loading          <= 1'b0;
                state            <= S_IDLE;
            end
            if (sd_error) begin
                ir_sector_active <= 1'b0;
                loading          <= 1'b0;
                state            <= S_ERROR;
            end
        end

        // ── Idle: wait for button press to cycle IR ──────────────────
        S_IDLE: begin
            if (btn_pulse) begin
                loading <= 1'b1;
                // Cycle to next IR (wrap around)
                if (current_ir == num_irs - 1)
                    current_ir <= 4'd0;
                else
                    current_ir <= current_ir + 4'd1;
                state <= S_LOAD_IR;
            end
        end

        // ── Error: SD card failed, use built-in IR ───────────────────
        S_ERROR: begin
            loading          <= 1'b0;
            ir_sector_active <= 1'b0;
            // Stay here until reset
        end

        default: state <= S_WAIT_INIT;

        endcase
    end
end

endmodule
