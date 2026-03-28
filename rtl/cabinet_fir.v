// ============================================================================
// cabinet_fir.v
// FIR convolution for speaker cabinet impulse response simulation.
//
// Architecture:
//   - Circular buffer of N_TAPS recent samples in registers
//   - FIR taps in Q1.15 (signed 16-bit) loaded from hex file
//   - MAC pipeline: 1 tap per clock cycle
//   - N_TAPS clocks per output sample
//
// With N_TAPS=129: 129 clocks per sample, fits in 562-cycle budget.
//
// Fixed point: Q16.16 signed throughout
// Multiply: Q16.16 × Q1.15 = Q17.31 → shift right 15 → Q16.16
// ============================================================================

module cabinet_fir #(
    parameter N_TAPS = 256,
    parameter HEX_FILE = "data/cab_ir.hex"
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sample_en,
    input  wire signed [31:0] audio_in,
    output reg  signed [31:0] audio_out,
    output reg         out_valid
);

// ============================================================================
// FIR Tap Coefficients (Q1.15 signed 16-bit, loaded from hex file)
// ============================================================================

reg signed [15:0] taps [0:N_TAPS-1];

initial begin
    $readmemh(HEX_FILE, taps);
end

// ============================================================================
// Circular Sample Buffer
// ============================================================================

reg signed [31:0] sample_buf [0:N_TAPS-1];
reg [$clog2(N_TAPS)-1:0] write_ptr;  // Points to oldest sample (next write position)

integer init_i;
initial begin
    for (init_i = 0; init_i < N_TAPS; init_i = init_i + 1)
        sample_buf[init_i] = 0;
end

// ============================================================================
// MAC State Machine
// ============================================================================

localparam S_IDLE = 2'd0;
localparam S_MAC  = 2'd1;
localparam S_OUT  = 2'd2;

reg [1:0] mac_state;
reg [$clog2(N_TAPS)-1:0] tap_idx;    // Which tap we're processing
reg [$clog2(N_TAPS)-1:0] buf_idx;    // Read pointer into circular buffer
reg signed [63:0] accumulator;
reg signed [63:0] mac_product;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        mac_state   <= S_IDLE;
        out_valid   <= 1'b0;
        audio_out   <= 0;
        write_ptr   <= 0;
        tap_idx     <= 0;
        buf_idx     <= 0;
        accumulator <= 0;
    end else begin
        out_valid <= 1'b0;

        case (mac_state)

        S_IDLE: begin
            if (sample_en) begin
                // Write new sample into circular buffer
                sample_buf[write_ptr] <= audio_in;

                // Start MAC from tap[0] × newest sample
                // Newest sample is at write_ptr (just written)
                // We convolve: out = sum(tap[k] * sample[n-k])
                // sample[n] is at write_ptr, sample[n-1] at write_ptr-1, etc.
                tap_idx     <= 0;
                buf_idx     <= write_ptr;
                accumulator <= 0;
                mac_state   <= S_MAC;
            end
        end

        S_MAC: begin
            // MAC: accumulator += taps[tap_idx] * sample_buf[buf_idx]
            mac_product = $signed(taps[tap_idx]) * $signed(sample_buf[buf_idx]);
            accumulator <= accumulator + mac_product;

            if (tap_idx == N_TAPS - 1) begin
                mac_state <= S_OUT;
            end else begin
                tap_idx <= tap_idx + 1;
                // Move buffer read pointer backwards (circular)
                buf_idx <= (buf_idx == 0) ? N_TAPS - 1 : buf_idx - 1;
            end
        end

        S_OUT: begin
            // Shift accumulator: Q16.16 × Q1.15 product is in Q17.31
            // Shift right by 15 to get Q16.16
            audio_out <= accumulator >>> 15;
            out_valid <= 1'b1;

            // Advance write pointer for next sample
            write_ptr <= (write_ptr == N_TAPS - 1) ? 0 : write_ptr + 1;

            mac_state <= S_IDLE;
        end

        default: mac_state <= S_IDLE;

        endcase
    end
end

endmodule
