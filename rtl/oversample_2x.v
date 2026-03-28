// ============================================================================
// oversample_2x.v
// 2x oversampling module for aliasing reduction in nonlinear processing.
//
// The tube nonlinearity (Koren equation) generates harmonics that fold back
// below Nyquist as aliasing at 48kHz. By upsampling 2x before the triode
// and downsampling after, aliasing energy is pushed above the audible range.
//
// Upsampling: Linear interpolation (sample, then midpoint of sample and prev).
//   - On sample_en_48k: outputs original sample as first 96kHz sample
//   - Then outputs interpolated midpoint as second 96kHz sample
//
// Downsampling: Average of two processed samples (simple lowpass + decimate).
//
// Clock budget: ~4 clocks for up, ~2 clocks for down.
// Fixed point: Q16.16 signed throughout.
// ============================================================================

module oversample_2x (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sample_en_48k,              // 48kHz input trigger
    input  wire signed [31:0] audio_in,            // 48kHz input sample

    // Upsampled output (two pulses per 48kHz period)
    output reg         up_valid_a,                  // first 96kHz sample ready
    output reg         up_valid_b,                  // second 96kHz sample ready
    output reg  signed [31:0] audio_up,             // upsampled output

    // Downsampling interface
    input  wire signed [31:0] processed_a,          // first processed 96kHz sample
    input  wire        proc_valid_a,                // first sample valid
    input  wire signed [31:0] processed_b,          // second processed 96kHz sample
    input  wire        proc_valid_b,                // second sample valid

    // 48kHz decimated output
    output reg  signed [31:0] audio_down,
    output reg         down_valid
);

// ============================================================================
// Upsampling: Linear interpolation
// ============================================================================

// Store previous sample for interpolation
reg signed [31:0] prev_sample;

// State machine for upsampling
localparam UP_IDLE = 2'd0;
localparam UP_A    = 2'd1;   // Output original sample
localparam UP_B    = 2'd2;   // Output interpolated sample

reg [1:0] up_state;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        up_state    <= UP_IDLE;
        up_valid_a  <= 1'b0;
        up_valid_b  <= 1'b0;
        audio_up    <= 32'sd0;
        prev_sample <= 32'sd0;
    end else begin
        up_valid_a <= 1'b0;
        up_valid_b <= 1'b0;

        case (up_state)

        UP_IDLE: begin
            if (sample_en_48k) begin
                // First 96kHz sample: the original input sample
                audio_up   <= audio_in;
                up_valid_a <= 1'b1;
                up_state   <= UP_B;
            end
        end

        UP_B: begin
            // Second 96kHz sample: midpoint between previous and current
            // (prev + current) / 2 = linear interpolation at the midpoint
            audio_up   <= (prev_sample + audio_in) >>> 1;
            up_valid_b <= 1'b1;
            prev_sample <= audio_in;
            up_state   <= UP_IDLE;
        end

        default: up_state <= UP_IDLE;

        endcase
    end
end

// ============================================================================
// Downsampling: Average of two processed samples
// ============================================================================

// Capture first processed sample, then average with second
reg signed [31:0] proc_a_held;
reg               have_a;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        proc_a_held <= 32'sd0;
        have_a      <= 1'b0;
        audio_down  <= 32'sd0;
        down_valid  <= 1'b0;
    end else begin
        down_valid <= 1'b0;

        if (proc_valid_a) begin
            proc_a_held <= processed_a;
            have_a      <= 1'b1;
        end

        if (proc_valid_b && have_a) begin
            // Average the two samples: simple lowpass before decimation
            audio_down <= (proc_a_held + processed_b) >>> 1;
            down_valid <= 1'b1;
            have_a     <= 1'b0;
        end
    end
end

endmodule
