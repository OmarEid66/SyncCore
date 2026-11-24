// Block 8 - NCO frequency control
// Builds the phase step for the NCO accumulator:
//   phase_inc = freq_word + vp ,  freq_word = integrator (or force_freq in DFT mode)
// freq_word is in "turns per symbol * 2^32", so fine_cfo_est / 2^32 is the residual
// frequency offset as a fraction of the symbol rate.
module nco_freq_ctrl (
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [31:0] integ,
    input  wire signed [31:0] vp,
    input  wire               in_valid,
    input  wire               force_en,
    input  wire signed [31:0] force_freq,
    output reg         [31:0] phase_inc,
    output reg                inc_valid,
    output reg  signed [31:0] fine_cfo_est
);
    wire signed [31:0] freq = force_en ? force_freq : integ;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase_inc <= 32'd0; inc_valid <= 1'b0; fine_cfo_est <= 32'sd0;
        end else begin
            inc_valid <= in_valid;
            if (in_valid) begin
                phase_inc    <= freq + vp;      // wraps modulo 2^32 on purpose
                fine_cfo_est <= freq;
            end
        end
    end
endmodule
