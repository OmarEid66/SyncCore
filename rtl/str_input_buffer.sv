//==============================================================================
// Module: str_input_buffer
//
// What this module does:
//   A 4-tap sliding window shift register for the incoming I and Q samples.
//   Cubic interpolation needs 4 consecutive neighboring samples [x-1, x0, x1, x2]
//   to reconstruct the continuous-time wave between sample points.
//
// Operation:
//   - Shifts in a new 12-bit I/Q pair whenever 'in_sample_valid' is high.
//   - Pauses and holds data safely if 'in_sample_valid' drops (handles data gaps).
//   - Asserts 'buf_valid' once the first 4 samples have loaded after reset.
//==============================================================================

module str_input_buffer (
  input  logic               clk,
  input  logic               rst_n,
  input  logic signed [11:0] in_sample_i,
  input  logic signed [11:0] in_sample_q,
  input  logic               in_sample_valid,

  output logic signed [11:0] xm1_i,
  output logic signed [11:0] x0_i,
  output logic signed [11:0] x1_i,
  output logic signed [11:0] x2_i,

  output logic signed [11:0] xm1_q,
  output logic signed [11:0] x0_q,
  output logic signed [11:0] x1_q,
  output logic signed [11:0] x2_q,

  output logic               buf_valid
);

  logic [1:0] fill_cnt;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      x2_i      <= 12'sd0;
      x1_i      <= 12'sd0;
      x0_i      <= 12'sd0;
      xm1_i     <= 12'sd0;
      x2_q      <= 12'sd0;
      x1_q      <= 12'sd0;
      x0_q      <= 12'sd0;
      xm1_q     <= 12'sd0;
      fill_cnt  <= 2'd0;
      buf_valid <= 1'b0;
    end else begin
      if (in_sample_valid) begin
        x2_i  <= in_sample_i;
        x1_i  <= x2_i;
        x0_i  <= x1_i;
        xm1_i <= x0_i;

        x2_q  <= in_sample_q;
        x1_q  <= x2_q;
        x0_q  <= x1_q;
        xm1_q <= x0_q;

        if (fill_cnt < 2'd3) begin
          fill_cnt  <= fill_cnt + 2'd1;
          buf_valid <= 1'b0;
        end else begin
          buf_valid <= 1'b1;
        end
      end else begin
        buf_valid <= 1'b0;
      end
    end
  end

endmodule
