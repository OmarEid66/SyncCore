//==============================================================================
// Module: timing_sample_scheduler
//
// What this module does:
//   Acts as the data distributor between the Farrow interpolator and the
//   Gardner Timing Error Detector (TED).
//
// Operation:
//   - Stores the previous on-time symbol sample: I[k-1], Q[k-1]
//   - Captures the midpoint sample:              I[k-1/2], Q[k-1/2]
//   - Receives the current on-time symbol:       I[k], Q[k]
//   - Computes differences: (I[k] - I[k-1]) and (Q[k] - Q[k-1])
//   - Pre-registers all inputs so the Gardner multiplier can execute in a single cycle.
//==============================================================================

module timing_sample_scheduler (
  input  logic               clk,
  input  logic               rst_n,

  input  logic signed [15:0] y_i,
  input  logic signed [15:0] y_q,
  input  logic               farrow_valid,
  input  logic               farrow_is_sym,

  output logic signed [16:0] ted_di,
  output logic signed [16:0] ted_dq,
  output logic signed [15:0] ted_mid_i,
  output logic signed [15:0] ted_mid_q,
  output logic               ted_in_valid
);

  logic signed [15:0] mid_i_reg, mid_q_reg;
  logic signed [15:0] prev_sym_i_reg, prev_sym_q_reg;
  logic               has_mid, has_prev_sym;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      mid_i_reg      <= 16'sd0;
      mid_q_reg      <= 16'sd0;
      prev_sym_i_reg <= 16'sd0;
      prev_sym_q_reg <= 16'sd0;
      has_mid        <= 1'b0;
      has_prev_sym   <= 1'b0;
      ted_di         <= 17'sd0;
      ted_dq         <= 17'sd0;
      ted_mid_i      <= 16'sd0;
      ted_mid_q      <= 16'sd0;
      ted_in_valid   <= 1'b0;
    end else begin
      ted_in_valid <= 1'b0;

      if (farrow_valid) begin
        if (!farrow_is_sym) begin
          // Capture midpoint sample y(k - 1/2)
          mid_i_reg <= y_i;
          mid_q_reg <= y_q;
          has_mid   <= 1'b1;
        end else begin
          // Symbol sample y(k): update history & emit registered TED operands
          prev_sym_i_reg <= y_i;
          prev_sym_q_reg <= y_q;
          has_prev_sym   <= 1'b1;

          if (has_prev_sym && has_mid) begin
            ted_di       <= $signed({y_i[15], y_i}) - $signed({prev_sym_i_reg[15], prev_sym_i_reg});
            ted_dq       <= $signed({y_q[15], y_q}) - $signed({prev_sym_q_reg[15], prev_sym_q_reg});
            ted_mid_i    <= mid_i_reg;
            ted_mid_q    <= mid_q_reg;
            ted_in_valid <= 1'b1;
          end
        end
      end
    end
  end

endmodule
