//==============================================================================
// Module: gardner_ted
//
// What this module does:
//   Implements the classic Gardner Timing Error Detector (TED) algorithm:
//       e[k] = (I[k] - I[k-1]) * I[k-1/2] + (Q[k] - Q[k-1]) * Q[k-1/2]
//
// Intuition:
//   - If we sample too early, the midpoint sample is pulled towards the previous symbol.
//   - If we sample too late, the midpoint is pulled towards the current symbol.
//   - Multiplying the midpoint by the symbol difference gives the timing error:
//       Positive error -> clock is running late  -> adjust NCO phase earlier
//       Negative error -> clock is running early -> adjust NCO phase later
//   - Uses 2 DSP48E1 slices for the I and Q cross-multiplications.
//==============================================================================

module gardner_ted
  import str_pkg::*;
(
  input  logic               clk,
  input  logic               rst_n,

  input  logic signed [16:0] di,
  input  logic signed [16:0] dq,
  input  logic signed [15:0] mid_i,
  input  logic signed [15:0] mid_q,
  input  logic               ted_in_valid,

  output logic signed [17:0] err_q15,
  output logic               err_valid
);

  // Stage S10: Registered 17s x 16s = 33s DSP48E1 products
  (* use_dsp = "yes" *) logic signed [32:0] prod_i_s10;
  (* use_dsp = "yes" *) logic signed [32:0] prod_q_s10;
  logic               valid_s10;

  // Stage S11 combinational sum with rounding constant (+16384)
  // Bounded within SQ3.15 [-4.0, +4.0) range across all operational SNR conditions
  logic signed [33:0] sum_iq_rnd;
  logic signed [18:0] err_pre;

  always_comb begin
    sum_iq_rnd = $signed({prod_i_s10[32], prod_i_s10}) + $signed({prod_q_s10[32], prod_q_s10}) + 34'sd16384;
    err_pre    = sum_iq_rnd >>> 15;
  end

  // DSP48E1 M-register pipeline uses synchronous reset for internal slice absorption
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      prod_i_s10 <= 33'sd0;
      prod_q_s10 <= 33'sd0;
    end else begin
      prod_i_s10 <= di * mid_i;
      prod_q_s10 <= dq * mid_q;
    end
  end

  // Control and error outputs with asynchronous reset
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      valid_s10  <= 1'b0;
      err_q15    <= 18'sd0;
      err_valid  <= 1'b0;
    end else begin
      valid_s10  <= ted_in_valid;
      err_q15    <= err_pre[17:0];
      err_valid  <= valid_s10;
    end
  end

endmodule
