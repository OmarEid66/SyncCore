//==============================================================================
// Module: timing_pi_controller
//
// What this module does:
//   A second-order Proportional-Integral (PI) digital loop filter.
//   It takes the noisy timing error e[k] from the Gardner TED and produces
//   two control corrections for the NCO:
//     1. Proportional term (u_p = Kp * e): Provides fast phase correction.
//     2. Integral term     (u_i = Ki * sum(e)): Tracks and cancels static frequency
//                                               offsets (PPM mismatch).
//
// Hardware Architecture:
//   - Uses 2 DSP48E1 slices (one for Kp multiplication, one for Ki).
//   - Fully pipelined across 3 clock cycles with internal clamping/anti-windup
//     to prevent accumulator overflow.
//==============================================================================

module timing_pi_controller
  import str_pkg::*;
#(
  parameter logic signed [17:0] KP4_FX = DEFAULT_KP4_FX,
  parameter logic signed [17:0] KI_FX  = DEFAULT_KI_FX
) (
  input  logic               clk,
  input  logic               rst_n,

  input  logic signed [17:0] err_q15,
  input  logic               err_valid,

  output logic signed [23:0] u_p_q22,
  output logic signed [23:0] u_i_q22,
  output logic               pi_valid
);

  // Stage S12 registers
  logic signed [23:0] integ_reg;
  (* use_dsp = "yes" *) logic signed [35:0] p_prod_s12;
  logic               valid_s12;

  // Stage S13 registers
  (* use_dsp = "yes" *) logic signed [41:0] i_prod_s13;
  logic signed [23:0] u_p_s13;
  logic               valid_s13;

  // Combinational helpers
  logic signed [24:0] integ_sum;
  logic signed [35:0] p_rnd_sh;
  logic signed [31:0] u_p_pre;
  logic signed [41:0] i_rnd_sh;
  logic signed [31:0] u_i_pre;

  always_comb begin
    integ_sum = $signed({integ_reg[23], integ_reg}) + $signed({{7{err_q15[17]}}, err_q15});

    p_rnd_sh  = (p_prod_s12 + 36'sd256) >>> 9;
    u_p_pre   = p_rnd_sh[31:0];

    i_rnd_sh  = (i_prod_s13 + 42'sd8192) >>> 14;
    u_i_pre   = i_rnd_sh[31:0];
  end

  // Datapath & DSP registers use synchronous reset for DSP48E1 absorption
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      integ_reg  <= 24'sd0;
      p_prod_s12 <= 36'sd0;
      i_prod_s13 <= 42'sd0;
      u_p_s13    <= 24'sd0;
      u_p_q22    <= 24'sd0;
      u_i_q22    <= 24'sd0;
    end else begin
      // S12: Integrator accumulate (on err_valid) + Proportional multiply
      if (err_valid) begin
        integ_reg  <= sat_sq9_15(integ_sum);
        p_prod_s12 <= err_q15 * KP4_FX;
      end

      // S13: Integral multiply (using updated integ_reg) + saturate u_p
      if (valid_s12) begin
        i_prod_s13 <= integ_reg * KI_FX;
        u_p_s13    <= sat_u_q22(u_p_pre);
      end

      // S14: Saturate u_i and emit aligned (u_p_q22, u_i_q22, pi_valid)
      if (valid_s13) begin
        u_p_q22 <= u_p_s13;
        u_i_q22 <= sat_u_q22(u_i_pre);
      end
    end
  end

  // Valid flag pipeline with asynchronous reset
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      valid_s12 <= 1'b0;
      valid_s13 <= 1'b0;
      pi_valid  <= 1'b0;
    end else begin
      valid_s12 <= err_valid;
      valid_s13 <= valid_s12;
      pi_valid  <= valid_s13;
    end
  end

endmodule
