//==============================================================================
// Module: str_top
//
// What this module does:
//   This is the top-level Symbol Timing Recovery (STR) block. It takes in raw,
//   oversampled I/Q samples (4 samples per symbol) from the ADC/frontend, finds
//   the exact ideal symbol sampling instants, and outputs one clean recovered
//   symbol per symbol period.
//
// How it works:
//   1. Incoming samples enter a 4-sample shift buffer.
//   2. A timing accumulator (NCO) tracks the fractional symbol phase.
//   3. A Cubic Farrow interpolator calculates values between physical samples.
//   4. A Gardner timing error detector (TED) checks if we sampled early or late.
//   5. A PI loop filter smooths the error and steers the NCO to stay locked.
//
// Key Specifications:
//   - Input : 12-bit signed (Q1.11) @ 4 samples/symbol
//   - Output: 16-bit signed (Q1.15) @ 1 recovered symbol/symbol
//   - Target: AMD/Xilinx Artix-7 XC7A35T FPGA
//   - Latency: 10 clock cycles from input to recovered symbol output
//==============================================================================

module str_top
  import str_pkg::*;
#(
  parameter logic signed [17:0] KP4_FX = DEFAULT_KP4_FX,
  parameter logic signed [17:0] KI_FX  = DEFAULT_KI_FX
) (
  input  logic               clk,
  input  logic               rst_n,

  input  logic signed [11:0] in_sample_i,
  input  logic signed [11:0] in_sample_q,
  input  logic               in_sample_valid,

  output logic signed [15:0] out_sym_i,
  output logic signed [15:0] out_sym_q,
  output logic               out_sym_valid
);

  // S0: Input buffer taps
  logic signed [11:0] xm1_i, x0_i, x1_i, x2_i;
  logic signed [11:0] xm1_q, x0_q, x1_q, x2_q;
  logic               buf_valid;

  // S2: Timing phase accumulator -> Farrow
  logic [11:0]        mu_s2;
  logic               strobe_s2;
  logic               is_sym_s2;

  // S8: Farrow -> Output register & Timing sample scheduler
  logic signed [15:0] farrow_y_i, farrow_y_q;
  logic               farrow_valid;
  logic               farrow_is_sym;

  // S9: Timing sample scheduler -> Gardner TED
  logic signed [16:0] ted_di, ted_dq;
  logic signed [15:0] ted_mid_i, ted_mid_q;
  logic               ted_in_valid;

  // S11: Gardner TED -> PI controller
  logic signed [17:0] ted_err_q15;
  logic               ted_err_valid;

  // S14: PI controller -> Timing phase accumulator (S15 feedback latch)
  logic signed [23:0] pi_u_p_q22;
  logic signed [23:0] pi_u_i_q22;
  logic               pi_valid;

  // Stage S0: 4-tap Q1.11 Input Shift Register
  str_input_buffer u_input_buf (
    .clk             (clk),
    .rst_n           (rst_n),
    .in_sample_i     (in_sample_i),
    .in_sample_q     (in_sample_q),
    .in_sample_valid (in_sample_valid),
    .xm1_i           (xm1_i),
    .x0_i            (x0_i),
    .x1_i            (x1_i),
    .x2_i            (x2_i),
    .xm1_q           (xm1_q),
    .x0_q            (x0_q),
    .x1_q            (x1_q),
    .x2_q            (x2_q),
    .buf_valid       (buf_valid)
  );

  // Stages S1-S2 & S15: Decrementing Mod-1 Timing Phase Accumulator (UQ0.22)
  timing_phase_accumulator u_nco (
    .clk       (clk),
    .rst_n     (rst_n),
    .buf_valid (buf_valid),
    .u_p_in    (pi_u_p_q22),
    .u_i_in    (pi_u_i_q22),
    .pi_valid  (pi_valid),
    .mu_s2     (mu_s2),
    .strobe_s2 (strobe_s2),
    .is_sym_s2 (is_sym_s2)
  );

  // Stages S1-S8: Pipelined Cubic Lagrange Farrow Interpolator (Horner form)
  farrow_interpolator u_farrow (
    .clk           (clk),
    .rst_n         (rst_n),
    .xm1_i         (xm1_i),
    .x0_i          (x0_i),
    .x1_i          (x1_i),
    .x2_i          (x2_i),
    .xm1_q         (xm1_q),
    .x0_q          (x0_q),
    .x1_q          (x1_q),
    .x2_q          (x2_q),
    .mu_s2         (mu_s2),
    .strobe_s2     (strobe_s2),
    .is_sym_s2     (is_sym_s2),
    .y_i           (farrow_y_i),
    .y_q           (farrow_y_q),
    .farrow_valid  (farrow_valid),
    .farrow_is_sym (farrow_is_sym)
  );

  // Stage S9: Registered Q1.15 Output Interface
  str_output_register u_out_reg (
    .clk           (clk),
    .rst_n         (rst_n),
    .y_i           (farrow_y_i),
    .y_q           (farrow_y_q),
    .farrow_valid  (farrow_valid),
    .farrow_is_sym (farrow_is_sym),
    .out_sym_i     (out_sym_i),
    .out_sym_q     (out_sym_q),
    .out_sym_valid (out_sym_valid)
  );

  // Stage S9: Midpoint / Symbol Extractor & TED Pre-Subtractor
  timing_sample_scheduler u_sched (
    .clk           (clk),
    .rst_n         (rst_n),
    .y_i           (farrow_y_i),
    .y_q           (farrow_y_q),
    .farrow_valid  (farrow_valid),
    .farrow_is_sym (farrow_is_sym),
    .ted_di        (ted_di),
    .ted_dq        (ted_dq),
    .ted_mid_i     (ted_mid_i),
    .ted_mid_q     (ted_mid_q),
    .ted_in_valid  (ted_in_valid)
  );

  // Stages S10-S11: Pipelined Gardner Timing Error Detector
  gardner_ted u_ted (
    .clk          (clk),
    .rst_n        (rst_n),
    .di           (ted_di),
    .dq           (ted_dq),
    .mid_i        (ted_mid_i),
    .mid_q        (ted_mid_q),
    .ted_in_valid (ted_in_valid),
    .err_q15      (ted_err_q15),
    .err_valid    (ted_err_valid)
  );

  // Stages S12-S14: Pipelined PI Timing Loop Filter
  timing_pi_controller #(
    .KP4_FX (KP4_FX),
    .KI_FX  (KI_FX)
  ) u_pi (
    .clk       (clk),
    .rst_n     (rst_n),
    .err_q15   (ted_err_q15),
    .err_valid (ted_err_valid),
    .u_p_q22   (pi_u_p_q22),
    .u_i_q22   (pi_u_i_q22),
    .pi_valid  (pi_valid)
  );

endmodule
