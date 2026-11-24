//==============================================================================
// Module: farrow_interpolator / farrow_channel
//
// What this module does:
//   Computes interpolated sample values at any arbitrary fractional delay (mu)
//   between physical clock samples using a 4-tap Cubic Lagrange polynomial:
//       y(mu) = c0 + mu * (c1 + mu * (c2 + mu * c3))
//   Applied independently and identically to the I and Q channels.
//
// Hardware Architecture:
//   - Polynomial Horner's rule maps cleanly into 3 cascaded multiply-accumulate
//     stages per channel (6 DSP48E1 slices total across I and Q).
//   - Polynomial coefficients (c0..c3) are calculated using simple additions
//     and bit-shifts (0 DSPs used for coefficients).
//   - Fully pipelined across 8 clock cycles for high operating frequency.
//==============================================================================

module farrow_channel
  import str_pkg::*;
(
  input  logic               clk,
  input  logic               rst_n,
  input  logic signed [11:0] xm1_q11,
  input  logic signed [11:0] x0_q11,
  input  logic signed [11:0] x1_q11,
  input  logic signed [11:0] x2_q11,
  input  logic [11:0]        mu_s2,
  input  logic [11:0]        mu_s4,
  input  logic [11:0]        mu_s6,
  output logic signed [15:0] y_q15
);

  // Align Q1.11 -> Q1.15 (exact << 4), sign-extended to 19 bits
  logic signed [18:0] xm1_e, x0_e, x1_e, x2_e;
  logic signed [18:0] v_c1, v_c3;
  logic signed [21:0] t1_c1, t1_c3;
  logic signed [25:0] t2_c1_comb, t2_c3_comb;
  logic signed [18:0] c1_base_comb, c2_comb, c3_base_comb;

  // Stage S1 registers
  logic signed [17:0] c0_s1;
  logic signed [18:0] c1_base_s1, c2_s1, c3_base_s1;
  logic signed [25:0] t2_c1_s1, t2_c3_s1;

  // Stage S2 combinational & registers (SQ3.15)
  logic signed [34:0] t3_c1, t3_c3;
  logic signed [34:0] t3_c1_sh, t3_c3_sh;
  logic signed [23:0] div6_c1, div6_c3;
  logic signed [23:0] c1_pre, c2_pre, c3_pre;
  logic signed [17:0] c0_s2, c1_s2, c2_s2, c3_s2;

  // Horner Stage 1 (S3 M-reg, S4 P-reg)
  logic signed [12:0] mu_s2_s, mu_s4_s, mu_s6_s;
  (* use_dsp = "yes" *) logic signed [30:0] p3_s3;
  logic signed [30:0] p3_sh;
  logic signed [17:0] c0_s3, c1_s3, c2_s3;
  logic signed [23:0] h2_pre;
  logic signed [17:0] h2_s4, c0_s4, c1_s4;

  // Horner Stage 2 (S5 M-reg, S6 P-reg)
  (* use_dsp = "yes" *) logic signed [30:0] p2_s5;
  logic signed [30:0] p2_sh;
  logic signed [17:0] c0_s5, c1_s5;
  logic signed [23:0] h1_pre;
  logic signed [17:0] h1_s6, c0_s6;

  // Horner Stage 3 (S7 M-reg, S8 P-reg)
  (* use_dsp = "yes" *) logic signed [30:0] p1_s7;
  logic signed [30:0] p1_sh;
  logic signed [17:0] c0_s7;
  logic signed [23:0] y_pre;
  // y_sq3_15 removed: sat_sq3_15+sat_q1_15 merged into sat_q1_15_from_24 to cut 4 CARRY4 from critical path

  always_comb begin
    // S1: align inputs and compute first two factors of 21845 = 5 * 17 * 257
    xm1_e = $signed({{3{xm1_q11[11]}}, xm1_q11, 4'b0000});
    x0_e  = $signed({{3{x0_q11[11]}},  x0_q11,  4'b0000});
    x1_e  = $signed({{3{x1_q11[11]}},  x1_q11,  4'b0000});
    x2_e  = $signed({{3{x2_q11[11]}},  x2_q11,  4'b0000});

    v_c1  = (xm1_e <<< 1) + x2_e;
    v_c3  = x2_e - xm1_e;

    t1_c1 = $signed({{3{v_c1[18]}}, v_c1}) + ($signed({{3{v_c1[18]}}, v_c1}) <<< 2); // *5
    t1_c3 = $signed({{3{v_c3[18]}}, v_c3}) + ($signed({{3{v_c3[18]}}, v_c3}) <<< 2); // *5

    t2_c1_comb = $signed({{4{t1_c1[21]}}, t1_c1}) + ($signed({{4{t1_c1[21]}}, t1_c1}) <<< 4); // *85
    t2_c3_comb = $signed({{4{t1_c3[21]}}, t1_c3}) + ($signed({{4{t1_c3[21]}}, t1_c3}) <<< 4); // *85

    c1_base_comb = x1_e - (x0_e >>> 1);
    c2_comb      = ((xm1_e + x1_e) >>> 1) - x0_e;
    c3_base_comb = (x0_e - x1_e) >>> 1;

    // S2: finish *257 and round >>> 17
    t3_c1    = $signed({{9{t2_c1_s1[25]}}, t2_c1_s1}) + ($signed({{9{t2_c1_s1[25]}}, t2_c1_s1}) <<< 8);
    t3_c3    = $signed({{9{t2_c3_s1[25]}}, t2_c3_s1}) + ($signed({{9{t2_c3_s1[25]}}, t2_c3_s1}) <<< 8);
    t3_c1_sh = (t3_c1 + 35'sd65536) >>> 17;
    t3_c3_sh = (t3_c3 + 35'sd65536) >>> 17;
    div6_c1  = t3_c1_sh[23:0];
    div6_c3  = t3_c3_sh[23:0];

    c1_pre  = $signed({{5{c1_base_s1[18]}}, c1_base_s1}) - div6_c1;
    c2_pre  = $signed({{5{c2_s1[18]}},      c2_s1});
    c3_pre  = div6_c3 + $signed({{5{c3_base_s1[18]}}, c3_base_s1});

    // Signed zero-extended mu for DSP48E1 B port
    mu_s2_s = $signed({1'b0, mu_s2});
    mu_s4_s = $signed({1'b0, mu_s4});
    mu_s6_s = $signed({1'b0, mu_s6});

    // Horner round-and-add combinations (S4, S6, S8)
    p3_sh    = (p3_s3 + 31'sd2048) >>> 12;
    p2_sh    = (p2_s5 + 31'sd2048) >>> 12;
    p1_sh    = (p1_s7 + 31'sd2048) >>> 12;
    h2_pre   = $signed({{6{c2_s3[17]}}, c2_s3}) + $signed(p3_sh[23:0]);
    h1_pre   = $signed({{6{c1_s5[17]}}, c1_s5}) + $signed(p2_sh[23:0]);
    y_pre    = $signed({{6{c0_s7[17]}}, c0_s7}) + $signed(p1_sh[23:0]);
    // sat_sq3_15 removed here; merged into sat_q1_15_from_24 at S8 register
  end

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      c0_s1      <= 18'sd0;
      c1_base_s1 <= 19'sd0;
      c2_s1      <= 19'sd0;
      c3_base_s1 <= 19'sd0;
      t2_c1_s1   <= 26'sd0;
      t2_c3_s1   <= 26'sd0;
      c0_s2      <= 18'sd0;
      c1_s2      <= 18'sd0;
      c2_s2      <= 18'sd0;
      c3_s2      <= 18'sd0;
      p3_s3      <= 31'sd0;
      c0_s3      <= 18'sd0;
      c1_s3      <= 18'sd0;
      c2_s3      <= 18'sd0;
      h2_s4      <= 18'sd0;
      c0_s4      <= 18'sd0;
      c1_s4      <= 18'sd0;
      p2_s5      <= 31'sd0;
      c0_s5      <= 18'sd0;
      c1_s5      <= 18'sd0;
      h1_s6      <= 18'sd0;
      c0_s6      <= 18'sd0;
      p1_s7      <= 31'sd0;
      c0_s7      <= 18'sd0;
      y_q15      <= 16'sd0;
    end else begin
      // S1
      c0_s1      <= x0_e[17:0];
      c1_base_s1 <= c1_base_comb;
      c2_s1      <= c2_comb;
      c3_base_s1 <= c3_base_comb;
      t2_c1_s1   <= t2_c1_comb;
      t2_c3_s1   <= t2_c3_comb;

      // S2 (c1, c2, c3 mathematically bounded < 2.0; fits in SQ3.15 without saturation)
      c0_s2 <= c0_s1;
      c1_s2 <= c1_pre[17:0];
      c2_s2 <= c2_pre[17:0];
      c3_s2 <= c3_pre[17:0];

      // S3 (Horner 1 M-reg)
      p3_s3 <= c3_s2 * mu_s2_s;
      c0_s3 <= c0_s2;
      c1_s3 <= c1_s2;
      c2_s3 <= c2_s2;

      // S4 (Horner 1 P-reg; mathematically bounded in SQ3.15)
      h2_s4 <= h2_pre[17:0];
      c0_s4 <= c0_s3;
      c1_s4 <= c1_s3;

      // S5 (Horner 2 M-reg)
      p2_s5 <= h2_s4 * mu_s4_s;
      c0_s5 <= c0_s4;
      c1_s5 <= c1_s4;

      // S6 (Horner 2 P-reg; mathematically bounded in SQ3.15)
      h1_s6 <= h1_pre[17:0];
      c0_s6 <= c0_s5;

      // S7 (Horner 3 M-reg)
      p1_s7 <= h1_s6 * mu_s6_s;
      c0_s7 <= c0_s6;

      // S8 (Horner 3 P-reg + direct Q1.15 saturation: merged sat removes 4 CARRY4 from critical path)
      y_q15 <= sat_q1_15_from_24(y_pre);
    end
  end

endmodule


module farrow_interpolator
  import str_pkg::*;
(
  input  logic               clk,
  input  logic               rst_n,

  input  logic signed [11:0] xm1_i,
  input  logic signed [11:0] x0_i,
  input  logic signed [11:0] x1_i,
  input  logic signed [11:0] x2_i,

  input  logic signed [11:0] xm1_q,
  input  logic signed [11:0] x0_q,
  input  logic signed [11:0] x1_q,
  input  logic signed [11:0] x2_q,

  // Aligned from timing_phase_accumulator at Stage S2
  input  logic [11:0]        mu_s2,
  input  logic               strobe_s2,
  input  logic               is_sym_s2,

  // Outputs at Stage S8
  output logic signed [15:0] y_i,
  output logic signed [15:0] y_q,
  output logic               farrow_valid,
  output logic               farrow_is_sym
);

  // Shared control & mu delay line (S2 -> S8 = 6 cycles)
  logic [11:0] mu_s3, mu_s4, mu_s5, mu_s6;
  logic [5:0]  strobe_pipe;
  logic [5:0]  is_sym_pipe;

  // mu pipeline uses synchronous reset for clean DSP48E1 BREG inference
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      mu_s3 <= 12'd0;
      mu_s4 <= 12'd0;
      mu_s5 <= 12'd0;
      mu_s6 <= 12'd0;
    end else begin
      mu_s3 <= mu_s2;
      mu_s4 <= mu_s3;
      mu_s5 <= mu_s4;
      mu_s6 <= mu_s5;
    end
  end

  // Control valid/strobe delay line preserves asynchronous reset
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      strobe_pipe <= 6'd0;
      is_sym_pipe <= 6'd0;
    end else begin
      strobe_pipe <= {strobe_pipe[4:0], strobe_s2};
      is_sym_pipe <= {is_sym_pipe[4:0], is_sym_s2};
    end
  end

  assign farrow_valid  = strobe_pipe[5];
  assign farrow_is_sym = is_sym_pipe[5];

  farrow_channel u_farrow_i (
    .clk     (clk),
    .rst_n   (rst_n),
    .xm1_q11 (xm1_i),
    .x0_q11  (x0_i),
    .x1_q11  (x1_i),
    .x2_q11  (x2_i),
    .mu_s2   (mu_s2),
    .mu_s4   (mu_s4),
    .mu_s6   (mu_s6),
    .y_q15   (y_i)
  );

  farrow_channel u_farrow_q (
    .clk     (clk),
    .rst_n   (rst_n),
    .xm1_q11 (xm1_q),
    .x0_q11  (x0_q),
    .x1_q11  (x1_q),
    .x2_q11  (x2_q),
    .mu_s2   (mu_s2),
    .mu_s4   (mu_s4),
    .mu_s6   (mu_s6),
    .y_q15   (y_q)
  );

endmodule
