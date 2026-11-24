//==============================================================================
// Package: str_pkg
//
// What this package does:
//   Contains common parameters, fixed-point bit-widths, and helper functions
//   (like rounding and saturation) used across all STR hardware modules.
//
// Format Summary:
//   - Input I/Q       : 12-bit signed Q1.11  (from ADC/filter)
//   - Output I/Q      : 16-bit signed Q1.15  (clean recovered symbols)
//   - Fractional delay: 12-bit unsigned (mu, range 0.0 to ~1.0)
//   - Gardner error   : 18-bit signed
//   - PI Integrator   : 24-bit signed accumulator
//==============================================================================

package str_pkg;

  // Interface and internal widths
  localparam int W_IN    = 12;
  localparam int W_OUT   = 16;
  localparam int W_MU    = 12;
  localparam int W_ETA   = 22;
  localparam int W_COEFF = 18;
  localparam int W_TED   = 18;
  localparam int W_INTEG = 24;
  localparam int W_CTRL  = 24;

  // Default verified PI gains for Bn*Ts = 0.0080, zeta = 0.707, sig_scale = 0.5
  localparam logic signed [17:0] DEFAULT_KP4_FX = 18'sd10344; // round(4*Kp*2^16)
  localparam logic signed [17:0] DEFAULT_KI_FX  = 18'sd883;   // round(Ki*2^21)

  // NCO constants in UQ0.22 / SQ2.22
  localparam logic [W_ETA-1:0]       ETA_INIT_Q22 = 22'd2097152; // 0.50 * 2^22
  localparam logic signed [W_CTRL-1:0] W_NOM_Q22  = 24'sd2097152; // 0.50 * 2^22
  localparam logic signed [W_CTRL-1:0] W_MIN_Q22  = 24'sd1048576; // 0.25 * 2^22
  localparam logic signed [W_CTRL-1:0] W_MAX_Q22  = 24'sd3145728; // 0.75 * 2^22
  localparam logic signed [W_CTRL-1:0] U_CLAMP_P  = 24'sd1048576; // +0.25 * 2^22
  localparam logic signed [W_CTRL-1:0] U_CLAMP_N  = -24'sd1048576;// -0.25 * 2^22

  // Saturation limits
  localparam logic signed [23:0] SQ3_15_MAX =  24'sd131071;
  localparam logic signed [23:0] SQ3_15_MIN = -24'sd131072;
  localparam logic signed [17:0] Q1_15_MAX  =  18'sd32767;
  localparam logic signed [17:0] Q1_15_MIN  = -18'sd32768;
  localparam logic signed [24:0] SQ9_15_MAX =  25'sd8388607;
  localparam logic signed [24:0] SQ9_15_MIN = -25'sd8388608;

  // Saturate signed 24-bit directly to 16-bit Q1.15
  // Equivalent to sat_q1_15(sat_sq3_15(x)) but avoids cascading carry chains.
  // Proof: for any x, the composed function clips at ±32767/32768 regardless
  //        of whether the intermediate SQ3.15 step would clip first.
  function automatic logic signed [15:0] sat_q1_15_from_24(input logic signed [23:0] x);
    if (x > 24'sd32767)
      return 16'sd32767;
    else if (x < -24'sd32768)
      return -16'sd32768;
    else
      return x[15:0];
  endfunction

  // Saturate signed 24-bit to 18-bit SQ3.15
  function automatic logic signed [17:0] sat_sq3_15(input logic signed [23:0] x);
    if (x > SQ3_15_MAX)
      return 18'sd131071;
    else if (x < SQ3_15_MIN)
      return -18'sd131072;
    else
      return x[17:0];
  endfunction

  // Saturate signed 18-bit SQ3.15 to 16-bit Q1.15
  function automatic logic signed [15:0] sat_q1_15(input logic signed [17:0] x);
    if (x > Q1_15_MAX)
      return 16'sd32767;
    else if (x < Q1_15_MIN)
      return -16'sd32768;
    else
      return x[15:0];
  endfunction

  // Saturate signed 25-bit to 24-bit SQ9.15
  function automatic logic signed [23:0] sat_sq9_15(input logic signed [24:0] x);
    if (x > SQ9_15_MAX)
      return 24'sd8388607;
    else if (x < SQ9_15_MIN)
      return -24'sd8388608;
    else
      return x[23:0];
  endfunction

  // Saturate signed 32-bit to [-0.25, +0.25] in SQ2.22
  function automatic logic signed [23:0] sat_u_q22(input logic signed [31:0] x);
    if (x > 32'sd1048576)
      return U_CLAMP_P;
    else if (x < -32'sd1048576)
      return U_CLAMP_N;
    else
      return x[23:0];
  endfunction

  // Saturate signed 25-bit NCO frequency word to [0.25, 0.75] in SQ2.22
  function automatic logic signed [23:0] sat_w_q22(input logic signed [24:0] x);
    if (x > 25'sd3145728)
      return W_MAX_Q22;
    else if (x < 25'sd1048576)
      return W_MIN_Q22;
    else
      return x[23:0];
  endfunction

endpackage
