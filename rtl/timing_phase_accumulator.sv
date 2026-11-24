//==============================================================================
// Module: timing_phase_accumulator
//
// What this module does:
//   Acts as the "digital clock" (NCO) for symbol timing. Because the incoming
//   sampling clock and the transmitter's symbol clock have slight timing
//   offsets and frequency drift, this accumulator tracks the exact symbol
//   phase and tells the interpolator where between samples to interpolate.
//
// Key Functions:
//   1. Phase Tracker (eta): Decrements each sample by nominal step (0.5 for 2 sps).
//   2. Strobe Generator   : Flags an underflow when an ideal sample instant passes.
//   3. Symbol/Midpoint Tag: Alternates between midpoint (T/2) and on-time symbol.
//   4. Fractional Delay   : Extracts 12-bit 'mu' (distance between adjacent samples)
//                           to steer the Farrow interpolator.
//   5. Feedback Receiver  : Adjusts phase and frequency based on PI filter output.
//==============================================================================

module timing_phase_accumulator
  import str_pkg::*;
(
  input  logic               clk,
  input  logic               rst_n,
  input  logic               buf_valid,

  // Feedback from timing_pi_controller (Stage S14 -> latched at S15)
  input  logic signed [23:0] u_p_in,
  input  logic signed [23:0] u_i_in,
  input  logic               pi_valid,

  // Aligned outputs to farrow_interpolator (Stage S2)
  output logic [11:0]        mu_s2,
  output logic               strobe_s2,
  output logic               is_sym_s2,

  // DFT Observation tap
  output logic [21:0]        eta_out
);

  // NCO state registers
  logic [21:0]        eta_reg;
  assign eta_out = eta_reg;
  logic               is_symbol_strobe;

  // Stage S15 precomputed feedback registers
  logic signed [23:0] w_freq_reg;
  logic signed [23:0] kick_reg;
  logic signed [23:0] w_plus_kick_reg;
  logic               kick_pending;

  // Stage S1 -> S2 pipeline registers
  logic signed [23:0] eta_k_s1;
  logic               strobe_s1;
  logic               is_sym_s1;

  // Combinational S1 (parallel single-carry-chain subtractors)
  logic               use_kick;
  logic signed [23:0] step_eff;
  logic signed [23:0] kick_eff;
  logic signed [23:0] eta_ext;
  logic signed [23:0] eta_sub;
  logic signed [23:0] eta_k_comb;
  logic [21:0]        eta_next;

  // Combinational S15 precompute
  logic signed [24:0] w_sum_comb;
  logic signed [23:0] w_clamped_comb;

  // Combinational S2 mu rounding & saturation
  logic signed [24:0] eta_2x;
  logic signed [24:0] mu_rnd;
  logic [11:0]        mu_clamped;

  always_comb begin
    use_kick   = buf_valid && is_symbol_strobe && kick_pending;
    step_eff   = use_kick ? w_plus_kick_reg : w_freq_reg;
    kick_eff   = use_kick ? kick_reg        : 24'sd0;
    eta_ext    = $signed({2'b00, eta_reg});

    // Two parallel 24-bit subtractors (never chained)
    eta_sub    = eta_ext - step_eff;
    eta_k_comb = eta_ext - kick_eff;

    // Proven mod-2^22 wrap with explicit saturation guard
    if (eta_sub[23]) begin
      // Underflow: eta_sub + 2^22 == eta_sub[21:0] for -2^22 <= eta_sub < 0
      if (eta_sub < -24'sd4194304)
        eta_next = 22'd0;
      else
        eta_next = eta_sub[21:0];
    end else begin
      if (eta_sub >= 24'sd4194304)
        eta_next = 22'h3F_FFFF;
      else
        eta_next = eta_sub[21:0];
    end

    // Feedback precompute (S15)
    w_sum_comb     = $signed({W_NOM_Q22[23], W_NOM_Q22}) + $signed({u_i_in[23], u_i_in});
    w_clamped_comb = sat_w_q22(w_sum_comb);

    // Stage S2: multiplier-free mu = clamp(round((2 * eta_k_s1) >>> 10), 0, 4095)
    eta_2x = $signed({eta_k_s1, 1'b0});
    mu_rnd = (eta_2x + 25'sd512) >>> 10;
    if (mu_rnd < 25'sd0)
      mu_clamped = 12'd0;
    else if (mu_rnd > 25'sd4095)
      mu_clamped = 12'd4095;
    else
      mu_clamped = mu_rnd[11:0];
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      eta_reg          <= ETA_INIT_Q22;
      is_symbol_strobe <= 1'b0;
      w_freq_reg       <= W_NOM_Q22;
      kick_reg         <= 24'sd0;
      w_plus_kick_reg  <= W_NOM_Q22;
      kick_pending     <= 1'b0;
      eta_k_s1         <= 24'sd0;
      strobe_s1        <= 1'b0;
      is_sym_s1        <= 1'b0;
      mu_s2            <= 12'd0;
      strobe_s2        <= 1'b0;
      is_sym_s2        <= 1'b0;
    end else begin
      // Stage S1: NCO phase update on valid sample
      if (buf_valid) begin
        eta_reg   <= eta_next;
        eta_k_s1  <= eta_k_comb;
        strobe_s1 <= eta_sub[23];
        is_sym_s1 <= is_symbol_strobe;
        if (eta_sub[23]) begin
          is_symbol_strobe <= ~is_symbol_strobe;
        end
      end else begin
        strobe_s1 <= 1'b0;
        is_sym_s1 <= 1'b0;
      end

      // Stage S15: Feedback register update (new pi_valid wins over consumption)
      if (pi_valid) begin
        w_freq_reg      <= w_clamped_comb;
        kick_reg        <= u_p_in;
        w_plus_kick_reg <= w_clamped_comb + u_p_in;
        kick_pending    <= 1'b1;
      end else if (use_kick) begin
        kick_pending    <= 1'b0;
      end

      // Stage S2: Registered UQ0.12 fractional delay aligned with Farrow S2
      mu_s2     <= mu_clamped;
      strobe_s2 <= strobe_s1;
      is_sym_s2 <= is_sym_s1;
    end
  end

endmodule
