//==============================================================================
// Module: str_output_register
//
// What this module does:
//   The output register stage for the recovered symbols.
//   Captures the on-time interpolated I/Q symbol at Stage S9 and registers it
//   to drive the top-level outputs (out_sym_i, out_sym_q, out_sym_valid).
//
// Why it exists:
//   Isolates internal Farrow interpolator math from external downstream logic,
//   ensuring clean, glitch-free outputs and short clock-to-output path delays.
//==============================================================================

module str_output_register (
  input  logic               clk,
  input  logic               rst_n,

  input  logic signed [15:0] y_i,
  input  logic signed [15:0] y_q,
  input  logic               farrow_valid,
  input  logic               farrow_is_sym,

  output logic signed [15:0] out_sym_i,
  output logic signed [15:0] out_sym_q,
  output logic               out_sym_valid
);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      out_sym_i     <= 16'sd0;
      out_sym_q     <= 16'sd0;
      out_sym_valid <= 1'b0;
    end else begin
      if (farrow_valid && farrow_is_sym) begin
        out_sym_i     <= y_i;
        out_sym_q     <= y_q;
        out_sym_valid <= 1'b1;
      end else begin
        out_sym_valid <= 1'b0;
      end
    end
  end

endmodule
