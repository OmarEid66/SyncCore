// Block 2 - NCO phase accumulator (exactly 32 bits, R.120)
// 2^32 = one full turn.  theta <= theta + phase_inc once per symbol.
// phase_adj/adj_valid: optional one-shot phase correction (e.g. k*2^30 to fix
// the 90-degree ambiguity after comparing against the preamble). Tie to 0 if unused.
// bypass: sin/cos see phase 0, so the derotator becomes a pass-through (DFT).
module nco_phase_acc (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        clear,
    input  wire [31:0] phase_inc,
    input  wire        inc_valid,
    input  wire [31:0] phase_adj,
    input  wire        adj_valid,
    input  wire        bypass,
    output wire [31:0] phase_out
);
    reg [31:0] theta;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            theta <= 32'd0;
        else if (clear)
            theta <= 32'd0;
        else if (inc_valid)
            theta <= theta + phase_inc + (adj_valid ? phase_adj : 32'd0);
        else if (adj_valid)
            theta <= theta + phase_adj;
    end

    assign phase_out = bypass ? 32'd0 : theta;
endmodule
