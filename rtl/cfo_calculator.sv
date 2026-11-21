//==============================================================================
// Module: cfo_calculator
// Description: Converts CORDIC phase estimate into normalized CFO and 32-bit
//              NCO Frequency Tuning Word (FTW).
//              Formula: f_norm = theta / (16 * pi) = (theta / pi) >>> 4
//              In signed Q1.23: simple arithmetic right shift by 4 bits!
//              NCO FTW: nco_ftw = $signed(coarse_cfo_est) <<< 8.
//==============================================================================

`timescale 1ns / 1ps

module cfo_calculator #(
    parameter int PHASE_WIDTH = 24, // Q1.23 format from CORDIC
    parameter int CFO_WIDTH   = 24, // Q1.23 format output
    parameter int NCO_WIDTH   = 32  // Downstream Costas NCO accumulator width
)(
    input  logic                          clk,
    input  logic                          rst_n,

    // Input Phase from CORDIC
    input  logic signed [PHASE_WIDTH-1:0] theta_in,
    input  logic                          valid_in,

    // CFO Output Register Interface
    output logic signed [CFO_WIDTH-1:0]   coarse_cfo_est,
    output logic                          coarse_cfo_valid,

    // Downstream NCO Tuning Word
    output logic signed [NCO_WIDTH-1:0]   nco_ftw
);

    logic signed [CFO_WIDTH-1:0] cfo_est_r;
    logic                        cfo_valid_r;
    logic signed [NCO_WIDTH-1:0] nco_ftw_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfo_est_r   <= '0;
            cfo_valid_r <= 1'b0;
            nco_ftw_r   <= '0;
        end else begin
            cfo_valid_r <= valid_in;
            if (valid_in) begin
                // Arithmetic right shift by 4 bits: f_norm = (theta/pi) / 16
                cfo_est_r <= $signed(theta_in) >>> 4;

                // 32-bit NCO Frequency Tuning Word:
                // assign nco_ftw = $signed(coarse_cfo_est) <<< 8;
                nco_ftw_r <= $signed($signed(theta_in) >>> 4) <<< 8;
            end
        end
    end

    assign coarse_cfo_est   = cfo_est_r;
    assign coarse_cfo_valid = cfo_valid_r;
    assign nco_ftw          = nco_ftw_r;

endmodule
