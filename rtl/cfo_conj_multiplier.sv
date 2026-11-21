//==============================================================================
// Module: cfo_conj_multiplier
// Description: Pipelined Complex Conjugate Multiplier
//              Implements P[n] = r[n] * conj(r[n-L])
//              P_I[n] = I_n * I_d + Q_n * Q_d
//              P_Q[n] = Q_n * I_d - I_n * Q_d
//              Includes 2-stage pipeline for timing closure at >= 200 MHz.
//==============================================================================

`timescale 1ns / 1ps

module cfo_conj_multiplier #(
    parameter int DATA_WIDTH = 12,
    parameter int PROD_WIDTH = 25  // 12x12 -> 24b + 1b sum = 25b
)(
    input  logic                     clk,
    input  logic                     rst_n,

    // Input Samples (Current and Delayed)
    input  logic signed [DATA_WIDTH-1:0] in_i,
    input  logic signed [DATA_WIDTH-1:0] in_q,
    input  logic signed [DATA_WIDTH-1:0] del_i,
    input  logic signed [DATA_WIDTH-1:0] del_q,
    input  logic                         in_valid,

    // Conjugate Product Output
    output logic signed [PROD_WIDTH-1:0] prod_i,
    output logic signed [PROD_WIDTH-1:0] prod_q,
    output logic                         prod_valid
);

    // Stage 1: Signed 12x12 Multiplications (24-bit signed)
    localparam int MULT_WIDTH = DATA_WIDTH * 2; // 24 bits

    logic signed [MULT_WIDTH-1:0] mult_ii_r;
    logic signed [MULT_WIDTH-1:0] mult_qq_r;
    logic signed [MULT_WIDTH-1:0] mult_qi_r;
    logic signed [MULT_WIDTH-1:0] mult_iq_r;
    logic                         stage1_valid_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mult_ii_r      <= '0;
            mult_qq_r      <= '0;
            mult_qi_r      <= '0;
            mult_iq_r      <= '0;
            stage1_valid_r <= 1'b0;
        end else begin
            stage1_valid_r <= in_valid;
            if (in_valid) begin
                mult_ii_r <= in_i * del_i;
                mult_qq_r <= in_q * del_q;
                mult_qi_r <= in_q * del_i;
                mult_iq_r <= in_i * del_q;
            end
        end
    end

    // Stage 2: Add / Subtract (25-bit signed)
    logic signed [PROD_WIDTH-1:0] prod_i_r;
    logic signed [PROD_WIDTH-1:0] prod_q_r;
    logic                         stage2_valid_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prod_i_r       <= '0;
            prod_q_r       <= '0;
            stage2_valid_r <= 1'b0;
        end else begin
            stage2_valid_r <= stage1_valid_r;
            if (stage1_valid_r) begin
                prod_i_r <= $signed(mult_ii_r) + $signed(mult_qq_r);
                prod_q_r <= $signed(mult_qi_r) - $signed(mult_iq_r);
            end
        end
    end

    assign prod_i     = prod_i_r;
    assign prod_q     = prod_q_r;
    assign prod_valid = stage2_valid_r;

endmodule
