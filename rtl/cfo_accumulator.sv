//==============================================================================
// Module: cfo_accumulator
// Description: Correlation Accumulator for Complex Correlation Products.
//              Accumulates M valid sample pairs:
//              R_I = sum_{n=0}^{M-1} P_I[n]
//              R_Q = sum_{n=0}^{M-1} P_Q[n]
//              Provides overflow-free accumulation and sample counting.
//==============================================================================

`timescale 1ns / 1ps

module cfo_accumulator #(
    parameter int PROD_WIDTH  = 25,
    parameter int ACCUM_WIDTH = 34,
    parameter int ACCUM_M     = 192
)(
    input  logic                          clk,
    input  logic                          rst_n,

    // Control
    input  logic                          start_accum, // Pulse to clear & begin window
    input  logic                          abort,       // Asynchronous/sync abort

    // Streaming Product Input
    input  logic signed [PROD_WIDTH-1:0]  prod_i,
    input  logic signed [PROD_WIDTH-1:0]  prod_q,
    input  logic                          prod_valid,

    // Latched Output at Completion
    output logic signed [ACCUM_WIDTH-1:0] accum_out_i,
    output logic signed [ACCUM_WIDTH-1:0] accum_out_q,
    output logic                          accum_complete,
    output logic                          accum_busy,
    output logic [$clog2(ACCUM_M):0]      sample_count_out
);

    logic signed [ACCUM_WIDTH-1:0] accum_reg_i;
    logic signed [ACCUM_WIDTH-1:0] accum_reg_q;
    logic [$clog2(ACCUM_M):0]      cnt_r;
    logic                          busy_r;
    logic                          complete_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            accum_reg_i    <= '0;
            accum_reg_q    <= '0;
            accum_out_i    <= '0;
            accum_out_q    <= '0;
            cnt_r          <= '0;
            busy_r         <= 1'b0;
            complete_r     <= 1'b0;
        end else if (abort) begin
            accum_reg_i    <= '0;
            accum_reg_q    <= '0;
            cnt_r          <= '0;
            busy_r         <= 1'b0;
            complete_r     <= 1'b0;
        end else begin
            complete_r <= 1'b0; // Default pulse

            if (start_accum) begin
                accum_reg_i <= '0;
                accum_reg_q <= '0;
                cnt_r       <= '0;
                busy_r      <= 1'b1;
            end else if (busy_r && prod_valid) begin
                if (cnt_r == ACCUM_M - 1) begin
                    // Final sample accumulated: latch output and signal done
                    accum_out_i <= accum_reg_i + $signed(prod_i);
                    accum_out_q <= accum_reg_q + $signed(prod_q);
                    complete_r  <= 1'b1;
                    busy_r      <= 1'b0;
                    cnt_r       <= '0;
                end else begin
                    accum_reg_i <= accum_reg_i + $signed(prod_i);
                    accum_reg_q <= accum_reg_q + $signed(prod_q);
                    cnt_r       <= cnt_r + 1'b1;
                end
            end
        end
    end

    assign accum_complete   = complete_r;
    assign accum_busy       = busy_r;
    assign sample_count_out = cnt_r;

endmodule
