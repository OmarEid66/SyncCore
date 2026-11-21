//==============================================================================
// Module: cfo_delay_line
// Description: Parameterized valid-gated sample delay line for complex I/Q data.
//              Provides a delay of L valid samples: r[n-L] aligned with r[n].
//              Only shifts when in_valid is asserted, accommodating streaming gaps.
//==============================================================================

`timescale 1ns / 1ps

module cfo_delay_line #(
    parameter int DATA_WIDTH = 12,
    parameter int DELAY_L    = 16
)(
    input  logic                     clk,
    input  logic                     rst_n,

    // Control
    input  logic                     clr,           // Synchronous clear

    // Input Sample Interface
    input  logic signed [DATA_WIDTH-1:0] in_i,
    input  logic signed [DATA_WIDTH-1:0] in_q,
    input  logic                         in_valid,

    // Delayed Sample Interface
    output logic signed [DATA_WIDTH-1:0] del_out_i,
    output logic signed [DATA_WIDTH-1:0] del_out_q,
    output logic                         del_out_valid
);

    // Shift register memory array
    logic signed [DATA_WIDTH-1:0] shift_reg_i [0:DELAY_L-1];
    logic signed [DATA_WIDTH-1:0] shift_reg_q [0:DELAY_L-1];
    logic                         shift_reg_vld [0:DELAY_L-1];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < DELAY_L; i++) begin
                shift_reg_i[i]   <= '0;
                shift_reg_q[i]   <= '0;
                shift_reg_vld[i] <= 1'b0;
            end
        end else if (clr) begin
            for (int i = 0; i < DELAY_L; i++) begin
                shift_reg_i[i]   <= '0;
                shift_reg_q[i]   <= '0;
                shift_reg_vld[i] <= 1'b0;
            end
        end else if (in_valid) begin
            // Shift pipeline on valid sample
            shift_reg_i[0]   <= in_i;
            shift_reg_q[0]   <= in_q;
            shift_reg_vld[0] <= 1'b1;

            for (int i = 1; i < DELAY_L; i++) begin
                shift_reg_i[i]   <= shift_reg_i[i-1];
                shift_reg_q[i]   <= shift_reg_q[i-1];
                shift_reg_vld[i] <= shift_reg_vld[i-1];
            end
        end
    end

    // Tap L-1 corresponds to a delay of L valid samples
    assign del_out_i     = shift_reg_i[DELAY_L-1];
    assign del_out_q     = shift_reg_q[DELAY_L-1];
    assign del_out_valid = shift_reg_vld[DELAY_L-1] & in_valid;

endmodule
