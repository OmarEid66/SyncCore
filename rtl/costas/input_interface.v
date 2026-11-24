// Block 1 - Input interface
// Registers one symbol per x_valid strobe (output of STR / coarse CFO stage).
// -32768 is clamped to -32767 so that negation never overflows downstream.
module input_interface (
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [15:0] x_i,
    input  wire signed [15:0] x_q,
    input  wire               x_valid,
    output reg  signed [15:0] xr_i,
    output reg  signed [15:0] xr_q,
    output reg                xr_valid
);
    localparam signed [15:0] MIN_V = -16'sd32767;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            xr_i     <= 16'sd0;
            xr_q     <= 16'sd0;
            xr_valid <= 1'b0;
        end else begin
            xr_valid <= x_valid;
            if (x_valid) begin
                xr_i <= (x_i == -16'sd32768) ? MIN_V : x_i;
                xr_q <= (x_q == -16'sd32768) ? MIN_V : x_q;
            end
        end
    end
endmodule
