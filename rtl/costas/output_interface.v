// Block 9 - Output interface
// Registers the corrected symbol (one per strobe) and the status outputs.
module output_interface (
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [15:0] y_i,
    input  wire signed [15:0] y_q,
    input  wire               y_valid,
    input  wire               lock_i,
    input  wire signed [31:0] fine_cfo_i,
    output reg  signed [15:0] out_i,
    output reg  signed [15:0] out_q,
    output reg                out_valid,
    output reg                lock_detect,
    output reg  signed [31:0] fine_cfo_est
);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_i <= 16'sd0; out_q <= 16'sd0; out_valid <= 1'b0;
            lock_detect <= 1'b0; fine_cfo_est <= 32'sd0;
        end else begin
            out_valid    <= y_valid;
            lock_detect  <= lock_i;
            fine_cfo_est <= fine_cfo_i;
            if (y_valid) begin
                out_i <= y_i;
                out_q <= y_q;
            end
        end
    end
endmodule
