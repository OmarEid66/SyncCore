// Block 4 - Complex rotator (derotator)
//   y = x * exp(-j*theta):
//   yI = xI*cos + xQ*sin ,  yQ = xQ*cos - xI*sin
// Stage 1: 4 multipliers. Stage 2: add, round (+2^14), >>>15, saturate to +/-32767.
module complex_rotator (
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [15:0] xi,
    input  wire signed [15:0] xq,
    input  wire               in_valid,
    input  wire signed [15:0] cos_i,
    input  wire signed [15:0] sin_i,
    output reg  signed [15:0] yi,
    output reg  signed [15:0] yq,
    output reg                out_valid
);
    reg signed [31:0] p_ic, p_qs, p_qc, p_is;
    reg               v1;

    wire signed [32:0] sum_i = p_ic + p_qs;
    wire signed [32:0] sum_q = p_qc - p_is;
    wire signed [32:0] sh_i  = (sum_i + 33'sd16384) >>> 15;
    wire signed [32:0] sh_q  = (sum_q + 33'sd16384) >>> 15;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_ic <= 32'sd0; p_qs <= 32'sd0; p_qc <= 32'sd0; p_is <= 32'sd0;
            v1 <= 1'b0; out_valid <= 1'b0;
            yi <= 16'sd0; yq <= 16'sd0;
        end else begin
            p_ic <= xi * cos_i;
            p_qs <= xq * sin_i;
            p_qc <= xq * cos_i;
            p_is <= xi * sin_i;
            v1   <= in_valid;

            out_valid <= v1;
            if (v1) begin
                yi <= (sh_i >  33'sd32767) ?  16'sd32767 :
                      (sh_i < -33'sd32767) ? -16'sd32767 : sh_i[15:0];
                yq <= (sh_q >  33'sd32767) ?  16'sd32767 :
                      (sh_q < -33'sd32767) ? -16'sd32767 : sh_q[15:0];
            end
        end
    end
endmodule
