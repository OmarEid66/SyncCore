// Block 6 - Phase error detector (decision directed)
//   e = yQ*dnI - yI*dnQ   (Q1.15 * Q2.14 = Q3.29, rounded and >>>14 -> Q1.15)
// e is approximately sin(phase error) in "radians", detector gain Kd = 1.
// Stage 1: 2 multipliers.  Stage 2: subtract, round, shift, saturate.
module phase_error_detector (
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [15:0] yi,
    input  wire signed [15:0] yq,
    input  wire signed [15:0] dn_i,
    input  wire signed [15:0] dn_q,
    input  wire               in_valid,
    output reg  signed [15:0] e,
    output reg                e_valid
);
    reg signed [31:0] p1, p2;
    reg               v1;

    wire signed [32:0] diff = p1 - p2;
    wire signed [32:0] sh   = (diff + 33'sd8192) >>> 14;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p1 <= 32'sd0; p2 <= 32'sd0; v1 <= 1'b0;
            e <= 16'sd0; e_valid <= 1'b0;
        end else begin
            p1 <= yq * dn_i;
            p2 <= yi * dn_q;
            v1 <= in_valid;

            e_valid <= v1;
            if (v1)
                e <= (sh >  33'sd32767) ?  16'sd32767 :
                     (sh < -33'sd32767) ? -16'sd32767 : sh[15:0];
        end
    end
endmodule
