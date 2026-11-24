// Block 5 - Decision / slicer
// Finds the nearest ideal constellation point d and outputs the NORMALISED
// reference dn = d / |d|^2 in Q2.14, so that the phase error is simply
//   e = Im{y * conj(d)} / |d|^2 = yQ*dnI - yI*dnQ      (no divider needed)
// mod_type: 2'b00 = QPSK (points +/-0.7071 per axis), 2'b01 = 16-QAM
//           (levels +/-1/sqrt(10), +/-3/sqrt(10)).  Assumes unit average symbol power.
// Other mod_type codes behave as QPSK.
module decision_slicer (
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [15:0] yi,
    input  wire signed [15:0] yq,
    input  wire               y_valid,
    input  wire        [1:0]  mod_type,
    output reg  signed [15:0] yi_d,
    output reg  signed [15:0] yq_d,
    output reg  signed [15:0] dn_i,
    output reg  signed [15:0] dn_q,
    output reg                out_valid
);
    localparam [15:0] TH_QAM  = 16'd20724;   // 2/sqrt(10) in Q1.15
    localparam [15:0] A_QPSK  = 16'd11585;   // 0.7071 in Q2.14
    localparam [15:0] DN_IN_IN   = 16'd25905; // |I| inner, |Q| inner : both
    localparam [15:0] DN_IN_OUT_I = 16'd5181; // |I| inner, |Q| outer : I part
    localparam [15:0] DN_IN_OUT_Q = 16'd15543;//                       Q part
    localparam [15:0] DN_OUT_OUT = 16'd8635;  // both outer

    wire [15:0] ay_i = yi[15] ? (~yi + 16'd1) : yi;   // yi >= -32767, no overflow
    wire [15:0] ay_q = yq[15] ? (~yq + 16'd1) : yq;
    wire        out_i = (ay_i > TH_QAM);
    wire        out_q = (ay_q > TH_QAM);

    reg [15:0] mag_i, mag_q;
    always @* begin
        if (mod_type == 2'b01) begin
            case ({out_i, out_q})
                2'b00:   begin mag_i = DN_IN_IN;      mag_q = DN_IN_IN;      end
                2'b01:   begin mag_i = DN_IN_OUT_I;   mag_q = DN_IN_OUT_Q;   end
                2'b10:   begin mag_i = DN_IN_OUT_Q;   mag_q = DN_IN_OUT_I;   end
                default: begin mag_i = DN_OUT_OUT;    mag_q = DN_OUT_OUT;    end
            endcase
        end else begin
            mag_i = A_QPSK;
            mag_q = A_QPSK;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            yi_d <= 16'sd0; yq_d <= 16'sd0;
            dn_i <= 16'sd0; dn_q <= 16'sd0;
            out_valid <= 1'b0;
        end else begin
            out_valid <= y_valid;
            if (y_valid) begin
                yi_d <= yi;
                yq_d <= yq;
                dn_i <= yi[15] ? -$signed(mag_i) : $signed(mag_i);
                dn_q <= yq[15] ? -$signed(mag_q) : $signed(mag_q);
            end
        end
    end
endmodule
