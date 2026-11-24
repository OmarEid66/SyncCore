// Block 7 - PI loop filter
//   vp    = Kp*e                 (proportional path -> phase)
//   integ = integ + Ki*e         (integral path -> frequency, = fine_cfo_est)
// One 8-bit code g sets both gains (damping fixed near 0.707):
//   Kp = g*2^-11,  Ki = Kp^2/2 = g^2*2^-23   (loop units, Kd = K0 = 1)
// Fixed-point coefficients already include the radian -> NCO-LSB constant 2^32/(2*pi):
//   vp = (e * g*652) >>> 6        di = (e * g*g*10) >>> 12
// g=106 / 157 / 255  <->  Bn ~ 0.02 / 0.03 / 0.05 x symbol rate.
// The integrator saturates at +/-2^30 (+/-0.25 R_sym) as anti-windup.
// freeze = open loop (error forced to 0), used for DFT.
module loop_filter (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               clear,
    input  wire signed [15:0] e,
    input  wire               e_valid,
    input  wire        [7:0]  gain,
    input  wire               freeze,
    output reg  signed [31:0] integ,
    output reg  signed [31:0] vp,
    output reg                out_valid
);
    localparam signed [37:0] ILIM = 38'sd1073741824;   // 2^30

    reg  [17:0] kp_c;                 // g*652      (< 2^18)
    reg  [20:0] ki_c;                 // g*g*10     (< 2^21)
    wire [15:0] g2 = gain * gain;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            kp_c <= 18'd0;
            ki_c <= 21'd0;
        end else begin
            kp_c <= gain * 18'd652;
            ki_c <= g2   * 21'd10;
        end
    end

    wire signed [15:0] e_in = freeze ? 16'sd0 : e;
    wire signed [18:0] kp_s = {1'b0, kp_c};
    wire signed [21:0] ki_s = {1'b0, ki_c};

    reg signed [34:0] prod_p;
    reg signed [37:0] prod_i;
    reg               v1;

    wire signed [34:0] pp_sh = (prod_p + 35'sd32)   >>> 6;
    wire signed [37:0] pi_sh = (prod_i + 38'sd2048) >>> 12;
    wire signed [37:0] i_sum = integ + pi_sh;
    wire signed [37:0] i_new = (i_sum >  ILIM) ?  ILIM :
                               (i_sum < -ILIM) ? -ILIM : i_sum;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prod_p <= 35'sd0; prod_i <= 38'sd0; v1 <= 1'b0;
            integ <= 32'sd0; vp <= 32'sd0; out_valid <= 1'b0;
        end else if (clear) begin
            prod_p <= 35'sd0; prod_i <= 38'sd0; v1 <= 1'b0;
            integ <= 32'sd0; vp <= 32'sd0; out_valid <= 1'b0;
        end else begin
            prod_p <= e_in * kp_s;
            prod_i <= e_in * ki_s;
            v1     <= e_valid;

            out_valid <= v1;
            if (v1) begin
                vp    <= pp_sh[31:0];
                integ <= i_new[31:0];
            end
        end
    end
endmodule
