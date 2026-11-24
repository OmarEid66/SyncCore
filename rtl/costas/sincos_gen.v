// Block 3 - Sine / cosine generator
// Quarter-wave ROM (256 x Q1.15, sin(2*pi*(k+0.5)/1024)) addressed by the top
// 10 bits of the 32-bit NCO phase.  cos(t) = sin(t + 90 deg).  Output registered.
// Phase truncation error is +/-0.18 deg (about 0.1 deg RMS).
module sincos_gen #(
    parameter ROM_FILE = "sine_quarter.mem"
) (
    input  wire               clk,
    input  wire        [31:0] phase,
    output reg  signed [15:0] cos_o,
    output reg  signed [15:0] sin_o
);
    reg [15:0] rom [0:255];
    initial $readmemh(ROM_FILE, rom);

    wire [9:0] p_s = phase[31:22];
    wire [9:0] p_c = p_s + 10'd256;

    function signed [15:0] lut;
        input [9:0] p;
        reg   [7:0] idx;
        reg  [15:0] v;
        begin
            idx = p[8] ? ~p[7:0] : p[7:0];   // 255 - idx
            v   = rom[idx];
            lut = p[9] ? -$signed(v) : $signed(v);
        end
    endfunction

    always @(posedge clk) begin
        sin_o <= lut(p_s);
        cos_o <= lut(p_c);
    end
endmodule
