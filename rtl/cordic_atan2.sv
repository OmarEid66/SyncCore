//==============================================================================
// Module: cordic_atan2
// Description: 16-Stage Fully Pipelined CORDIC Vectoring Engine.
//              Computes theta = atan2(y, x) normalized to [-pi, +pi).
//              Output format: Signed 24-bit Q1.23 where 2^23 represents +pi.
//              Latency: 17 clock cycles (1 pre-rotation + 16 iterations).
//==============================================================================

`timescale 1ns / 1ps

module cordic_atan2 #(
    parameter int XY_WIDTH    = 34,
    parameter int PHASE_WIDTH = 24, // Q1.23 format
    parameter int NUM_STAGES  = 16
)(
    input  logic                          clk,
    input  logic                          rst_n,

    // Input Cartesian Coordinates
    input  logic signed [XY_WIDTH-1:0]    x_in,
    input  logic signed [XY_WIDTH-1:0]    y_in,
    input  logic                          valid_in,

    // Output Polar Phase
    output logic signed [PHASE_WIDTH-1:0] theta_out,
    output logic                          valid_out
);

    // Elementary angles: alpha_i = round(atan(2^-i) / pi * 2^23)
    localparam logic signed [PHASE_WIDTH-1:0] ALPHA [0:NUM_STAGES-1] = '{
        24'sd2097152, // i = 0  (atan(1) = pi/4)
        24'sd1238401, // i = 1
        24'sd654611,  // i = 2
        24'sd332674,  // i = 3
        24'sd167015,  // i = 4
        24'sd83592,   // i = 5
        24'sd41807,   // i = 6
        24'sd20905,   // i = 7
        24'sd10453,   // i = 8
        24'sd5226,    // i = 9
        24'sd2613,    // i = 10
        24'sd1307,    // i = 11
        24'sd653,     // i = 12
        24'sd327,     // i = 13
        24'sd163,     // i = 14
        24'sd82       // i = 15
    };

    localparam logic signed [PHASE_WIDTH-1:0] PI_POS = 24'sd8388607;  // ~ +pi
    localparam logic signed [PHASE_WIDTH-1:0] PI_NEG = -24'sd8388608; // -pi

    // Pipeline Registers
    // Stage 0: Quadrant Pre-Rotation
    localparam int CORDIC_W = 36; // Guarded internal width for CORDIC gain (1.647)

    logic signed [CORDIC_W-1:0]    pipe_x [0:NUM_STAGES];
    logic signed [CORDIC_W-1:0]    pipe_y [0:NUM_STAGES];
    logic signed [PHASE_WIDTH-1:0] pipe_z [0:NUM_STAGES];
    logic                          pipe_vld [0:NUM_STAGES];

    // Stage 0: Initial Pre-Rotation to ensure x >= 0 (Right Half-Plane)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pipe_x[0]   <= '0;
            pipe_y[0]   <= '0;
            pipe_z[0]   <= '0;
            pipe_vld[0] <= 1'b0;
        end else begin
            pipe_vld[0] <= valid_in;
            if (valid_in) begin
                if (x_in < 0) begin
                    pipe_x[0] <= -$signed({ { (CORDIC_W-XY_WIDTH){x_in[XY_WIDTH-1]} }, x_in });
                    pipe_y[0] <= -$signed({ { (CORDIC_W-XY_WIDTH){y_in[XY_WIDTH-1]} }, y_in });
                    if (y_in >= 0)
                        pipe_z[0] <= PI_POS;
                    else
                        pipe_z[0] <= PI_NEG;
                end else begin
                    pipe_x[0] <= $signed({ { (CORDIC_W-XY_WIDTH){x_in[XY_WIDTH-1]} }, x_in });
                    pipe_y[0] <= $signed({ { (CORDIC_W-XY_WIDTH){y_in[XY_WIDTH-1]} }, y_in });
                    pipe_z[0] <= '0;
                end
            end
        end
    end

    // Iterative Stages 1 to NUM_STAGES
    generate
        for (genvar s = 0; s < NUM_STAGES; s++) begin : gen_cordic_stages
            always_ff @(posedge clk or negedge rst_n) begin
                if (!rst_n) begin
                    pipe_x[s+1]   <= '0;
                    pipe_y[s+1]   <= '0;
                    pipe_z[s+1]   <= '0;
                    pipe_vld[s+1] <= 1'b0;
                end else begin
                    pipe_vld[s+1] <= pipe_vld[s];
                    if (pipe_vld[s]) begin
                        logic signed [CORDIC_W-1:0] x_shift;
                        logic signed [CORDIC_W-1:0] y_shift;

                        x_shift = pipe_x[s] >>> s;
                        y_shift = pipe_y[s] >>> s;

                        if (pipe_y[s] >= 0) begin
                            // Clockwise rotation to drive y towards 0
                            pipe_x[s+1] <= pipe_x[s] + y_shift;
                            pipe_y[s+1] <= pipe_y[s] - x_shift;
                            pipe_z[s+1] <= pipe_z[s] + ALPHA[s];
                        end else begin
                            // Counter-clockwise rotation
                            pipe_x[s+1] <= pipe_x[s] - y_shift;
                            pipe_y[s+1] <= pipe_y[s] + x_shift;
                            pipe_z[s+1] <= pipe_z[s] - ALPHA[s];
                        end
                    end
                end
            end
        end
    endgenerate

    assign theta_out = pipe_z[NUM_STAGES];
    assign valid_out = pipe_vld[NUM_STAGES];

endmodule
