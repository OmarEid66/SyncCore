//==============================================================================
// Module: coarse_derotator
// Description: Coarse Carrier Frequency Offset (CFO) Derotator.
//              Converts input 12-bit signed Q1.11 baseband samples to 16-bit Q1.15,
//              and rotates them by exp(-j * theta_coarse[n]) using a 32-bit NCO,
//              a quarter-wave sine/cosine lookup table, and a complex multiplier.
//              Latency: Exactly 3 clock cycles (1 cycle sincos + 2 cycles complex rotator).
//==============================================================================

`timescale 1ns / 1ps

module coarse_derotator #(
    parameter int DATA_IN_WIDTH  = 12,                  // Signed Q1.11 per channel
    parameter int DATA_OUT_WIDTH = 16,                  // Signed Q1.15 per channel
    parameter int NCO_WIDTH      = 32,                  // 32-bit phase accumulator
    parameter     ROM_FILE       = "sine_quarter.mem"   // Quarter-wave LUT file
)(
    input  logic                          clk,
    input  logic                          rst_n,
    input  logic                          clear,            // Clear NCO phase & reset active status

    // Coarse CFO Tuning Word Interface
    input  logic                          ftw_valid,        // Strobe to latch new coarse FTW
    input  logic signed [NCO_WIDTH-1:0]   ftw_in,           // 32-bit FTW from coarse CFO estimator

    // DFT & Debug Controls
    input  logic                          force_en,         // Force manual FTW
    input  logic signed [NCO_WIDTH-1:0]   force_ftw,        // Manual FTW value
    input  logic                          bypass,           // Bypass derotation (pass-through)

    // Streaming Input Samples (Q1.11, e.g. 4 sps from STR / ADC)
    input  logic signed [DATA_IN_WIDTH-1:0] sample_i_in,
    input  logic signed [DATA_IN_WIDTH-1:0] sample_q_in,
    input  logic                            sample_valid_in,

    // Streaming Output Derotated Samples (Q1.15, 3-cycle latency)
    output logic signed [DATA_OUT_WIDTH-1:0] sample_i_out,
    output logic signed [DATA_OUT_WIDTH-1:0] sample_q_out,
    output logic                             sample_valid_out,

    // DFT Observation Taps
    output logic [NCO_WIDTH-1:0]             dft_phase_out,
    output logic signed [NCO_WIDTH-1:0]      dft_active_ftw,
    output logic                             derot_active
);

    // 1. FTW Latch Register
    logic signed [NCO_WIDTH-1:0] active_ftw_r;
    logic                        active_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active_ftw_r <= '0;
            active_r     <= 1'b0;
        end else if (clear) begin
            active_ftw_r <= '0;
            active_r     <= 1'b0;
        end else if (force_en) begin
            active_ftw_r <= force_ftw;
            active_r     <= 1'b1;
        end else if (ftw_valid) begin
            active_ftw_r <= ftw_in;
            active_r     <= 1'b1;
        end
    end

    // 2. 32-Bit NCO Phase Accumulator
    logic [NCO_WIDTH-1:0] phase_acc_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase_acc_r <= '0;
        end else if (clear) begin
            phase_acc_r <= '0;
        end else if (sample_valid_in) begin
            if (active_r && !bypass) begin
                phase_acc_r <= phase_acc_r + active_ftw_r;
            end
        end
    end

    // Phase into sine/cosine generator: forced to 0 when unrotated / bypassed
    wire [NCO_WIDTH-1:0] phase_to_sc = (active_r && !bypass) ? phase_acc_r : 32'd0;

    // 3. Quarter-Wave Sine/Cosine Generator (1-cycle latency)
    logic signed [15:0] cos_w;
    logic signed [15:0] sin_w;

    sincos_gen #(
        .ROM_FILE (ROM_FILE)
    ) u_sincos (
        .clk   (clk),
        .phase (phase_to_sc),
        .cos_o (cos_w),
        .sin_o (sin_w)
    );

    // 4. Data Width Scaling: 12-bit Q1.11 to 16-bit Q1.15
    logic signed [15:0] in_scaled_i;
    logic signed [15:0] in_scaled_q;

    generate
        if (DATA_IN_WIDTH == 12) begin : gen_w12
            assign in_scaled_i = {sample_i_in, 4'b0000};
            assign in_scaled_q = {sample_q_in, 4'b0000};
        end else if (DATA_IN_WIDTH == 16) begin : gen_w16
            assign in_scaled_i = sample_i_in;
            assign in_scaled_q = sample_q_in;
        end else if (DATA_IN_WIDTH < 16) begin : gen_wless
            assign in_scaled_i = {sample_i_in, {(16-DATA_IN_WIDTH){1'b0}}};
            assign in_scaled_q = {sample_q_in, {(16-DATA_IN_WIDTH){1'b0}}};
        end else begin : gen_wmore
            assign in_scaled_i = sample_i_in[DATA_IN_WIDTH-1 : DATA_IN_WIDTH-16];
            assign in_scaled_q = sample_q_in[DATA_IN_WIDTH-1 : DATA_IN_WIDTH-16];
        end
    endgenerate

    // Register scaled data by 1 cycle to align with sincos_gen output
    logic signed [15:0] in_d1_i;
    logic signed [15:0] in_d1_q;
    logic               valid_d1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_d1_i  <= 16'sd0;
            in_d1_q  <= 16'sd0;
            valid_d1 <= 1'b0;
        end else begin
            in_d1_i  <= in_scaled_i;
            in_d1_q  <= in_scaled_q;
            valid_d1 <= sample_valid_in;
        end
    end

    // 5. Complex Rotator (2-cycle latency)
    // Computes: y = (I + jQ) * exp(-j * theta) = (I*cos + Q*sin) + j(Q*cos - I*sin)
    logic signed [15:0] rot_i;
    logic signed [15:0] rot_q;
    logic               rot_valid;

    complex_rotator u_rot (
        .clk       (clk),
        .rst_n     (rst_n),
        .xi        (in_d1_i),
        .xq        (in_d1_q),
        .in_valid  (valid_d1),
        .cos_i     (cos_w),
        .sin_i     (sin_w),
        .yi        (rot_i),
        .yq        (rot_q),
        .out_valid (rot_valid)
    );

    // 6. Output Assignment (Signed Q1.15)
    assign sample_i_out     = rot_i;
    assign sample_q_out     = rot_q;
    assign sample_valid_out = rot_valid;

    // DFT Outputs
    assign dft_phase_out  = phase_acc_r;
    assign dft_active_ftw = active_ftw_r;
    assign derot_active   = active_r;

endmodule
