//==============================================================================
// Module: cfo_preamble_detector
// Description: Dual-Mode Preamble Detector
//              Mode A: Autonomous detection via normalized auto-correlation metric:
//                      M[n] = |C[n]|^2 / (E1[n] * E2[n] + eps) >= gamma_thresh
//                      and E1[n] >= energy_min_thresh for N_consec samples.
//              Mode B: External trigger via preamble_enable and burst_start.
//==============================================================================

`timescale 1ns / 1ps

module cfo_preamble_detector #(
    parameter int DATA_WIDTH     = 12,
    parameter int PROD_WIDTH     = 25,
    parameter int DELAY_L        = 16,
    parameter int N_CONSEC_WIN   = 2
)(
    input  logic                          clk,
    input  logic                          rst_n,

    // Control & Mode Select
    input  logic                          burst_start,      // Arms/resets detector
    input  logic                          preamble_enable,  // Mode B external trigger
    input  logic                          auto_mode_en,     // 1: Autonomous, 0: External

    // Programmable Thresholds
    input  logic [15:0]                   gamma_scaled,     // Q0.16 threshold (e.g. 0.55 * 65536 = 36045)
    input  logic [27:0]                   energy_min_thresh,// Minimum window energy

    // Inputs from Multiplier & Delay Line
    input  logic signed [DATA_WIDTH-1:0]  in_i,
    input  logic signed [DATA_WIDTH-1:0]  in_q,
    input  logic signed [DATA_WIDTH-1:0]  del_i,
    input  logic signed [DATA_WIDTH-1:0]  del_q,
    input  logic signed [PROD_WIDTH-1:0]  prod_i,
    input  logic signed [PROD_WIDTH-1:0]  prod_q,
    input  logic                          prod_valid,

    // Detection Outputs
    output logic                          preamble_detected,
    output logic [31:0]                   det_metric_out,
    output logic                          armed
);

    // 1. Moving Sum of Products C_I, C_Q over L samples
    localparam int C_SUM_WIDTH = PROD_WIDTH + 4; // 29 bits signed

    logic signed [PROD_WIDTH-1:0] prod_i_pipe [0:DELAY_L-1];
    logic signed [PROD_WIDTH-1:0] prod_q_pipe [0:DELAY_L-1];
    logic signed [C_SUM_WIDTH-1:0] c_i_acc;
    logic signed [C_SUM_WIDTH-1:0] c_q_acc;

    // 2. Moving Sum of Energies E1, E2 over L samples
    // Input retiming registers: break long delay-line -> DSP48 route.
    // Free-running (no clock enable) so Vivado absorbs them into DSP48 AREG.
    logic signed [DATA_WIDTH-1:0] in_i_r,  in_q_r;
    logic signed [DATA_WIDTH-1:0] del_i_r, del_q_r;
    logic                         prod_valid_d1;  // prod_valid delayed 1 to align with retimed inputs

    always_ff @(posedge clk) begin
        in_i_r       <= in_i;
        in_q_r       <= in_q;
        del_i_r      <= del_i;
        del_q_r      <= del_q;
        prod_valid_d1 <= prod_valid;
    end

    // Stage 0: Registered individual squaring (12b signed * 12b signed -> 24b signed)
    // Fully pipelined into DSP48E1 with PREG=1 to achieve > 200 MHz with zero cascade.
    // Squarers use retimed _r inputs — driven by local FFs, not distant delay line taps.
    logic signed [23:0] in_i_sq_r,  in_q_sq_r;
    logic signed [23:0] del_i_sq_r, del_q_sq_r;
    logic signed [PROD_WIDTH-1:0] prod_i_d1, prod_q_d1;
    logic                         prod_vld_d1;

    // Stage 1: Registered instantaneous power: I^2 + Q^2
    logic [23:0] inst_e1_r, inst_e2_r;
    logic signed [PROD_WIDTH-1:0] prod_i_r, prod_q_r;
    logic                         prod_vld_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_i_sq_r   <= '0;
            in_q_sq_r   <= '0;
            del_i_sq_r  <= '0;
            del_q_sq_r  <= '0;
            prod_i_d1   <= '0;
            prod_q_d1   <= '0;
            prod_vld_d1 <= 1'b0;

            inst_e1_r   <= '0;
            inst_e2_r   <= '0;
            prod_i_r    <= '0;
            prod_q_r    <= '0;
            prod_vld_r  <= 1'b0;
        end else begin
            // Stage 0: Compute individual squares using retimed inputs (no cascade, PREG=1)
            prod_vld_d1 <= prod_valid_d1;
            if (prod_valid_d1) begin
                in_i_sq_r   <= in_i_r  * in_i_r;
                in_q_sq_r   <= in_q_r  * in_q_r;
                del_i_sq_r  <= del_i_r * del_i_r;
                del_q_sq_r  <= del_q_r * del_q_r;
                prod_i_d1   <= prod_i;
                prod_q_d1   <= prod_q;
            end

            // Stage 1: Add paired squares into instantaneous energy
            prod_vld_r <= prod_vld_d1;
            if (prod_vld_d1) begin
                inst_e1_r <= in_i_sq_r  + in_q_sq_r;
                inst_e2_r <= del_i_sq_r + del_q_sq_r;
                prod_i_r  <= prod_i_d1;
                prod_q_r  <= prod_q_d1;
            end
        end
    end

    logic [23:0] e1_pipe [0:DELAY_L-1];
    logic [23:0] e2_pipe [0:DELAY_L-1];
    logic [27:0] e1_acc;
    logic [27:0] e2_acc;

    // Pipeline registers for high-speed timing closure (>= 200 MHz)
    (* DONT_TOUCH = "yes" *) logic signed [15:0] c_i_t_a, c_i_t_b;
    (* DONT_TOUCH = "yes" *) logic signed [15:0] c_q_t_a, c_q_t_b;
    logic [15:0]        e1_t_r, e2_t_r;
    logic [27:0]        e1_acc_pipe1;
    logic               pipe1_vld;

    // Stage 1: Auto-correlation magnitude squares & Energy Product
    logic [31:0]        c_i_sq_r, c_q_sq_r;
    logic [31:0]        e_prod_r;
    logic [27:0]        e1_acc_pipe2;
    logic               pipe2_vld;

    // Stage 2: Correlation Magnitude Sum & Registered Energy Truncation
    logic [31:0]        c_mag_sq_r;
    logic [15:0]        e_prod_trunc_r;
    logic [27:0]        e1_acc_pipe3;
    logic               pipe3_vld;

    // Stage 3: Threshold Multiplier (DSP48E1 with AREG=1, PREG=1) & Aligned Magnitude
    logic [31:0]        c_mag_sq_d1;
    logic [31:0]        thresh_cmp_r;
    logic [27:0]        e1_acc_pipe4;
    logic               pipe4_vld;

    // Pipeline control registers (with reset)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pipe1_vld <= 1'b0;
            pipe2_vld <= 1'b0;
            pipe3_vld <= 1'b0;
            pipe4_vld <= 1'b0;
        end else if (burst_start) begin
            pipe1_vld <= 1'b0;
            pipe2_vld <= 1'b0;
            pipe3_vld <= 1'b0;
            pipe4_vld <= 1'b0;
        end else begin
            pipe1_vld <= prod_valid && auto_mode_en;
            pipe2_vld <= pipe1_vld;
            pipe3_vld <= pipe2_vld;
            pipe4_vld <= pipe3_vld;
        end
    end

    // Pipeline datapath registers (synchronous, allows DSP48E1 AREG/BREG/PREG packing)
    always_ff @(posedge clk) begin
        // Stage 0 -> Stage 1: Capture truncated window values with dedicated A/B inputs
        c_i_t_a      <= c_i_acc[27 -: 16];
        c_i_t_b      <= c_i_acc[27 -: 16];
        c_q_t_a      <= c_q_acc[27 -: 16];
        c_q_t_b      <= c_q_acc[27 -: 16];
        e1_t_r       <= e1_acc[27 -: 16];
        e2_t_r       <= e2_acc[27 -: 16];
        e1_acc_pipe1 <= e1_acc;

        // Stage 1 -> Stage 2: Register DSP products (absorbs into AREG=1, BREG=1, PREG=1)
        c_i_sq_r     <= $signed(c_i_t_a) * $signed(c_i_t_b);
        c_q_sq_r     <= $signed(c_q_t_a) * $signed(c_q_t_b);
        e_prod_r     <= (e1_t_r * e2_t_r) >> 16;
        e1_acc_pipe2 <= e1_acc_pipe1;

        // Stage 2 -> Stage 3: Sum correlation squares & isolate energy product input
        c_mag_sq_r     <= (c_i_sq_r + c_q_sq_r) >> 16;
        e_prod_trunc_r <= e_prod_r[15:0];
        e1_acc_pipe3   <= e1_acc_pipe2;

        // Stage 3 -> Stage 4: Threshold product (AREG=1, PREG=1) & alignment register
        c_mag_sq_d1  <= c_mag_sq_r;
        thresh_cmp_r <= (e_prod_trunc_r * gamma_scaled) >> 16;
        e1_acc_pipe4 <= e1_acc_pipe3;
    end

    // Armed status & detection logic
    logic armed_r;
    logic [3:0] consec_cnt;
    logic det_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            armed_r   <= 1'b1;
            c_i_acc   <= '0;
            c_q_acc   <= '0;
            e1_acc    <= '0;
            e2_acc    <= '0;
            consec_cnt<= '0;
            det_r     <= 1'b0;
            for (int k = 0; k < DELAY_L; k++) begin
                prod_i_pipe[k] <= '0;
                prod_q_pipe[k] <= '0;
                e1_pipe[k]     <= '0;
                e2_pipe[k]     <= '0;
            end
        end else if (burst_start) begin
            armed_r   <= 1'b1;
            c_i_acc   <= '0;
            c_q_acc   <= '0;
            e1_acc    <= '0;
            e2_acc    <= '0;
            consec_cnt<= '0;
            det_r     <= 1'b0;
            for (int k = 0; k < DELAY_L; k++) begin
                prod_i_pipe[k] <= '0;
                prod_q_pipe[k] <= '0;
                e1_pipe[k]     <= '0;
                e2_pipe[k]     <= '0;
            end
        end else if (det_r) begin
            // Clear detection strobe after 1 cycle, disarm until next burst_start
            det_r   <= 1'b0;
            armed_r <= 1'b0;
        end else if (armed_r) begin
            if (!auto_mode_en && preamble_enable) begin
                // Mode B: External Trigger
                det_r   <= 1'b1;
                armed_r <= 1'b0;
            end else if (prod_vld_r) begin
                // Shift registers for sliding window
                prod_i_pipe[0] <= prod_i_r;
                prod_q_pipe[0] <= prod_q_r;
                e1_pipe[0]     <= inst_e1_r;
                e2_pipe[0]     <= inst_e2_r;
                for (int k = 1; k < DELAY_L; k++) begin
                    prod_i_pipe[k] <= prod_i_pipe[k-1];
                    prod_q_pipe[k] <= prod_q_pipe[k-1];
                    e1_pipe[k]     <= e1_pipe[k-1];
                    e2_pipe[k]     <= e2_pipe[k-1];
                end

                // Running accumulation: add incoming sample, subtract outgoing sample L taps ago
                c_i_acc <= c_i_acc + prod_i_r - prod_i_pipe[DELAY_L-1];
                c_q_acc <= c_q_acc + prod_q_r - prod_q_pipe[DELAY_L-1];
                e1_acc  <= e1_acc  + inst_e1_r - e1_pipe[DELAY_L-1];
                e2_acc  <= e2_acc  + inst_e2_r - e2_pipe[DELAY_L-1];
            end

            // Stage 4 Evaluation: Threshold comparison from pipelined registers
            if (auto_mode_en && pipe4_vld) begin
                if ((c_mag_sq_d1 >= thresh_cmp_r[31:0]) && (e1_acc_pipe4 >= energy_min_thresh)) begin
                    if (consec_cnt >= N_CONSEC_WIN - 1) begin
                        det_r      <= 1'b1;
                        consec_cnt <= '0;
                    end else begin
                        consec_cnt <= consec_cnt + 1'b1;
                    end
                end else begin
                    consec_cnt <= '0;
                end
            end
        end
    end

    assign preamble_detected = det_r;
    assign armed             = armed_r;
    assign det_metric_out    = {e1_acc[27:12], e2_acc[27:12]};

endmodule
