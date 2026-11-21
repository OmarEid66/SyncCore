//==============================================================================
// Module: coarse_cfo_estimator_top
// Description: Top-Level Preamble-Assisted Coarse Carrier Frequency Offset (CFO)
//              Estimator for Burst-Mode Baseband Wireless Receivers.
//
// Conforms to Project Specifications:
// - Preamble-assisted conjugate auto-correlation algorithm (L = 16 samples).
// - Synthesizable, modular, parameterized, FPGA-optimized architecture.
// - Capture range: +/- 0.10 * Rsym (+/- 3.125 MHz at 125 MHz clock).
// - Estimation accuracy: <= +/- 0.005 * Rsym over SNR 0 - 20 dB.
// - Latency: 57 symbols typ (well within 32 - 128 symbols specification).
// - Output: 24-bit signed Q1.23 normalized CFO and 32-bit downstream NCO FTW.
//==============================================================================

`timescale 1ns / 1ps

module coarse_cfo_estimator_top #(
    parameter int DATA_WIDTH    = 12,       // Input signed Q1.11 per channel
    parameter int DELAY_L       = 16,       // Preamble repetition delay (samples)
    parameter int ACCUM_M       = 192,      // Default accumulation length (samples)
    parameter int CORDIC_STAGES = 16,       // CORDIC vectoring iterations
    parameter int CFO_WIDTH     = 24,       // Output CFO width (Q1.23)
    parameter int NCO_WIDTH     = 32        // Downstream NCO phase accumulator width
)(
    input  logic                          clk,
    input  logic                          rst_n,

    // Streaming I/Q Interface from STR (4 samples/symbol)
    input  logic signed [DATA_WIDTH-1:0]  str_out_i,
    input  logic signed [DATA_WIDTH-1:0]  str_out_q,
    input  logic                          str_out_valid,

    // Control & Mode Select
    input  logic                          burst_start,      // Resets/arms the estimator
    input  logic                          preamble_enable,  // Mode B external trigger
    input  logic                          auto_mode_en,     // 1: Autonomous, 0: External

    // Programmable Detection Thresholds
    input  logic [15:0]                   gamma_scaled,     // Normalized metric threshold Q0.16 (e.g. 0.55 * 65536)
    input  logic [27:0]                   energy_min_thresh,// Minimum window energy threshold

    // Status & Observation (DFT)
    output logic                          preamble_detected,// 1-cycle pulse on preamble detection
    output logic [31:0]                   det_metric_out,   // Sliding detection energy metric
    output logic                          accum_busy,       // High while accumulator is active

    // CFO Output Register Interface
    output logic signed [CFO_WIDTH-1:0]   coarse_cfo_est,   // 24-bit signed Q1.23 normalized CFO
    output logic                          coarse_cfo_valid, // 1-cycle strobe when estimate is valid

    // Downstream NCO Interface
    output logic signed [NCO_WIDTH-1:0]   nco_ftw           // 32-bit NCO frequency tuning word
);

    // Internal Signal Declarations
    localparam int PROD_WIDTH  = 25;
    localparam int ACCUM_WIDTH = 34;

    // 1. Delay Line Signals
    logic signed [DATA_WIDTH-1:0] del_i;
    logic signed [DATA_WIDTH-1:0] del_q;
    logic                         del_valid;

    // 2. Conjugate Multiplier Signals
    logic signed [PROD_WIDTH-1:0] prod_i;
    logic signed [PROD_WIDTH-1:0] prod_q;
    logic                         prod_valid;

    // 3. Preamble Detector Signals
    logic                         det_pulse;
    logic                         det_armed;

    // 4. Accumulator Signals
    logic                         start_accum_pulse;
    logic signed [ACCUM_WIDTH-1:0] accum_out_i;
    logic signed [ACCUM_WIDTH-1:0] accum_out_q;
    logic                         accum_complete_pulse;
    logic                         acc_busy_sig;
    logic [$clog2(ACCUM_M):0]     sample_cnt;

    // 5. CORDIC Signals
    logic signed [CFO_WIDTH-1:0]  cordic_theta_out;
    logic                         cordic_valid_out;

    // Top-Level State Machine
    typedef enum logic [2:0] {
        STATE_IDLE       = 3'd0,
        STATE_SEARCHING  = 3'd1,
        STATE_ACCUM      = 3'd2,
        STATE_CORDIC     = 3'd3,
        STATE_DONE       = 3'd4
    } state_t;

    state_t state_r, state_next;

    //--------------------------------------------------------------------------
    // 1. Sample Delay Line (L = 16 samples)
    //--------------------------------------------------------------------------
    cfo_delay_line #(
        .DATA_WIDTH (DATA_WIDTH),
        .DELAY_L    (DELAY_L)
    ) u_delay_line (
        .clk           (clk),
        .rst_n         (rst_n),
        .clr           (burst_start),
        .in_i          (str_out_i),
        .in_q          (str_out_q),
        .in_valid      (str_out_valid),
        .del_out_i     (del_i),
        .del_out_q     (del_q),
        .del_out_valid (del_valid)
    );

    //--------------------------------------------------------------------------
    // 2. Complex Conjugate Multiplier
    //--------------------------------------------------------------------------
    cfo_conj_multiplier #(
        .DATA_WIDTH (DATA_WIDTH),
        .PROD_WIDTH (PROD_WIDTH)
    ) u_conj_mult (
        .clk        (clk),
        .rst_n      (rst_n),
        .in_i       (str_out_i),
        .in_q       (str_out_q),
        .del_i      (del_i),
        .del_q      (del_q),
        .in_valid   (del_valid),
        .prod_i     (prod_i),
        .prod_q     (prod_q),
        .prod_valid (prod_valid)
    );

    //--------------------------------------------------------------------------
    // 3. Preamble Detector (Dual-Mode: Autonomous + External Trigger)
    //--------------------------------------------------------------------------
    cfo_preamble_detector #(
        .DATA_WIDTH   (DATA_WIDTH),
        .PROD_WIDTH   (PROD_WIDTH),
        .DELAY_L      (DELAY_L),
        .N_CONSEC_WIN (2)
    ) u_detector (
        .clk               (clk),
        .rst_n             (rst_n),
        .burst_start       (burst_start),
        .preamble_enable   (preamble_enable),
        .auto_mode_en      (auto_mode_en),
        .gamma_scaled      (gamma_scaled),
        .energy_min_thresh (energy_min_thresh),
        .in_i              (str_out_i),
        .in_q              (str_out_q),
        .del_i             (del_i),
        .del_q             (del_q),
        .prod_i            (prod_i),
        .prod_q            (prod_q),
        .prod_valid        (prod_valid),
        .preamble_detected (det_pulse),
        .det_metric_out    (det_metric_out),
        .armed             (det_armed)
    );

    //--------------------------------------------------------------------------
    // 4. Correlation Accumulator (M samples)
    //--------------------------------------------------------------------------
    cfo_accumulator #(
        .PROD_WIDTH  (PROD_WIDTH),
        .ACCUM_WIDTH (ACCUM_WIDTH),
        .ACCUM_M     (ACCUM_M)
    ) u_accumulator (
        .clk              (clk),
        .rst_n            (rst_n),
        .start_accum      (start_accum_pulse),
        .abort            (burst_start && !preamble_enable && !det_pulse),
        .prod_i           (prod_i),
        .prod_q           (prod_q),
        .prod_valid       (prod_valid),
        .accum_out_i      (accum_out_i),
        .accum_out_q      (accum_out_q),
        .accum_complete   (accum_complete_pulse),
        .accum_busy       (acc_busy_sig),
        .sample_count_out (sample_cnt)
    );

    //--------------------------------------------------------------------------
    // 5. CORDIC Phase Estimator (atan2)
    //--------------------------------------------------------------------------
    cordic_atan2 #(
        .XY_WIDTH    (ACCUM_WIDTH),
        .PHASE_WIDTH (CFO_WIDTH),
        .NUM_STAGES  (CORDIC_STAGES)
    ) u_cordic (
        .clk       (clk),
        .rst_n     (rst_n),
        .x_in      (accum_out_i),
        .y_in      (accum_out_q),
        .valid_in  (accum_complete_pulse),
        .theta_out (cordic_theta_out),
        .valid_out (cordic_valid_out)
    );

    //--------------------------------------------------------------------------
    // 6. CFO Calculator & Output Register
    //--------------------------------------------------------------------------
    cfo_calculator #(
        .PHASE_WIDTH (CFO_WIDTH),
        .CFO_WIDTH   (CFO_WIDTH),
        .NCO_WIDTH   (NCO_WIDTH)
    ) u_cfo_calc (
        .clk              (clk),
        .rst_n            (rst_n),
        .theta_in         (cordic_theta_out),
        .valid_in         (cordic_valid_out),
        .coarse_cfo_est   (coarse_cfo_est),
        .coarse_cfo_valid (coarse_cfo_valid),
        .nco_ftw          (nco_ftw)
    );

    //--------------------------------------------------------------------------
    // 7. Top-Level Control FSM
    //--------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_r <= STATE_IDLE;
        end else if (burst_start && !preamble_enable && !det_pulse) begin
            state_r <= STATE_SEARCHING;
        end else begin
            state_r <= state_next;
        end
    end

    always_comb begin
        state_next        = state_r;
        start_accum_pulse = 1'b0;

        case (state_r)
            STATE_IDLE, STATE_SEARCHING: begin
                if (det_pulse || (!auto_mode_en && preamble_enable)) begin
                    state_next        = STATE_ACCUM;
                    start_accum_pulse = 1'b1;
                end
            end

            STATE_ACCUM: begin
                if (burst_start && !preamble_enable) begin
                    state_next = STATE_SEARCHING;
                end else if (accum_complete_pulse) begin
                    state_next = STATE_CORDIC;
                end
            end

            STATE_CORDIC: begin
                if (burst_start && !preamble_enable) begin
                    state_next = STATE_SEARCHING;
                end else if (cordic_valid_out) begin
                    state_next = STATE_DONE;
                end
            end

            STATE_DONE: begin
                if (burst_start) begin
                    if (preamble_enable || det_pulse) begin
                        state_next        = STATE_ACCUM;
                        start_accum_pulse = 1'b1;
                    end else begin
                        state_next = STATE_SEARCHING;
                    end
                end else if (det_pulse || (!auto_mode_en && preamble_enable)) begin
                    state_next        = STATE_ACCUM;
                    start_accum_pulse = 1'b1;
                end
            end

            default: state_next = STATE_IDLE;
        endcase
    end

    assign preamble_detected = det_pulse;
    assign accum_busy        = acc_busy_sig;

endmodule
