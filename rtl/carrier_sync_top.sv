//==============================================================================
// Module: carrier_sync_top
// Description: Integrated Carrier Synchronization Top-Level Module.
//              Combines:
//              1. Preamble-Assisted Coarse CFO Estimator (Auto-correlation + CORDIC)
//              2. 32-bit Coarse CFO Derotator (NCO + Quarter-wave LUT + Complex Rotator)
//              3. Decision-Directed Costas Loop (Fine CFO + Phase Tracking for QPSK/16-QAM)
//              4. 16-Channel Design-for-Testability (DFT) Observation & Force Infrastructure
//
// Conforms to SI CLASH / ERI Specifications:
// - Clock: 100 - 200 MHz (Typ 125 MHz).
// - Input: 12-bit signed Q1.11 oversampled baseband samples (4 sps or 1 sps).
// - Output: 16-bit signed Q1.15 carrier-synchronized symbols (1 sps).
// - Coarse Capture Range: +/- 10% Rsym (+/- 3.125 MHz at 125 MHz Fclk).
// - Fine Pull-in Range: +/- 1% to +/- 5% Rsym.
// - Lock acquisition time: < 200 symbols.
// - Steady-state residual phase error: <= 1.5 deg RMS.
// - Steady-state pipeline latency: 7 clock cycles (well within R.106 limit of 64 cycles).
//==============================================================================

`timescale 1ns / 1ps

module carrier_sync_top #(
    parameter int DATA_IN_WIDTH    = 12,                  // ADC/STR signed Q1.11 per channel
    parameter int DATA_OUT_WIDTH   = 16,                  // Costas loop signed Q1.15 per channel
    parameter int DELAY_L          = 16,                  // Preamble repetition delay (samples)
    parameter int ACCUM_M          = 192,                 // Preamble accumulation length (samples)
    parameter int CORDIC_STAGES    = 16,                  // CORDIC vectoring stages
    parameter int CFO_WIDTH        = 24,                  // Coarse CFO width (Q1.23)
    parameter int NCO_WIDTH        = 32,                  // 32-bit NCO accumulator width
    parameter [7:0] ACQ_GAIN       = 8'd255,              // Costas gain while unlocked (Bn ~ 0.05)
    parameter [15:0] T_LOCK        = 16'd6500,            // Costas lock threshold
    parameter [15:0] T_UNLOCK      = 16'd10000,           // Costas unlock threshold
    parameter        ROM_FILE      = "sine_quarter.mem"   // Sine ROM file name
)(
    input  logic                            clk,
    input  logic                            rst_n,

    // Streaming I/Q Input Interface (e.g. from STR or ADC)
    input  logic signed [DATA_IN_WIDTH-1:0] in_sample_i,
    input  logic signed [DATA_IN_WIDTH-1:0] in_sample_q,
    input  logic                            in_sample_valid,

    // Control & Frame Synchronization
    input  logic                            burst_start,        // Pulses high at burst arrival (re-arms estimator & loop)
    input  logic                            preamble_enable,    // External trigger for Mode B
    input  logic                            auto_mode_en,       // 1: Auto preamble detection (Mode A), 0: Mode B
    input  logic [1:0]                      mod_type,           // 00 = QPSK, 01 = 16-QAM
    input  logic [7:0]                      costas_loop_gain,   // Tracking gain (106..255 <=> Bn 0.02..0.05, typ 157)

    // Symbol Strobe Configuration (when input is 4 sps)
    input  logic                            input_is_4sps,      // 1: Input is 4 sps, 0: Input is already 1 sps
    input  logic                            sym_strobe_ext_en,  // 1: Use external sym_strobe_in, 0: Auto decimate
    input  logic                            sym_strobe_in,      // External symbol strobe pulse (at 4 sps)
    input  logic [1:0]                      strobe_phase,       // Strobe phase for internal decimation (0..3)

    // Coarse Preamble Detection Thresholds
    input  logic [15:0]                     gamma_scaled,       // Normalized metric threshold Q0.16 (e.g. 36045 = 0.55)
    input  logic [27:0]                     energy_min_thresh,  // Energy threshold (e.g. 1000000)

    // Design for Testability (DFT) & Manual Override Controls
    input  logic [3:0]                      dft_sel,            // 16-channel observation selector
    input  logic                            coarse_bypass,      // 1: Bypass coarse derotator
    input  logic                            coarse_force_en,    // 1: Force coarse FTW
    input  logic signed [NCO_WIDTH-1:0]     coarse_force_ftw,   // Forced coarse FTW value
    input  logic                            costas_bypass,      // 1: Bypass Costas derotator
    input  logic                            costas_freeze,      // 1: Freeze Costas loop integrator (open loop)
    input  logic                            costas_force_en,    // 1: Force Costas NCO frequency
    input  logic signed [31:0]              costas_force_freq,  // Forced Costas frequency
    input  logic [31:0]                     phase_adj,          // Optional 90-degree phase ambiguity adjust
    input  logic                            adj_valid,          // Phase adjust strobe
    input  logic                            loop_clear_ext,     // Manual Costas loop clear

    // Streaming Carrier-Synchronized Output (1 sample/symbol)
    output logic signed [DATA_OUT_WIDTH-1:0] out_sym_i,
    output logic signed [DATA_OUT_WIDTH-1:0] out_sym_q,
    output logic                             out_sym_valid,

    // Status & Frequency Estimates
    output logic signed [CFO_WIDTH-1:0]      coarse_cfo_est,     // Signed Q1.23 normalized coarse CFO
    output logic                             coarse_cfo_valid,   // Strobe when coarse CFO estimate is valid
    output logic signed [31:0]               fine_cfo_est,       // Signed Q0.32 fine CFO (turns/symbol)
    output logic                             lock_detect,        // Costas loop lock indicator
    output logic                             preamble_detected,  // Preamble auto-detector trigger pulse

    // DFT 32-Bit Observation Bus
    output logic [31:0]                      dft_obs_data
);

    //==========================================================================
    // 1. Coarse CFO Estimator
    //==========================================================================
    logic signed [NCO_WIDTH-1:0] coarse_nco_ftw;
    logic [31:0]                 coarse_det_metric;
    logic                        coarse_accum_busy;

    coarse_cfo_estimator_top #(
        .DATA_WIDTH    (DATA_IN_WIDTH),
        .DELAY_L       (DELAY_L),
        .ACCUM_M       (ACCUM_M),
        .CORDIC_STAGES (CORDIC_STAGES),
        .CFO_WIDTH     (CFO_WIDTH),
        .NCO_WIDTH     (NCO_WIDTH)
    ) u_coarse_cfo (
        .clk               (clk),
        .rst_n             (rst_n),
        .str_out_i         (in_sample_i),
        .str_out_q         (in_sample_q),
        .str_out_valid     (in_sample_valid),
        .burst_start       (burst_start),
        .preamble_enable   (preamble_enable),
        .auto_mode_en      (auto_mode_en),
        .gamma_scaled      (gamma_scaled),
        .energy_min_thresh (energy_min_thresh),
        .preamble_detected (preamble_detected),
        .det_metric_out    (coarse_det_metric),
        .accum_busy        (coarse_accum_busy),
        .coarse_cfo_est    (coarse_cfo_est),
        .coarse_cfo_valid  (coarse_cfo_valid),
        .nco_ftw           (coarse_nco_ftw)
    );

    //==========================================================================
    // 2. Coarse CFO Derotator
    //==========================================================================
    logic signed [DATA_OUT_WIDTH-1:0] derot_sample_i;
    logic signed [DATA_OUT_WIDTH-1:0] derot_sample_q;
    logic                             derot_sample_valid;
    logic [NCO_WIDTH-1:0]             coarse_dft_phase;
    logic signed [NCO_WIDTH-1:0]      coarse_active_ftw;
    logic                             derot_active;

    coarse_derotator #(
        .DATA_IN_WIDTH  (DATA_IN_WIDTH),
        .DATA_OUT_WIDTH (DATA_OUT_WIDTH),
        .NCO_WIDTH      (NCO_WIDTH),
        .ROM_FILE       (ROM_FILE)
    ) u_derotator (
        .clk              (clk),
        .rst_n            (rst_n),
        .clear            (burst_start),
        .ftw_valid        (coarse_cfo_valid),
        .ftw_in           (coarse_nco_ftw),
        .force_en         (coarse_force_en),
        .force_ftw        (coarse_force_ftw),
        .bypass           (coarse_bypass),
        .sample_i_in      (in_sample_i),
        .sample_q_in      (in_sample_q),
        .sample_valid_in  (in_sample_valid),
        .sample_i_out     (derot_sample_i),
        .sample_q_out     (derot_sample_q),
        .sample_valid_out (derot_sample_valid),
        .dft_phase_out    (coarse_dft_phase),
        .dft_active_ftw   (coarse_active_ftw),
        .derot_active     (derot_active)
    );

    //==========================================================================
    // 3. Symbol Decimation & Strobe Alignment
    //    coarse_derotator has 3 cycles latency.
    //==========================================================================
    // Delay external strobe by 3 cycles to match derotator latency
    logic [2:0] ext_strobe_pipe;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ext_strobe_pipe <= 3'b0;
        end else begin
            ext_strobe_pipe <= {ext_strobe_pipe[1:0], (sym_strobe_in && in_sample_valid)};
        end
    end
    wire aligned_ext_strobe = ext_strobe_pipe[2];

    // Internal 4 sps counter aligned to derot_sample_valid
    logic [1:0] dec_cnt;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dec_cnt <= 2'd0;
        end else if (burst_start) begin
            dec_cnt <= 2'd0;
        end else if (derot_sample_valid) begin
            dec_cnt <= dec_cnt + 2'd1;
        end
    end
    wire auto_sym_strobe = (dec_cnt == strobe_phase);

    // Selected symbol strobe
    wire sym_strobe_active = sym_strobe_ext_en ? aligned_ext_strobe : auto_sym_strobe;

    // Costas loop x_valid:
    // If input is already 1 sps, every valid sample is a symbol.
    // If input is 4 sps, only the strobe cycle is valid into Costas loop.
    wire costas_in_valid = input_is_4sps ? (derot_sample_valid && sym_strobe_active)
                                         : derot_sample_valid;

    //==========================================================================
    // 4. Costas Loop Clear & Arm Coordination
    //==========================================================================
    // Hold Costas loop cleared during coarse estimation unless bypassed/forced
    logic wait_for_coarse;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wait_for_coarse <= 1'b0;
        end else if (burst_start) begin
            // If coarse CFO is active, wait for estimate before starting Costas loop
            wait_for_coarse <= !coarse_bypass && !coarse_force_en;
        end else if (coarse_cfo_valid || coarse_force_en) begin
            wait_for_coarse <= 1'b0;
        end
    end

    wire costas_loop_clear = burst_start || wait_for_coarse || loop_clear_ext;

    //==========================================================================
    // 5. Decision-Directed Costas Loop Top
    //==========================================================================
    logic signed [DATA_OUT_WIDTH-1:0] costas_out_i;
    logic signed [DATA_OUT_WIDTH-1:0] costas_out_q;
    logic                             costas_out_valid;
    logic signed [15:0]               dft_costas_phase_err;
    logic [31:0]                      dft_costas_nco_phase;
    logic signed [31:0]               dft_costas_nco_freq;
    logic [15:0]                      dft_costas_lock_metric;

    costas_loop_top #(
        .ACQ_GAIN (ACQ_GAIN),
        .T_LOCK   (T_LOCK),
        .T_UNLOCK (T_UNLOCK),
        .ROM_FILE (ROM_FILE)
    ) u_costas_loop (
        .clk              (clk),
        .rst_n            (rst_n),
        .loop_clear       (costas_loop_clear),
        .x_i              (derot_sample_i),
        .x_q              (derot_sample_q),
        .x_valid          (costas_in_valid),
        .mod_type         (mod_type),
        .costas_loop_gain (costas_loop_gain),
        .force_en         (costas_force_en),
        .force_freq       (costas_force_freq),
        .freeze           (costas_freeze),
        .bypass           (costas_bypass),
        .phase_adj        (phase_adj),
        .adj_valid        (adj_valid),
        .y_i              (costas_out_i),
        .y_q              (costas_out_q),
        .y_valid          (costas_out_valid),
        .fine_cfo_est     (fine_cfo_est),
        .lock_detect      (lock_detect),
        .dft_phase_err    (dft_costas_phase_err),
        .dft_nco_phase    (dft_costas_nco_phase),
        .dft_nco_freq     (dft_costas_nco_freq),
        .dft_lock_metric  (dft_costas_lock_metric)
    );

    //==========================================================================
    // 6. Outputs & Observation MUX (Checkpoint 5)
    //==========================================================================
    assign out_sym_i     = costas_out_i;
    assign out_sym_q     = costas_out_q;
    assign out_sym_valid = costas_out_valid;

    always_comb begin
        case (dft_sel)
            4'h0: dft_obs_data = fine_cfo_est;                                       // Fine CFO estimate
            4'h1: dft_obs_data = {{16{dft_costas_phase_err[15]}}, dft_costas_phase_err}; // Costas phase error
            4'h2: dft_obs_data = dft_costas_nco_phase;                                // Costas NCO phase
            4'h3: dft_obs_data = dft_costas_nco_freq;                                 // Costas NCO integrator
            4'h4: dft_obs_data = {16'd0, dft_costas_lock_metric};                     // Costas lock metric
            4'h5: dft_obs_data = {{8{coarse_cfo_est[23]}}, coarse_cfo_est};          // Coarse CFO est Q1.23
            4'h6: dft_obs_data = coarse_nco_ftw;                                      // Coarse NCO FTW
            4'h7: dft_obs_data = coarse_det_metric;                                   // Coarse metric
            4'h8: dft_obs_data = coarse_dft_phase;                                    // Coarse derotator phase
            4'h9: dft_obs_data = {costas_out_i, costas_out_q};                        // Costas outputs
            4'hA: dft_obs_data = {derot_sample_i, derot_sample_q};                    // Derotator outputs
            4'hB: dft_obs_data = {{16{in_sample_i[DATA_IN_WIDTH-1]}}, in_sample_i, 4'd0}; // Scaled input I
            4'hC: dft_obs_data = {{16{in_sample_q[DATA_IN_WIDTH-1]}}, in_sample_q, 4'd0}; // Scaled input Q
            4'hD: dft_obs_data = {27'd0, derot_active, lock_detect, coarse_cfo_valid, preamble_detected, coarse_accum_busy};
            4'hE: dft_obs_data = {16'd0, 14'd0, mod_type};                           // Modulation config
            4'hF: dft_obs_data = 32'h53594E43;                                        // ID "SYNC" (ASCII)
            default: dft_obs_data = 32'h0;
        endcase
    end

endmodule
