//==============================================================================
// Module: sync_subsystem_top
// Description: Full End-to-End Synchronization Subsystem Top-Level.
//
//   Integrates:
//     1. Symbol Timing Recovery (STR)   -- Farrow + Gardner TED + PI loop
//     2. Coarse CFO Estimator           -- Preamble auto-correlation + CORDIC
//     3. Coarse CFO Derotator           -- 32-bit NCO + quarter-wave LUT
//     4. Costas Loop                    -- Decision-directed fine CFO + phase
//
//   Signal Chain:
//     ADC samples (12-bit Q1.11, 4 sps)
//       ---> str_top          : Farrow interpolator + Gardner TED
//       ---> (parallel) coarse_cfo_estimator_top  (runs on raw 4-sps)
//       ---> coarse_derotator : rotates STR 1-sps symbols
//       ---> costas_loop_top  : fine CFO + phase tracking
//       ---> out_sym (16-bit Q1.15, 1 sps)
//
//   Port widths and formats:
//     in_sample_i/q   : 12-bit signed Q1.11, 4 sps input
//     out_sym_i/q     : 16-bit signed Q1.15, 1 sps output
//
//   Target FPGA : AMD/Xilinx Artix-7 XC7A35T
//   Clock       : 100 – 200 MHz (typ 125 MHz)
//==============================================================================

`timescale 1ns / 1ps

module sync_subsystem_top #(
    // -------------------------------------------------------------------------
    // STR parameters
    // -------------------------------------------------------------------------
    parameter logic signed [17:0] STR_KP4_FX   = 18'sd10344, // Gardner PI proportional gain (4*Kp * 2^16)
    parameter logic signed [17:0] STR_KI_FX    = 18'sd883,   // Gardner PI integral gain (Ki * 2^21)

    // -------------------------------------------------------------------------
    // Coarse CFO estimator parameters
    // -------------------------------------------------------------------------
    parameter int DELAY_L          = 16,          // Preamble repetition delay (samples)
    parameter int ACCUM_M          = 192,         // Preamble accumulation length (samples)
    parameter int CORDIC_STAGES    = 16,          // CORDIC vectoring stages
    parameter int CFO_WIDTH        = 24,          // Coarse CFO output width (Q1.23)
    parameter int NCO_WIDTH        = 32,          // NCO phase accumulator width

    // -------------------------------------------------------------------------
    // Costas loop parameters
    // -------------------------------------------------------------------------
    parameter [7:0]  ACQ_GAIN  = 8'd255,          // Costas gain during acquisition
    parameter [15:0] T_LOCK    = 16'd6500,         // Costas lock threshold
    parameter [15:0] T_UNLOCK  = 16'd10000,        // Costas unlock threshold

    // -------------------------------------------------------------------------
    // ROM file for NCO sine table (shared by derotator and Costas loop)
    // -------------------------------------------------------------------------
    parameter        ROM_FILE  = "sine_quarter.mem"
)(
    input  logic               clk,
    input  logic               rst_n,

    // =========================================================================
    // Streaming I/Q Input (from ADC / front-end filter)
    // =========================================================================
    input  logic signed [11:0] in_sample_i,      // Q1.11 raw sample, I channel
    input  logic signed [11:0] in_sample_q,      // Q1.11 raw sample, Q channel
    input  logic               in_sample_valid,  // 1 when input sample is valid (4 sps rate)

    // =========================================================================
    // Burst / Frame Control
    // =========================================================================
    input  logic               burst_start,      // 1-cycle pulse: re-arms all loops at burst start
    input  logic               preamble_enable,  // Mode B external preamble trigger
    input  logic               auto_mode_en,     // 1=autonomous preamble detect, 0=external Mode B

    // =========================================================================
    // Coarse CFO Estimator Thresholds
    // =========================================================================
    input  logic [15:0]        gamma_scaled,     // Detection metric threshold Q0.16 (e.g. 0x8CB2 = 0.55)
    input  logic [27:0]        energy_min_thresh,// Minimum energy gate for detection

    // =========================================================================
    // Costas Loop Control
    // =========================================================================
    input  logic [1:0]         mod_type,         // 2'b00=QPSK, 2'b01=16-QAM
    input  logic [7:0]         costas_loop_gain, // Runtime Bn knob (106..255)

    // =========================================================================
    // Design-for-Testability (DFT) Observation MUX
    // =========================================================================
    input  logic [3:0]         dft_sel,          // 16-channel observation selector

    // =========================================================================
    // Manual Override / Force Registers for Block Isolation (Checkpoint 5)
    // =========================================================================
    // --- STR Block Isolation Controls ---
    input  logic               str_bypass,       // 1: Bypass STR (direct 4-sps decimation)
    input  logic               str_force_en,     // 1: Force manual fractional delay mu
    input  logic [11:0]        str_force_mu,     // Forced fractional delay value

    // --- Coarse CFO & Derotator Isolation Controls ---
    input  logic               coarse_bypass,    // 1: Bypass coarse derotator (passthrough)
    input  logic               coarse_force_en,  // 1: Use coarse_force_ftw instead of estimated FTW
    input  logic signed [NCO_WIDTH-1:0] coarse_force_ftw, // Forced coarse FTW

    // --- Costas Loop Isolation Controls ---
    input  logic               costas_bypass,    // 1: Bypass Costas derotator
    input  logic               costas_freeze,    // 1: Freeze Costas integrator (open-loop observe)
    input  logic               costas_force_en,  // 1: Force Costas NCO frequency
    input  logic signed [31:0] costas_force_freq,// Forced Costas frequency word
    input  logic [31:0]        phase_adj,        // 90-degree phase ambiguity correction
    input  logic               adj_valid,        // Phase adjust strobe
    input  logic               loop_clear_ext,   // Manual Costas clear

    // =========================================================================
    // Streaming Output (carrier-synchronized symbols)
    // =========================================================================
    output logic signed [15:0] out_sym_i,        // Q1.15 recovered symbol, I
    output logic signed [15:0] out_sym_q,        // Q1.15 recovered symbol, Q
    output logic               out_sym_valid,    // 1 sps symbol strobe

    // =========================================================================
    // Status & Estimates
    // =========================================================================
    output logic signed [CFO_WIDTH-1:0] coarse_cfo_est,   // Q1.23 normalized coarse CFO
    output logic                        coarse_cfo_valid,  // Strobe when estimate is ready
    output logic signed [31:0]          fine_cfo_est,      // Q0.32 fine CFO (turns/symbol)
    output logic                        lock_detect,       // Costas lock indicator
    output logic                        preamble_detected, // Preamble auto-detection pulse

    // =========================================================================
    // DFT 32-Bit Observation Bus
    // =========================================================================
    output logic [31:0]                 dft_obs_data
);

    // =========================================================================
    // Internal Wires
    // =========================================================================

    // --- STR outputs (1 sps, Q1.15) ---
    logic signed [15:0] str_sym_i, str_sym_q;
    logic               str_sym_valid;
    logic signed [17:0] dft_str_ted_err;
    logic [11:0]        dft_str_mu;
    logic [21:0]        dft_str_eta;

    // STR bypass decimation counter (for standalone block isolation)
    logic [1:0] bypass_dec_cnt;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bypass_dec_cnt <= 2'b00;
        end else if (burst_start) begin
            bypass_dec_cnt <= 2'b00;
        end else if (in_sample_valid) begin
            bypass_dec_cnt <= bypass_dec_cnt + 2'd1;
        end
    end
    wire bypass_sym_strobe = in_sample_valid && (bypass_dec_cnt == 2'b00);

    // Selected STR output: normal STR recovered symbols or bypassed raw samples
    wire signed [15:0] active_str_sym_i = str_bypass ? {in_sample_i, 4'b0000} : str_sym_i;
    wire signed [15:0] active_str_sym_q = str_bypass ? {in_sample_q, 4'b0000} : str_sym_q;
    wire               active_str_valid = str_bypass ? bypass_sym_strobe       : str_sym_valid;

    // --- Coarse CFO estimator outputs ---
    logic signed [NCO_WIDTH-1:0] coarse_nco_ftw;
    logic [31:0]                 coarse_det_metric;
    logic                        coarse_accum_busy;

    // --- Coarse derotator outputs ---
    logic signed [15:0] derot_sym_i, derot_sym_q;
    logic               derot_sym_valid;
    logic [NCO_WIDTH-1:0]        coarse_dft_phase;
    logic signed [NCO_WIDTH-1:0] coarse_active_ftw;
    logic                        derot_active;

    // --- Costas outputs (DFT probes) ---
    logic signed [15:0] dft_costas_phase_err;
    logic [31:0]        dft_costas_nco_phase;
    logic signed [31:0] dft_costas_nco_freq;
    logic [15:0]        dft_costas_lock_metric;

    // =========================================================================
    // 1. Symbol Timing Recovery (STR)
    //    Input  : 12-bit Q1.11 @ 4 sps
    //    Output : 16-bit Q1.15 @ 1 sps
    // =========================================================================
    str_top #(
        .KP4_FX (STR_KP4_FX),
        .KI_FX  (STR_KI_FX)
    ) u_str (
        .clk             (clk),
        .rst_n           (rst_n),
        .in_sample_i     (in_sample_i),
        .in_sample_q     (in_sample_q),
        .in_sample_valid (in_sample_valid),
        .dft_force_en    (str_force_en),
        .dft_force_mu    (str_force_mu),
        .out_sym_i       (str_sym_i),
        .out_sym_q       (str_sym_q),
        .out_sym_valid   (str_sym_valid),
        .dft_ted_err     (dft_str_ted_err),
        .dft_mu          (dft_str_mu),
        .dft_eta         (dft_str_eta)
    );

    // =========================================================================
    // 2. Coarse CFO Estimator
    //    Runs on the RAW 4-sps input (not STR output) to observe the full
    //    preamble structure at 4 samples/symbol for correlation.
    //    Input  : 12-bit Q1.11 @ 4 sps (same bus as STR)
    // =========================================================================
    coarse_cfo_estimator_top #(
        .DATA_WIDTH    (12),
        .DELAY_L       (DELAY_L),
        .ACCUM_M       (ACCUM_M),
        .CORDIC_STAGES (CORDIC_STAGES),
        .CFO_WIDTH     (CFO_WIDTH),
        .NCO_WIDTH     (NCO_WIDTH)
    ) u_coarse_cfo (
        .clk               (clk),
        .rst_n             (rst_n),
        .str_out_i         (in_sample_i),     // Raw 4-sps
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

    // =========================================================================
    // 3. Coarse CFO Derotator
    //    Applies the estimated coarse FTW to the STR 1-sps output symbols.
    //    DATA_IN_WIDTH=16 since STR output is already Q1.15.
    //    Note on FTW scaling: coarse_nco_ftw is phase step per 4-sps sample.
    //    Since derotator is clocked at 1 sps (str_sym_valid), the phase step
    //    per symbol is 4x larger: (coarse_nco_ftw <<< 2).
    // =========================================================================
    wire signed [NCO_WIDTH-1:0] derot_ftw_in = coarse_nco_ftw <<< 2;
    wire signed [NCO_WIDTH-1:0] derot_force_ftw = coarse_force_ftw;

    coarse_derotator #(
        .DATA_IN_WIDTH  (16),
        .DATA_OUT_WIDTH (16),
        .NCO_WIDTH      (NCO_WIDTH),
        .ROM_FILE       (ROM_FILE)
    ) u_derotator (
        .clk              (clk),
        .rst_n            (rst_n),
        .clear            (burst_start),
        .ftw_valid        (coarse_cfo_valid),
        .ftw_in           (derot_ftw_in),
        .force_en         (coarse_force_en),
        .force_ftw        (derot_force_ftw),
        .bypass           (coarse_bypass),
        .sample_i_in      (active_str_sym_i),  // 1 sps from STR (or bypass)
        .sample_q_in      (active_str_sym_q),
        .sample_valid_in  (active_str_valid),
        .sample_i_out     (derot_sym_i),
        .sample_q_out     (derot_sym_q),
        .sample_valid_out (derot_sym_valid),
        .dft_phase_out    (coarse_dft_phase),
        .dft_active_ftw   (coarse_active_ftw),
        .derot_active     (derot_active)
    );

    // =========================================================================
    // 4. Costas Loop Clear Coordination
    //    Hold clear until coarse estimate is ready (unless bypassed).
    // =========================================================================
    logic wait_for_coarse;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wait_for_coarse <= 1'b0;
        end else if (burst_start) begin
            wait_for_coarse <= !coarse_bypass && !coarse_force_en;
        end else if (coarse_cfo_valid || coarse_force_en) begin
            wait_for_coarse <= 1'b0;
        end
    end

    wire costas_loop_clear = burst_start || wait_for_coarse || loop_clear_ext;

    // =========================================================================
    // 5. Costas Loop (Decision-Directed Fine CFO + Phase Tracking)
    //    Input is 1 sps from derotator (already timing-corrected by STR).
    // =========================================================================
    costas_loop_top #(
        .ACQ_GAIN (ACQ_GAIN),
        .T_LOCK   (T_LOCK),
        .T_UNLOCK (T_UNLOCK),
        .ROM_FILE (ROM_FILE)
    ) u_costas_loop (
        .clk              (clk),
        .rst_n            (rst_n),
        .loop_clear       (costas_loop_clear),
        .x_i              (derot_sym_i),
        .x_q              (derot_sym_q),
        .x_valid          (derot_sym_valid),   // 1 sps — every valid is a symbol
        .mod_type         (mod_type),
        .costas_loop_gain (costas_loop_gain),
        .force_en         (costas_force_en),
        .force_freq       (costas_force_freq),
        .freeze           (costas_freeze),
        .bypass           (costas_bypass),
        .phase_adj        (phase_adj),
        .adj_valid        (adj_valid),
        .y_i              (out_sym_i),
        .y_q              (out_sym_q),
        .y_valid          (out_sym_valid),
        .fine_cfo_est     (fine_cfo_est),
        .lock_detect      (lock_detect),
        .dft_phase_err    (dft_costas_phase_err),
        .dft_nco_phase    (dft_costas_nco_phase),
        .dft_nco_freq     (dft_costas_nco_freq),
        .dft_lock_metric  (dft_costas_lock_metric)
    );

    // =========================================================================
    // 6. DFT Observation MUX (16 channels) — Checkpoint 5 Specification
    //    Channels 0-4 : Costas Loop fine tracking signals
    //    Channels 5-8 : Coarse CFO & Derotator status & NCO phase
    //    Channels 9-B : STR internal status (TED error, STR NCO phase/mu, symbols)
    //    Channels C-F : Constellation, Status bitfield, Control, ASCII ID
    // =========================================================================
    always_comb begin
        case (dft_sel)
            4'h0: dft_obs_data = fine_cfo_est;
            4'h1: dft_obs_data = {{16{dft_costas_phase_err[15]}}, dft_costas_phase_err};
            4'h2: dft_obs_data = dft_costas_nco_phase;                                // Fine NCO phase
            4'h3: dft_obs_data = dft_costas_nco_freq;
            4'h4: dft_obs_data = {16'd0, dft_costas_lock_metric};
            4'h5: dft_obs_data = {{8{coarse_cfo_est[23]}}, coarse_cfo_est};
            4'h6: dft_obs_data = coarse_active_ftw;                                  // Active Coarse Derotator FTW
            4'h7: dft_obs_data = coarse_det_metric;
            4'h8: dft_obs_data = coarse_dft_phase;                                    // Coarse Derotator NCO phase
            4'h9: dft_obs_data = {{14{dft_str_ted_err[17]}}, dft_str_ted_err};       // Gardner TED error
            4'hA: dft_obs_data = {10'd0, dft_str_eta};                                // STR NCO phase (eta UQ0.22)
            4'hB: dft_obs_data = {str_sym_i, str_sym_q};                             // STR output recovered symbols
            4'hC: dft_obs_data = {out_sym_i, out_sym_q};                             // Final synchronized symbols
            4'hD: dft_obs_data = {24'd0, str_bypass, coarse_bypass, costas_bypass,
                                  derot_active, lock_detect, coarse_cfo_valid, preamble_detected, coarse_accum_busy};
            4'hE: dft_obs_data = {4'd0, dft_str_mu, 14'd0, mod_type};                // STR fractional delay (mu) & mod
            4'hF: dft_obs_data = 32'h55535452;                                        // ID "USTR" (ASCII)
            default: dft_obs_data = 32'h0;
        endcase
    end

endmodule
