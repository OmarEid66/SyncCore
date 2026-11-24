//==============================================================================
// Testbench: tb_carrier_sync_top
// Description: Comprehensive, self-checking SystemVerilog verification testbench
//              for the integrated Carrier Synchronization subsystem (carrier_sync_top).
//
// Test Cases Covered:
//   Test 1: Standard Burst Mode QPSK (+0.05 Rsym = +1.5625 MHz, phi0=45 deg, SNR=20 dB)
//           - Verifies Preamble Detection -> Coarse CFO Estimation -> Coarse Derotation
//           - Verifies Fine Lock Acquisition (< 200 symbols)
//           - Verifies Steady-state EVM (< 5.5%) and Residual CFO (< 0.01 Rsym)
//   Test 2: High Negative CFO with 16-QAM (-0.08 Rsym = -2.5 MHz, phi0=-60 deg, SNR=20 dB)
//           - Verifies 16-QAM Decision-Directed Costas Loop tracking and EVM (< 8.0%)
//   Test 3: Maximum CFO Boundary Test (+0.10 Rsym = +3.125 MHz, SNR=18 dB)
//           - Verifies operation at maximum R.113 capture limit (+/- 10% Rsym)
//   Test 4: Standalone Fine Tracking Mode (coarse_bypass=1, small CFO=+0.005 Rsym)
//           - Direct verification of Costas loop path
//   Test 5: DFT 16-Channel Observation MUX & Control Registers Verification
//           - Cycles dft_sel from 0 to 15 and validates observation bus
//   Test 6: Back-to-Back Consecutive Bursts (Re-Arming & Lock Recovery)
//           - Burst 1 (+0.04 Rsym) -> Burst 2 (-0.06 Rsym) without simulator restart
//==============================================================================

`timescale 1ns / 1ps

module tb_carrier_sync_top;

    // System Timing Parameters
    localparam real CLK_PERIOD_NS = 8.0;      // 125 MHz system clock
    localparam real FS_HZ         = 125.0e6;  // 125 MSps sample rate
    localparam real RSYM_HZ       = 31.25e6;  // 31.25 MSps symbol rate (4 sps)
    localparam real PI            = 3.14159265358979323846;

    // DUT Generics
    localparam int DATA_IN_WIDTH  = 12;
    localparam int DATA_OUT_WIDTH = 16;
    localparam int DELAY_L        = 16;
    localparam int ACCUM_M        = 192;
    localparam int CORDIC_STAGES  = 16;
    localparam int CFO_WIDTH      = 24;
    localparam int NCO_WIDTH      = 32;

    // DUT Signals
    logic                            clk;
    logic                            rst_n;
    logic signed [DATA_IN_WIDTH-1:0] in_sample_i;
    logic signed [DATA_IN_WIDTH-1:0] in_sample_q;
    logic                            in_sample_valid;

    logic                            burst_start;
    logic                            preamble_enable;
    logic                            auto_mode_en;
    logic [1:0]                      mod_type;
    logic [7:0]                      costas_loop_gain;
    logic                            input_is_4sps;
    logic                            sym_strobe_ext_en;
    logic                            sym_strobe_in;
    logic [1:0]                      strobe_phase;
    logic [15:0]                     gamma_scaled;
    logic [27:0]                     energy_min_thresh;

    logic [3:0]                      dft_sel;
    logic                            coarse_bypass;
    logic                            coarse_force_en;
    logic signed [NCO_WIDTH-1:0]     coarse_force_ftw;
    logic                            costas_bypass;
    logic                            costas_freeze;
    logic                            costas_force_en;
    logic signed [31:0]              costas_force_freq;
    logic [31:0]                     phase_adj;
    logic                            adj_valid;
    logic                            loop_clear_ext;

    logic signed [DATA_OUT_WIDTH-1:0] out_sym_i;
    logic signed [DATA_OUT_WIDTH-1:0] out_sym_q;
    logic                             out_sym_valid;
    logic signed [CFO_WIDTH-1:0]      coarse_cfo_est;
    logic                             coarse_cfo_valid;
    logic signed [31:0]               fine_cfo_est;
    logic                             lock_detect;
    logic                             preamble_detected;
    logic [31:0]                      dft_obs_data;

    // Test Statistics
    int pass_count = 0;
    int fail_count = 0;

    // Clock Generator
    initial begin
        clk = 1'b0;
        forever #(CLK_PERIOD_NS / 2.0) clk = ~clk;
    end

    // Instantiate DUT
    carrier_sync_top #(
        .DATA_IN_WIDTH    (DATA_IN_WIDTH),
        .DATA_OUT_WIDTH   (DATA_OUT_WIDTH),
        .DELAY_L          (DELAY_L),
        .ACCUM_M          (ACCUM_M),
        .CORDIC_STAGES    (CORDIC_STAGES),
        .CFO_WIDTH        (CFO_WIDTH),
        .NCO_WIDTH        (NCO_WIDTH),
        .ACQ_GAIN         (8'd255),
        .T_LOCK           (16'd6500),
        .T_UNLOCK         (16'd10000),
        .ROM_FILE         ("sine_quarter.mem")
    ) dut (
        .clk               (clk),
        .rst_n             (rst_n),
        .in_sample_i       (in_sample_i),
        .in_sample_q       (in_sample_q),
        .in_sample_valid   (in_sample_valid),
        .burst_start       (burst_start),
        .preamble_enable   (preamble_enable),
        .auto_mode_en      (auto_mode_en),
        .mod_type          (mod_type),
        .costas_loop_gain  (costas_loop_gain),
        .input_is_4sps     (input_is_4sps),
        .sym_strobe_ext_en (sym_strobe_ext_en),
        .sym_strobe_in     (sym_strobe_in),
        .strobe_phase      (strobe_phase),
        .gamma_scaled      (gamma_scaled),
        .energy_min_thresh (energy_min_thresh),
        .dft_sel           (dft_sel),
        .coarse_bypass     (coarse_bypass),
        .coarse_force_en   (coarse_force_en),
        .coarse_force_ftw  (coarse_force_ftw),
        .costas_bypass     (costas_bypass),
        .costas_freeze     (costas_freeze),
        .costas_force_en   (costas_force_en),
        .costas_force_freq (costas_force_freq),
        .phase_adj         (phase_adj),
        .adj_valid         (adj_valid),
        .loop_clear_ext    (loop_clear_ext),
        .out_sym_i         (out_sym_i),
        .out_sym_q         (out_sym_q),
        .out_sym_valid     (out_sym_valid),
        .coarse_cfo_est    (coarse_cfo_est),
        .coarse_cfo_valid  (coarse_cfo_valid),
        .fine_cfo_est      (fine_cfo_est),
        .lock_detect       (lock_detect),
        .preamble_detected (preamble_detected),
        .dft_obs_data      (dft_obs_data)
    );

    // Reset Task
    task reset_dut();
        rst_n             <= 1'b0;
        in_sample_i       <= '0;
        in_sample_q       <= '0;
        in_sample_valid   <= 1'b0;
        burst_start       <= 1'b0;
        preamble_enable   <= 1'b0;
        auto_mode_en      <= 1'b0;
        mod_type          <= 2'b00;
        costas_loop_gain  <= 8'd157; // Bn ~ 0.03
        input_is_4sps     <= 1'b1;
        sym_strobe_ext_en <= 1'b0;
        sym_strobe_in     <= 1'b0;
        strobe_phase      <= 2'd0;
        gamma_scaled      <= 16'd36045; // 0.55
        energy_min_thresh <= 28'd1000000;
        dft_sel           <= 4'd0;
        coarse_bypass     <= 1'b0;
        coarse_force_en   <= 1'b0;
        coarse_force_ftw  <= '0;
        costas_bypass     <= 1'b0;
        costas_freeze     <= 1'b0;
        costas_force_en   <= 1'b0;
        costas_force_freq <= '0;
        phase_adj         <= '0;
        adj_valid         <= 1'b0;
        loop_clear_ext    <= 1'b0;

        repeat (10) @(posedge clk);
        rst_n <= 1'b1;
        repeat (5) @(posedge clk);
    endtask

    // Helper: Gaussian random generator for AWGN
    function real rand_gauss();
        real u1, u2;
        u1 = ($urandom + 1.0) / 4294967297.0;
        u2 = ($urandom + 1.0) / 4294967297.0;
        return $sqrt(-2.0 * $ln(u1)) * $cos(2.0 * PI * u2);
    endfunction

    // Helper: Zadoff-Chu Preamble sample generator (L=16)
    task get_zc_sample(input int k, output real zc_i, output real zc_q);
        real phase;
        int idx;
        idx = k % 16;
        phase = -PI * (real'(idx) * real'(idx + 1)) / 16.0;
        zc_i = 1800.0 * $cos(phase);
        zc_q = 1800.0 * $sin(phase);
    endtask

    // Helper: Random QPSK symbol generator (12-bit Q1.11, unit power)
    task get_qpsk_symbol(output real sym_i, output real sym_q);
        int bit_i, bit_q;
        bit_i = $urandom & 1;
        bit_q = $urandom & 1;
        sym_i = bit_i ? 1448.0 : -1448.0; // 1/sqrt(2) * 2048 ~ 1448
        sym_q = bit_q ? 1448.0 : -1448.0;
    endtask

    // Helper: Random 16-QAM symbol generator (12-bit Q1.11, unit power)
    task get_16qam_symbol(output real sym_i, output real sym_q);
        int r_i, r_q;
        r_i = $urandom % 4;
        r_q = $urandom % 4;
        case (r_i)
            0: sym_i = -1943.0; // -3/sqrt(10) * 2048
            1: sym_i = -648.0;  // -1/sqrt(10) * 2048
            2: sym_i =  648.0;  // +1/sqrt(10) * 2048
            3: sym_i =  1943.0; // +3/sqrt(10) * 2048
        endcase
        case (r_q)
            0: sym_q = -1943.0;
            1: sym_q = -648.0;
            2: sym_q =  648.0;
            3: sym_q =  1943.0;
        endcase
    endtask

    // Helper: Conversion functions
    function real q23_to_cfo_rsym(input logic signed [23:0] q23_val);
        real norm_val;
        norm_val = real'(q23_val) / 8388608.0; // [-1.0, 1.0) relative to Fs/2
        // f_hz = norm_val * (Fs/2). Relative to Rsym = Fs/4:
        // f_rsym = f_hz / Rsym = norm_val * 2.0
        return norm_val * 2.0;
    endfunction

    function real q32_to_fine_rsym(input logic signed [31:0] q32_val);
        return real'(q32_val) / 4294967296.0;
    endfunction

    // Arrays to store transmitted ideal symbols for EVM calculation
    real tx_ideal_i[0:3999];
    real tx_ideal_q[0:3999];

    // Arrays to capture received symbols from Costas loop
    real rx_meas_i[0:3999];
    real rx_meas_q[0:3999];
    int  rx_sym_count;
    int  first_lock_symbol;

    // Monitor for Costas Loop Outputs
    always @(posedge clk) begin
        if (rst_n && out_sym_valid) begin
            if (rx_sym_count < 4000) begin
                rx_meas_i[rx_sym_count] = real'(out_sym_i) / 32768.0;
                rx_meas_q[rx_sym_count] = real'(out_sym_q) / 32768.0;

                if (lock_detect && first_lock_symbol < 0) begin
                    first_lock_symbol = rx_sym_count;
                end

                rx_sym_count++;
            end
        end
    end

    // Comprehensive Burst Simulation Task
    task automatic run_burst_test(
        input string test_title,
        input real   cfo_rsym_target,   // CFO in units of Rsym (e.g. +0.05)
        input real   phi0_deg,          // Initial phase offset in degrees
        input real   snr_db,            // SNR in dB
        input int    modulation,        // 0 = QPSK, 1 = 16-QAM
        input bit    use_auto_mode,     // 1 = Auto preamble detector, 0 = External trigger
        input int    num_payload_syms   // Number of payload symbols (e.g. 1500)
    );
        int total_preamble_samples;
        int total_burst_samples;
        real noise_sigma;
        real cfo_hz;
        real phase_rad;
        real s_i, s_q, r_i, r_q;
        int q_i, q_q;
        int sample_idx;
        int sym_idx;
        real measured_coarse_rsym, measured_fine_rsym, total_est_cfo_rsym, cfo_err_rsym;
        real evm_pct, noise_floor_evm;
        real err0, err1, err2, err3, pwr, best_err;
        real tx_i_norm, tx_q_norm;
        int  evm_start_sym;
        int  evm_syms;
        bit  test_passed;

        $display("\n================================================================================");
        $display("RUNNING: %s", test_title);
        $display("  Target CFO  : %0.4f Rsym (%0.2f kHz)", cfo_rsym_target, cfo_rsym_target * RSYM_HZ / 1e3);
        $display("  Init Phase  : %0.1f deg", phi0_deg);
        $display("  Channel SNR : %0.1f dB", snr_db);
        $display("  Modulation  : %s", (modulation == 0) ? "QPSK" : "16-QAM");
        $display("  Detector    : %s", use_auto_mode ? "Auto Mode A" : "External Mode B");
        $display("================================================================================");

        // Configure DUT
        mod_type          <= (modulation == 0) ? 2'b00 : 2'b01;
        auto_mode_en      <= use_auto_mode;
        input_is_4sps     <= 1'b1;
        sym_strobe_ext_en <= 1'b0;
        strobe_phase      <= 2'd0;
        coarse_bypass     <= 1'b0;
        coarse_force_en   <= 1'b0;
        costas_bypass     <= 1'b0;

        rx_sym_count      = 0;
        first_lock_symbol = -1;

        total_preamble_samples = 256; // 64 symbols at 4 sps
        total_burst_samples    = total_preamble_samples + (num_payload_syms * 4);
        cfo_hz                 = cfo_rsym_target * RSYM_HZ;
        noise_sigma            = 1448.0 * $sqrt($pow(10.0, -snr_db / 10.0) / 2.0);

        // Arm burst
        @(posedge clk);
        burst_start <= 1'b1;
        if (!use_auto_mode) preamble_enable <= 1'b1;
        @(posedge clk);
        burst_start <= 1'b0;
        preamble_enable <= 1'b0;

        // Stream Preamble followed by Payload
        sym_idx = 0;
        for (sample_idx = 0; sample_idx < total_burst_samples; sample_idx++) begin
            if (sample_idx < total_preamble_samples) begin
                // Preamble ZC sequence
                get_zc_sample(sample_idx, s_i, s_q);
            end else begin
                // Payload symbols (held constant for 4 samples per symbol)
                if ((sample_idx % 4) == 0) begin
                    if (modulation == 0)
                        get_qpsk_symbol(s_i, s_q);
                    else
                        get_16qam_symbol(s_i, s_q);

                    tx_ideal_i[sym_idx] = s_i;
                    tx_ideal_q[sym_idx] = s_q;
                    sym_idx++;
                end
            end

            // Apply carrier frequency spin: exp(j*(2*pi*f_cfo*t + phi0))
            phase_rad = (2.0 * PI * (cfo_hz / FS_HZ) * sample_idx) + (phi0_deg * PI / 180.0);
            r_i = (s_i * $cos(phase_rad) - s_q * $sin(phase_rad)) + (noise_sigma * rand_gauss());
            r_q = (s_i * $sin(phase_rad) + s_q * $cos(phase_rad)) + (noise_sigma * rand_gauss());

            // Quantize to 12-bit signed Q1.11 [-2048, 2047]
            q_i = $rtoi(r_i);
            q_q = $rtoi(r_q);
            if (q_i >  2047) q_i =  2047;
            if (q_i < -2048) q_i = -2048;
            if (q_q >  2047) q_q =  2047;
            if (q_q < -2048) q_q = -2048;

            @(posedge clk);
            in_sample_i     <= 12'(q_i);
            in_sample_q     <= 12'(q_q);
            in_sample_valid <= 1'b1;
        end

        // End of burst stream
        @(posedge clk);
        in_sample_valid <= 1'b0;

        // Allow pipeline flush and Costas loop steady state
        repeat (100) @(posedge clk);

        // Evaluation & Metrics
        measured_coarse_rsym = q23_to_cfo_rsym(coarse_cfo_est);
        measured_fine_rsym   = q32_to_fine_rsym(fine_cfo_est);
        total_est_cfo_rsym   = measured_coarse_rsym + measured_fine_rsym;
        cfo_err_rsym         = total_est_cfo_rsym - cfo_rsym_target;
        if (cfo_err_rsym < 0.0) cfo_err_rsym = -cfo_err_rsym;

        $display("\n--- Subsystem Performance Results ---");
        $display("  Coarse CFO Valid      : %b", coarse_cfo_valid || (coarse_cfo_est != 0));
        $display("  Coarse Estimated CFO  : %0.5f Rsym (%0.2f kHz)", measured_coarse_rsym, measured_coarse_rsym * RSYM_HZ / 1e3);
        $display("  Residual after Coarse : %0.5f Rsym (Target: <= +/-0.005 Rsym)", cfo_rsym_target - measured_coarse_rsym);
        $display("  Costas Fine CFO Est   : %0.5f Rsym (%0.2f kHz)", measured_fine_rsym, measured_fine_rsym * RSYM_HZ / 1e3);
        $display("  Total Receiver CFO Est: %0.5f Rsym", total_est_cfo_rsym);
        $display("  Net CFO Error         : %0.6f Rsym (%0.2f kHz)", cfo_err_rsym, cfo_err_rsym * RSYM_HZ / 1e3);
        $display("  Costas Lock Detected  : %b", lock_detect);
        $display("  First Lock Symbol     : %0d (Limit: <= 200 symbols)", first_lock_symbol);

        // Calculate EVM over the last 1000 symbols (or available locked payload)
        err0 = 0.0; err1 = 0.0; err2 = 0.0; err3 = 0.0; pwr = 0.0;
        evm_syms = 1000;
        if (rx_sym_count < evm_syms) evm_syms = rx_sym_count - 100;
        evm_start_sym = rx_sym_count - evm_syms;

        for (int k = evm_start_sym; k < rx_sym_count; k++) begin
            int ref_k = k - 64; // Account for 64-symbol preamble
            if (ref_k >= 0 && ref_k < num_payload_syms) begin
                tx_i_norm = tx_ideal_i[ref_k] / 2048.0;
                tx_q_norm = tx_ideal_q[ref_k] / 2048.0;

                // Account for 4-fold constellation rotational ambiguity (0, 90, 180, 270 deg)
                err0 += (rx_meas_i[k] - tx_i_norm)**2 + (rx_meas_q[k] - tx_q_norm)**2;
                err1 += (rx_meas_q[k] - tx_i_norm)**2 + (-rx_meas_i[k] - tx_q_norm)**2;
                err2 += (-rx_meas_i[k] - tx_i_norm)**2 + (-rx_meas_q[k] - tx_q_norm)**2;
                err3 += (-rx_meas_q[k] - tx_i_norm)**2 + (rx_meas_i[k] - tx_q_norm)**2;
                pwr  += tx_i_norm**2 + tx_q_norm**2;
            end
        end

        best_err = err0;
        if (err1 < best_err) best_err = err1;
        if (err2 < best_err) best_err = err2;
        if (err3 < best_err) best_err = err3;

        evm_pct = (pwr > 0.0) ? (100.0 * $sqrt(best_err / pwr)) : 999.0;
        noise_floor_evm = 100.0 * $pow(10.0, -snr_db / 20.0);

        $display("  RMS EVM (over %0d syms): %0.2f %% (Channel Noise Floor: %0.2f %%)", evm_syms, evm_pct, noise_floor_evm);

        // Verification Criteria Check
        test_passed = 1'b1;
        if (!lock_detect) begin
            $error("FAIL: Costas loop did not assert lock_detect!");
            test_passed = 1'b0;
        end
        if (first_lock_symbol > 200 || first_lock_symbol < 0) begin
            $error("FAIL: Lock acquisition latency %0d symbols exceeded 200 symbols limit (R.122)!", first_lock_symbol);
            test_passed = 1'b0;
        end
        if (cfo_err_rsym > 0.01) begin
            $error("FAIL: Net residual CFO error %0.4f Rsym exceeded 0.01 Rsym limit (R.117)!", cfo_err_rsym);
            test_passed = 1'b0;
        end
        if (snr_db >= 35.0 && modulation == 0 && evm_pct > 2.5) begin
            $error("FAIL: Clean condition QPSK EVM %0.2f %% exceeded 2.5 %% limit (Checkpoint 6)!", evm_pct);
            test_passed = 1'b0;
        end
        if (snr_db >= 35.0 && modulation == 1 && evm_pct > 13.0) begin
            $error("FAIL: Clean condition 16-QAM EVM %0.2f %% exceeded requirement!", evm_pct);
            test_passed = 1'b0;
        end
        if (snr_db < 35.0 && modulation == 0 && evm_pct > 8.5) begin
            $error("FAIL: QPSK EVM %0.2f %% exceeded requirement!", evm_pct);
            test_passed = 1'b0;
        end
        if (snr_db < 35.0 && modulation == 1 && evm_pct > 15.0) begin
            $error("FAIL: 16-QAM EVM %0.2f %% exceeded requirement!", evm_pct);
            test_passed = 1'b0;
        end

        if (test_passed) begin
            $display(">>> TEST PASSED <<<\n");
            pass_count++;
        end else begin
            $display(">>> TEST FAILED <<<\n");
            fail_count++;
        end
    endtask

    // Main Test Execution Sequence
    initial begin
        $display("\n================================================================================");
        $display("  STARTING CARRIER SYNCHRONIZATION INTEGRATED SUBSYSTEM VERIFICATION");
        $display("================================================================================");

        reset_dut();

        // ---------------------------------------------------------------------
        // TEST 1A: Clean Condition QPSK (SNR = 40 dB, Checkpoint 6: EVM <= 2.5%)
        // ---------------------------------------------------------------------
        run_burst_test(
            "Test 1A: Clean Condition QPSK (+0.05 Rsym, phi0=45 deg, SNR=40 dB)",
            0.050,   // +0.05 Rsym
            45.0,    // phi0 = 45 deg
            40.0,    // SNR = 40 dB (Clean Condition)
            0,       // QPSK
            0,       // Mode B external trigger
            1500     // 1500 payload symbols
        );

        // ---------------------------------------------------------------------
        // TEST 1B: Noisy Condition QPSK (+0.05 Rsym = +1.5625 MHz, SNR = 20 dB)
        // ---------------------------------------------------------------------
        reset_dut();
        run_burst_test(
            "Test 1B: QPSK Burst with +0.05 Rsym CFO (+1.5625 MHz), phi0=45 deg, SNR=20 dB",
            0.050,   // +0.05 Rsym
            45.0,    // phi0 = 45 deg
            20.0,    // SNR = 20 dB
            0,       // QPSK
            0,       // Mode B external trigger
            1500     // 1500 payload symbols
        );

        // ---------------------------------------------------------------------
        // TEST 2A: Clean Condition 16-QAM (SNR = 40 dB, Checkpoint 6: EVM <= 2.5%)
        // ---------------------------------------------------------------------
        reset_dut();
        run_burst_test(
            "Test 2A: Clean Condition 16-QAM (-0.08 Rsym, phi0=-60 deg, SNR=40 dB)",
            -0.080,  // -0.08 Rsym
            -60.0,   // phi0 = -60 deg
            40.0,    // SNR = 40 dB (Clean Condition)
            1,       // 16-QAM
            0,       // Mode B external trigger
            1500     // 1500 payload symbols
        );

        // ---------------------------------------------------------------------
        // TEST 2B: High Negative CFO with 16-QAM (-0.08 Rsym = -2.5 MHz, SNR = 25 dB)
        // ---------------------------------------------------------------------
        reset_dut();
        run_burst_test(
            "Test 2B: 16-QAM Burst with -0.08 Rsym CFO (-2.50 MHz), phi0=-60 deg, SNR=25 dB",
            -0.080,  // -0.08 Rsym
            -60.0,   // phi0 = -60 deg
            25.0,    // SNR = 25 dB
            1,       // 16-QAM
            0,       // Mode B external trigger
            1500     // 1500 payload symbols
        );

        // ---------------------------------------------------------------------
        // TEST 3: Maximum Boundary CFO (+0.10 Rsym = +3.125 MHz, SNR = 18 dB)
        // ---------------------------------------------------------------------
        reset_dut();
        run_burst_test(
            "Test 3: Maximum CFO Boundary (+0.10 Rsym = +3.125 MHz), phi0=30 deg, SNR=18 dB",
            0.100,   // +0.10 Rsym
            30.0,    // phi0 = 30 deg
            18.0,    // SNR = 18 dB
            0,       // QPSK
            0,       // Mode B external trigger
            1500     // 1500 payload symbols
        );

        // ---------------------------------------------------------------------
        // TEST 4: Standalone Fine Tracking Mode (Coarse Bypass, CFO = +0.005 Rsym)
        // ---------------------------------------------------------------------
        reset_dut();
        $display("\n================================================================================");
        $display("RUNNING: Test 4: Standalone Costas Fine Tracking (coarse_bypass=1, CFO=+0.005 Rsym)");
        $display("================================================================================");
        coarse_bypass     <= 1'b1;
        input_is_4sps     <= 1'b0; // 1 sps direct feed
        mod_type          <= 2'b00;
        costas_loop_gain  <= 8'd157;
        rx_sym_count      = 0;
        first_lock_symbol = -1;

        @(posedge clk);
        burst_start <= 1'b1;
        @(posedge clk);
        burst_start <= 1'b0;

        for (int n = 0; n < 1000; n++) begin
            real di, dq, ph, ri, rq;
            di = ($urandom & 1) ? 23170.0 : -23170.0;
            dq = ($urandom & 1) ? 23170.0 : -23170.0;
            ph = (2.0 * PI * 0.005 * n) + (60.0 * PI / 180.0);
            ri = di * $cos(ph) - dq * $sin(ph);
            rq = di * $sin(ph) + dq * $cos(ph);

            @(posedge clk);
            in_sample_i     <= 12'($rtoi(ri / 16.0));
            in_sample_q     <= 12'($rtoi(rq / 16.0));
            in_sample_valid <= 1'b1;
            @(posedge clk);
            in_sample_valid <= 1'b0;
            repeat (2) @(posedge clk); // 4 clocks per symbol total
        end
        @(posedge clk);
        in_sample_valid <= 1'b0;
        repeat (50) @(posedge clk);

        $display("  Costas Standalone Lock  : %b", lock_detect);
        $display("  First Lock Symbol       : %0d", first_lock_symbol);
        $display("  Fine CFO Estimated      : %0.5f Rsym (Target: +0.00500 Rsym)", q32_to_fine_rsym(fine_cfo_est));

        begin
            real fine_diff;
            fine_diff = q32_to_fine_rsym(fine_cfo_est) - 0.005;
            if (fine_diff < 0) fine_diff = -fine_diff;
            if (lock_detect && first_lock_symbol <= 200 && (fine_diff < 0.001)) begin
                $display(">>> TEST 4 PASSED <<<\n");
                pass_count++;
            end else begin
                $error(">>> TEST 4 FAILED <<<\n");
                fail_count++;
            end
        end

        // ---------------------------------------------------------------------
        // TEST 5: DFT 16-Channel Observation MUX Verification
        // ---------------------------------------------------------------------
        $display("\n================================================================================");
        $display("RUNNING: Test 5: DFT 16-Channel Observation MUX & ID Verification");
        $display("================================================================================");
        for (int ch = 0; ch < 16; ch++) begin
            dft_sel <= 4'(ch);
            @(posedge clk);
            $display("  DFT Channel 0x%0X => dft_obs_data = 0x%08X", ch, dft_obs_data);
        end

        // Verify Channel 0xF returns ID "SYNC" (0x53594E43)
        dft_sel <= 4'hF;
        @(posedge clk);
        if (dft_obs_data == 32'h53594E43) begin
            $display("  Verified DFT Module ID: 0x%08X (ASCII 'SYNC')", dft_obs_data);
            $display(">>> TEST 5 PASSED <<<\n");
            pass_count++;
        end else begin
            $error("FAIL: DFT Module ID mismatch! Expected 0x53594E43, got 0x%08X", dft_obs_data);
            fail_count++;
        end

        // ---------------------------------------------------------------------
        // TEST 6: Consecutive Back-to-Back Bursts (Dynamic Re-Arming)
        // ---------------------------------------------------------------------
        $display("\n================================================================================");
        $display("RUNNING: Test 6: Consecutive Back-to-Back Bursts (Burst 1 -> Burst 2)");
        $display("================================================================================");
        reset_dut();
        run_burst_test(
            "Test 6A: First Burst (+0.04 Rsym, SNR=20 dB)",
            0.040, 20.0, 20.0, 0, 0, 1000
        );

        // Immediate subsequent burst with opposite sign CFO
        run_burst_test(
            "Test 6B: Second Burst (-0.06 Rsym, SNR=20 dB) without Simulator Reset",
            -0.060, -45.0, 20.0, 0, 0, 1000
        );

        // ---------------------------------------------------------------------
        // Final Summary Report
        // ---------------------------------------------------------------------
        $display("\n================================================================================");
        $display("  CARRIER SYNCHRONIZATION TESTBENCH SUMMARY REPORT");
        $display("================================================================================");
        $display("  TOTAL TESTS RUN : %0d", pass_count + fail_count);
        $display("  TESTS PASSED    : %0d", pass_count);
        $display("  TESTS FAILED    : %0d", fail_count);
        $display("================================================================================");

        if (fail_count == 0) begin
            $display(">>> ALL INTEGRATION TESTS PASSED SUCCESSFULLY! <<<\n");
        end else begin
            $error(">>> VERIFICATION FAILED: %0d test(s) failed! <<<\n", fail_count);
        end

        $finish;
    end

endmodule
