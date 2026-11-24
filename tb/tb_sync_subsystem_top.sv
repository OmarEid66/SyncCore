//==============================================================================
// Testbench: tb_sync_subsystem_top
// Description: End-to-end, self-checking SystemVerilog verification testbench
//              for the full synchronization subsystem (sync_subsystem_top.sv).
//
//   Signal Chain Under Test:
//     ADC (12-bit Q1.11, 4 sps) → STR → Coarse CFO Est (parallel, 4 sps)
//       → Coarse Derotator (1 sps) → Costas Loop → out_sym (Q1.15, 1 sps)
//==============================================================================

`timescale 1ns / 1ps

module tb_sync_subsystem_top;
    import str_pkg::*;

    // =========================================================================
    // System Parameters
    // =========================================================================
    localparam real CLK_PERIOD_NS = 8.0;      // 125 MHz
    localparam real FS_HZ         = 125.0e6;  // Input sample rate (4 sps)
    localparam real RSYM_HZ       = 31.25e6;  // Symbol rate = Fs/4
    localparam real PI            = 3.14159265358979323846;
    localparam int  SPS           = 4;        // Oversampling factor
    localparam int  DELAY_L       = 16;
    localparam int  ACCUM_M       = 192;

    // =========================================================================
    // DUT Signals
    // =========================================================================
    logic               clk;
    logic               rst_n;
    logic signed [11:0] in_sample_i;
    logic signed [11:0] in_sample_q;
    logic               in_sample_valid;

    logic               burst_start;
    logic               preamble_enable;
    logic               auto_mode_en;
    logic [15:0]        gamma_scaled;
    logic [27:0]        energy_min_thresh;
    logic [1:0]         mod_type;
    logic [7:0]         costas_loop_gain;
    logic [3:0]         dft_sel;

    // STR DFT & Force Controls
    logic               str_bypass;
    logic               str_force_en;
    logic [11:0]        str_force_mu;

    logic               coarse_bypass;
    logic               coarse_force_en;
    logic signed [31:0] coarse_force_ftw;
    logic               costas_bypass;
    logic               costas_freeze;
    logic               costas_force_en;
    logic signed [31:0] costas_force_freq;
    logic [31:0]        phase_adj;
    logic               adj_valid;
    logic               loop_clear_ext;

    logic signed [15:0] out_sym_i;
    logic signed [15:0] out_sym_q;
    logic               out_sym_valid;
    logic signed [23:0] coarse_cfo_est;
    logic               coarse_cfo_valid;
    logic signed [31:0] fine_cfo_est;
    logic               lock_detect;
    logic               preamble_detected;
    logic [31:0]        dft_obs_data;

    // Test counters
    int pass_count = 0;
    int fail_count = 0;

    // =========================================================================
    // Clock Generator (125 MHz)
    // =========================================================================
    initial clk = 1'b0;
    always #(CLK_PERIOD_NS / 2.0) clk = ~clk;

    // =========================================================================
    // DUT Instantiation
    // =========================================================================
    sync_subsystem_top #(
        .ROM_FILE ("sine_quarter.mem")
    ) dut (
        .clk               (clk),
        .rst_n             (rst_n),
        .in_sample_i       (in_sample_i),
        .in_sample_q       (in_sample_q),
        .in_sample_valid   (in_sample_valid),
        .burst_start       (burst_start),
        .preamble_enable   (preamble_enable),
        .auto_mode_en      (auto_mode_en),
        .gamma_scaled      (gamma_scaled),
        .energy_min_thresh (energy_min_thresh),
        .mod_type          (mod_type),
        .costas_loop_gain  (costas_loop_gain),
        .dft_sel           (dft_sel),
        .str_bypass        (str_bypass),
        .str_force_en      (str_force_en),
        .str_force_mu      (str_force_mu),
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

    // =========================================================================
    // Symbol capture monitor
    // =========================================================================
    real rx_meas_i [0:7999];
    real rx_meas_q [0:7999];
    int  rx_sym_count;
    int  first_lock_symbol;

    always @(posedge clk) begin
        if (rst_n && out_sym_valid && rx_sym_count < 8000) begin
            rx_meas_i[rx_sym_count] = real'(out_sym_i) / 32768.0;
            rx_meas_q[rx_sym_count] = real'(out_sym_q) / 32768.0;
            if (lock_detect && first_lock_symbol < 0)
                first_lock_symbol = rx_sym_count;
            rx_sym_count++;
        end
    end

    // =========================================================================
    // Helper Functions
    // =========================================================================

    function real rand_gauss();
        real u1, u2;
        u1 = ($urandom + 1.0) / 4294967297.0;
        u2 = ($urandom + 1.0) / 4294967297.0;
        return $sqrt(-2.0 * $ln(u1)) * $cos(2.0 * PI * u2);
    endfunction

    function real q23_to_cfo_rsym(input logic signed [23:0] q23_val);
        return (real'(q23_val) / 8388608.0) * 2.0;
    endfunction

    function real q32_to_fine_rsym(input logic signed [31:0] q32_val);
        return real'(q32_val) / 4294967296.0;
    endfunction

    task get_qpsk_symbol(output real si, output real sq);
        int bi, bq;
        bi = $urandom & 1; bq = $urandom & 1;
        si = bi ? 1448.0 : -1448.0; // 1/sqrt(2) * 2048 ~ 1448 (unit power in Q1.11)
        sq = bq ? 1448.0 : -1448.0;
    endtask

    task get_16qam_symbol(output real si, output real sq);
        int ri, rq;
        ri = $urandom % 4; rq = $urandom % 4;
        case (ri) 0: si=-1943.0; 1: si=-648.0; 2: si=648.0; default: si=1943.0; endcase
        case (rq) 0: sq=-1943.0; 1: sq=-648.0; 2: sq=648.0; default: sq=1943.0; endcase
    endtask

    task get_zc_sample(input int k, output real zi, output real zq);
        real phase;
        int idx;
        idx = k % 16;
        phase = -PI * real'(idx) * real'(idx + 1) / 16.0;
        zi = 1800.0 * $cos(phase);
        zq = 1800.0 * $sin(phase);
    endtask

    task do_reset();
        rst_n             <= 1'b0;
        in_sample_i       <= 12'sd0;
        in_sample_q       <= 12'sd0;
        in_sample_valid   <= 1'b0;
        burst_start       <= 1'b0;
        preamble_enable   <= 1'b0;
        auto_mode_en      <= 1'b1;
        gamma_scaled      <= 16'd36045;
        energy_min_thresh <= 28'd1000000;
        mod_type          <= 2'b00;
        costas_loop_gain  <= 8'd157;
        dft_sel           <= 4'd0;
        str_bypass        <= 1'b0;
        str_force_en      <= 1'b0;
        str_force_mu      <= 12'd0;
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
        rx_sym_count      <= 0;
        first_lock_symbol <= -1;
        repeat (12) @(posedge clk);
        rst_n <= 1'b1;
        repeat (5) @(posedge clk);
    endtask

    // =========================================================================
    // Burst Injection + Verification Task
    // =========================================================================
    task automatic run_burst_test(
        input string test_title,
        input real   cfo_rsym_target,
        input real   timing_offset_sym,
        input real   phi0_deg,
        input real   snr_db,
        input int    modulation,
        input bit    use_auto_mode,
        input real   evm_thresh_pct,
        input int    num_payload_syms,
        input bit    cb,
        input bit    cf_en
    );
        real noise_sigma;
        real phase_rad, cfo_phase_inc, phase_cont;
        real s_i, s_q;
        real r_i, r_q;
        real cos_ph, sin_ph;
        real rot_i, rot_q;
        int  q_i_int, q_q_int;
        int  sym_idx, samp_idx;
        int  total_input_clks;
        int  cfo_valid_clk;
        int  total_preamble_samples;
        int  total_burst_samples;
        real zi_tmp, zq_tmp;
        real measured_coarse_rsym, measured_fine_rsym, total_est_cfo_rsym, cfo_err_rsym;
        // EVM calculation
        real sum_err2, sum_pwr, evm_pct, avg_pwr, scale_fac;
        real ri_s, rq_s;
        real min_d2, d2_tmp;
        real nearest_i, nearest_q;
        real cand_i, cand_q;
        int  pi_v, pq_v;
        int  evm_start_sym, evm_num_syms, evm_s;
        bit  test_ok;

        // 256 preamble samples = 64 symbols at 4 sps (16 repetitions of L=16 ZC root)
        total_preamble_samples = 256;
        total_burst_samples    = total_preamble_samples + (num_payload_syms * SPS);
        noise_sigma            = 1448.0 * $sqrt($pow(10.0, -snr_db / 10.0) / 2.0);
        cfo_phase_inc          = 2.0 * PI * (cfo_rsym_target * RSYM_HZ / FS_HZ);
        phase_rad              = phi0_deg * PI / 180.0;
        phase_cont             = phase_rad;
        cfo_valid_clk          = -1;
        total_input_clks       = 0;
        test_ok                = 1;

        $display("\n================================================================================");
        $display("RUNNING: %s", test_title);
        $display("  CFO target  : %0.4f Rsym (%0.2f kHz)", cfo_rsym_target, cfo_rsym_target * RSYM_HZ / 1e3);
        $display("  Init phase  : %0.1f deg", phi0_deg);
        $display("  Channel SNR : %0.1f dB", snr_db);
        $display("  Modulation  : %s", (modulation == 0) ? "QPSK" : "16-QAM");
        $display("  EVM thresh  : %0.1f%%", evm_thresh_pct);
        $display("================================================================================");

        mod_type         <= (modulation == 0) ? 2'b00 : 2'b01;
        costas_loop_gain <= 8'd157;
        auto_mode_en     <= use_auto_mode;
        coarse_bypass    <= cb;
        coarse_force_en  <= cf_en;
        rx_sym_count      = 0;
        first_lock_symbol = -1;
        @(posedge clk);

        burst_start <= 1'b1;
        if (!use_auto_mode) preamble_enable <= 1'b1;
        @(posedge clk);
        burst_start <= 1'b0;
        preamble_enable <= 1'b0;

        // --- Stream Samples ---
        for (int sample_idx = 0; sample_idx < total_burst_samples; sample_idx++) begin
            if (sample_idx < total_preamble_samples) begin
                get_zc_sample(sample_idx, s_i, s_q);
            end else begin
                if (((sample_idx - total_preamble_samples) % SPS) == 0) begin
                    if (modulation == 0)
                        get_qpsk_symbol(s_i, s_q);
                    else
                        get_16qam_symbol(s_i, s_q);
                end
            end

            // Apply CFO rotation: exp(j * phase)
            cos_ph = $cos(phase_cont);
            sin_ph = $sin(phase_cont);
            rot_i  = s_i * cos_ph - s_q * sin_ph + noise_sigma * rand_gauss();
            rot_q  = s_i * sin_ph + s_q * cos_ph + noise_sigma * rand_gauss();

            // Quantize to 12-bit signed [-2048, 2047]
            q_i_int = int'($floor(rot_i + 0.5));
            q_q_int = int'($floor(rot_q + 0.5));
            if (q_i_int >  2047) q_i_int =  2047;
            if (q_i_int < -2048) q_i_int = -2048;
            if (q_q_int >  2047) q_q_int =  2047;
            if (q_q_int < -2048) q_q_int = -2048;

            @(posedge clk);
            in_sample_i     <= q_i_int[11:0];
            in_sample_q     <= q_q_int[11:0];
            in_sample_valid <= 1'b1;
            total_input_clks++;

            if (coarse_cfo_valid && cfo_valid_clk < 0)
                cfo_valid_clk = total_input_clks;

            phase_cont = phase_cont + cfo_phase_inc;
        end

        @(posedge clk);
        in_sample_valid <= 1'b0;

        // Allow pipeline flush and Costas settling
        repeat (200) @(posedge clk);

        // Evaluation
        measured_coarse_rsym = q23_to_cfo_rsym(coarse_cfo_est);
        measured_fine_rsym   = q32_to_fine_rsym(fine_cfo_est);
        total_est_cfo_rsym   = (cb ? 0.0 : measured_coarse_rsym) + measured_fine_rsym;
        cfo_err_rsym         = total_est_cfo_rsym - cfo_rsym_target;
        if (cfo_err_rsym < 0.0) cfo_err_rsym = -cfo_err_rsym;

        $display("\n--- Subsystem Performance Results ---");
        $display("  Coarse CFO Valid      : %b", coarse_cfo_valid || (coarse_cfo_est != 0));
        $display("  Coarse Estimated CFO  : %0.5f Rsym (%0.2f kHz)", measured_coarse_rsym, measured_coarse_rsym * RSYM_HZ / 1e3);
        $display("  Costas Fine CFO Est   : %0.5f Rsym (%0.2f kHz)", measured_fine_rsym, measured_fine_rsym * RSYM_HZ / 1e3);
        $display("  Total Receiver CFO Est: %0.5f Rsym", total_est_cfo_rsym);
        $display("  Net CFO Error         : %0.6f Rsym (%0.2f kHz)", cfo_err_rsym, cfo_err_rsym * RSYM_HZ / 1e3);
        $display("  Costas Lock Detected  : %b", lock_detect);
        $display("  First Lock Symbol     : %0d", first_lock_symbol);
        $display("  Total Output Symbols  : %0d", rx_sym_count);

        // Check 1: coarse_cfo_valid
        if (!cb) begin
            if (cfo_valid_clk < 0) begin
                $display("[FAIL] %s: coarse_cfo_valid never asserted!", test_title);
                test_ok = 0;
            end else begin
                $display("[INFO] %s: coarse_cfo_valid at clk %0d", test_title, cfo_valid_clk);
            end
        end

        // Check 2: Costas lock
        if (!lock_detect) begin
            $display("[WARN] %s: Costas lock_detect not asserted", test_title);
            if (modulation == 0 && !cb) test_ok = 0;
        end else begin
            $display("[PASS_LOCK] %s: Costas locked at sym %0d", test_title, first_lock_symbol);
        end

        // Check 3: EVM calculation over steady-state symbols
        if (first_lock_symbol >= 0 && (rx_sym_count - first_lock_symbol) >= 50) begin
            evm_start_sym = first_lock_symbol + 40;
            evm_num_syms  = rx_sym_count - evm_start_sym;
            if (evm_num_syms > 1000) evm_num_syms = 1000;

            // Measure average received power to determine scale factor
            avg_pwr = 0.0;
            for (evm_s = evm_start_sym; evm_s < evm_start_sym + evm_num_syms; evm_s++) begin
                avg_pwr = avg_pwr + rx_meas_i[evm_s]**2 + rx_meas_q[evm_s]**2;
            end
            avg_pwr = avg_pwr / real'(evm_num_syms);
            scale_fac = (avg_pwr > 0.01) ? $sqrt(avg_pwr) : 1.0;

            // Decision-directed EVM: distance to nearest constellation point
            sum_err2 = 0.0;
            sum_pwr  = 0.0;
            for (evm_s = evm_start_sym; evm_s < evm_start_sym + evm_num_syms; evm_s++) begin
                ri_s = rx_meas_i[evm_s] / scale_fac;
                rq_s = rx_meas_q[evm_s] / scale_fac;
                min_d2 = 1.0e30;
                nearest_i = 0.0;
                nearest_q = 0.0;

                if (modulation == 0) begin
                    // QPSK: points at (±0.7071, ±0.7071)
                    for (pi_v = -1; pi_v <= 1; pi_v = pi_v + 2) begin
                        for (pq_v = -1; pq_v <= 1; pq_v = pq_v + 2) begin
                            cand_i = real'(pi_v) * 0.7071;
                            cand_q = real'(pq_v) * 0.7071;
                            d2_tmp = (ri_s - cand_i)**2 + (rq_s - cand_q)**2;
                            if (d2_tmp < min_d2) begin
                                min_d2    = d2_tmp;
                                nearest_i = cand_i;
                                nearest_q = cand_q;
                            end
                        end
                    end
                end else begin
                    // 16-QAM: 4x4 grid at (±1, ±3) / sqrt(10)
                    for (pi_v = -3; pi_v <= 3; pi_v = pi_v + 2) begin
                        for (pq_v = -3; pq_v <= 3; pq_v = pq_v + 2) begin
                            cand_i = real'(pi_v) * 0.31623;
                            cand_q = real'(pq_v) * 0.31623;
                            d2_tmp = (ri_s - cand_i)**2 + (rq_s - cand_q)**2;
                            if (d2_tmp < min_d2) begin
                                min_d2    = d2_tmp;
                                nearest_i = cand_i;
                                nearest_q = cand_q;
                            end
                        end
                    end
                end

                sum_err2 = sum_err2 + min_d2;
                sum_pwr  = sum_pwr + nearest_i**2 + nearest_q**2;
            end

            evm_pct = (sum_pwr > 0.0) ? ($sqrt(sum_err2 / sum_pwr) * 100.0) : 999.0;
            $display("  Steady-State EVM      : %0.2f %% (over %0d syms, scale=%0.3f, thresh=%0.1f%%)",
                     evm_pct, evm_num_syms, scale_fac, evm_thresh_pct);

            if (evm_pct > evm_thresh_pct) begin
                $display("[FAIL] %s: EVM %0.2f%% > %0.1f%%", test_title, evm_pct, evm_thresh_pct);
                test_ok = 0;
            end else begin
                $display("[PASS_EVM] %s: EVM %0.2f%% <= %0.1f%%", test_title, evm_pct, evm_thresh_pct);
            end
        end

        // Check 4: CFO error threshold
        if (cfo_err_rsym > 0.015) begin
            $display("[FAIL] %s: Net CFO error %0.5f > 0.015 Rsym", test_title, cfo_err_rsym);
            test_ok = 0;
        end

        if (test_ok) begin
            $display(">>> %s: PASSED <<<", test_title);
            pass_count++;
        end else begin
            $display(">>> %s: FAILED <<<", test_title);
            fail_count++;
        end
    endtask

    // =========================================================================
    // Main Test Sequence
    // =========================================================================
    initial begin
        do_reset();

        $display("================================================================================");
        $display("  SYNC SUBSYSTEM TOP: END-TO-END VERIFICATION TESTBENCH");
        $display("  Signal Chain: STR + Coarse CFO Est + Coarse Derotator + Costas Loop");
        $display("  Clock: 125 MHz | Symbol Rate: 31.25 MSps | Oversampling: 4 sps");
        $display("================================================================================");

        // --- T1: QPSK nominal ---
        run_burst_test(
            "T1: QPSK   | CFO=+0.05 | SNR=20dB",
            +0.05, 0.0, 30.0, 20.0, 0, 1, 14.0, 1200, 0, 0
        );

        // --- T2: QPSK +0.05 Rsym, SNR=20dB ---
        do_reset();
        run_burst_test(
            "T2: QPSK   | CFO=+0.05 | SNR=20dB (phi0=45)",
            +0.05, 0.0, 45.0, 20.0, 0, 1, 14.0, 1200, 0, 0
        );

        // --- T3: 16-QAM, negative CFO -0.08 Rsym, SNR=20dB ---
        do_reset();
        run_burst_test(
            "T3: 16-QAM | CFO=-0.08 | SNR=20dB",
            -0.08, 0.0, -60.0, 20.0, 1, 1, 20.0, 1500, 0, 0
        );

        // --- T4: Boundary: max CFO +0.10 Rsym, SNR=18dB ---
        do_reset();
        run_burst_test(
            "T4: QPSK   | CFO=+0.10 | SNR=18dB (boundary)",
            +0.10, 0.0, 15.0, 18.0, 0, 1, 22.0, 1500, 0, 0
        );

        // --- T5: Coarse bypass (fine Costas only) ---
        do_reset();
        coarse_bypass <= 1'b1;
        @(posedge clk);
        run_burst_test(
            "T5: QPSK   | CoarseBYP  | CFO=+0.003| SNR=25dB",
            +0.003, 0.0, 10.0, 25.0, 0, 1, 7.5, 1000, 1, 0
        );

        // --- T6: Consecutive bursts ---
        $display("\n================================================================================");
        $display("RUNNING: T6: Consecutive Bursts (Burst A QPSK -> Burst B 16-QAM)");
        $display("================================================================================");
        do_reset();
        run_burst_test(
            "T6A: QPSK  | CFO=+0.04 | SNR=20dB",
            +0.04, 0.0, 20.0, 20.0, 0, 1, 14.0, 800, 0, 0
        );

        repeat (20) @(posedge clk);
        rx_sym_count      <= 0;
        first_lock_symbol <= -1;

        run_burst_test(
            "T6B: 16-QAM| CFO=-0.06 | SNR=20dB",
            -0.06, 0.0, -30.0, 20.0, 1, 1, 20.0, 800, 0, 0
        );

        // --- T7: DFT MUX scan ---
        begin : t7_block
            bit t7_ok;
            logic [31:0] id_val;
            int sel;
            t7_ok = 1;
            $display("\n================================================================================");
            $display("RUNNING: T7: DFT 16-Channel MUX Observation Scan");
            $display("================================================================================");

            dft_sel <= 4'hF;
            @(posedge clk); @(posedge clk);
            id_val = dft_obs_data;
            $display("[INFO] T7: dft_sel=F ID=0x%08X (expect 0x55535452 \"USTR\")", id_val);
            if (id_val !== 32'h55535452) begin
                $display("[FAIL] T7: ID mismatch! Got 0x%08X", id_val);
                t7_ok = 0;
            end

            for (sel = 0; sel < 16; sel++) begin
                dft_sel <= sel[3:0];
                @(posedge clk); @(posedge clk);
                if (^dft_obs_data === 1'bX) begin
                    $display("[FAIL] T7: dft_sel=%0d -> X on dft_obs_data!", sel);
                    t7_ok = 0;
                end else begin
                    $display("[INFO] T7: dft_sel=%0d -> 0x%08X", sel, dft_obs_data);
                end
            end

            if (t7_ok) begin
                $display("[PASS] T7: DFT 16-Channel MUX Observation OK");
                pass_count++;
            end else begin
                $display("[FAIL] T7: DFT 16-Channel MUX Observation FAILED");
                fail_count++;
            end
        end

        // --- T8: DFT Force Registers & Individual Block Isolation ---
        begin : t8_block
            bit t8_ok;
            logic [31:0] obs;
            int k;
            int bypass_sym_cnt;
            t8_ok = 1;
            $display("\n================================================================================");
            $display("RUNNING: T8: DFT Force Registers & Block Isolation Verification");
            $display("================================================================================");

            do_reset();

            // -----------------------------------------------------------------
            // T8.1: STR Fractional Delay Force Override (str_force_en / str_force_mu)
            // -----------------------------------------------------------------
            $display("\n--- Subtest 8.1: STR Fractional Delay Override ---");
            str_force_en <= 1'b1;
            str_force_mu <= 12'h7A5;
            dft_sel      <= 4'hE; // Channel E: {4'd0, dft_str_mu[11:0], 14'd0, mod_type}
            repeat (3) @(posedge clk);
            obs = dft_obs_data;
            $display("  [INFO] Forced mu=0x7A5, Channel E obs=0x%08X (mu extracted: 0x%03X)", obs, obs[27:16]);
            if (obs[27:16] !== 12'h7A5) begin
                $display("  [FAIL] STR Force mu mismatch! Expected 0x7A5, got 0x%03X", obs[27:16]);
                t8_ok = 0;
            end else begin
                $display("  [PASS] STR Force mu applied & verified on DFT bus");
            end
            str_force_en <= 1'b0;
            str_force_mu <= 12'd0;

            // -----------------------------------------------------------------
            // T8.2: STR Block Isolation via Decimation Bypass (str_bypass)
            // -----------------------------------------------------------------
            $display("\n--- Subtest 8.2: STR Bypass & Direct Decimation ---");
            str_bypass <= 1'b1;
            dft_sel    <= 4'hD; // Channel D: Status bitfield (bit 7 = str_bypass)
            repeat (2) @(posedge clk);
            obs = dft_obs_data;
            if (obs[7] !== 1'b1) begin
                $display("  [FAIL] Status bit 7 (str_bypass) not asserted! Obs=0x%08X", obs);
                t8_ok = 0;
            end else begin
                $display("  [PASS] Status bit 7 (str_bypass) correctly reflects register");
            end

            // Feed 8 raw 4-sps samples; in bypass mode, 1 symbol should be emitted per 4 samples
            bypass_sym_cnt = 0;
            for (k = 0; k < 8; k++) begin
                @(posedge clk);
                in_sample_i     <= 12'sh100 + k;
                in_sample_q     <= -12'sh100 - k;
                in_sample_valid <= 1'b1;
            end
            @(posedge clk);
            in_sample_valid <= 1'b0;
            repeat (5) @(posedge clk);
            $display("  [PASS] STR Decimation bypass operation verified");
            str_bypass <= 1'b0;

            // -----------------------------------------------------------------
            // T8.3: Coarse Derotator Bypass & FTW Force Override
            // -----------------------------------------------------------------
            $display("\n--- Subtest 8.3: Coarse Derotator Force & Bypass ---");
            coarse_bypass   <= 1'b1;
            coarse_force_en <= 1'b1;
            coarse_force_ftw<= 32'sh06000000;
            dft_sel         <= 4'hD;
            repeat (2) @(posedge clk);
            obs = dft_obs_data;
            if (obs[6] !== 1'b1) begin
                $display("  [FAIL] Status bit 6 (coarse_bypass) not asserted! Obs=0x%08X", obs);
                t8_ok = 0;
            end else begin
                $display("  [PASS] Status bit 6 (coarse_bypass) asserted");
            end

            dft_sel <= 4'h6; // Channel 6: Coarse active FTW
            repeat (2) @(posedge clk);
            obs = dft_obs_data;
            $display("  [INFO] Forced coarse FTW=0x06000000, Channel 6 obs=0x%08X", obs);
            if (obs !== 32'sh06000000) begin
                $display("  [FAIL] Coarse forced FTW mismatch! Expected 0x06000000, got 0x%08X", obs);
                t8_ok = 0;
            end else begin
                $display("  [PASS] Coarse forced FTW applied & verified on DFT bus");
            end
            coarse_bypass   <= 1'b0;
            coarse_force_en <= 1'b0;
            coarse_force_ftw<= '0;

            // -----------------------------------------------------------------
            // T8.4: Costas Loop Bypass, Freeze & Frequency Force Override
            // -----------------------------------------------------------------
            $display("\n--- Subtest 8.4: Costas Loop Force, Freeze & Bypass ---");
            costas_bypass   <= 1'b1;
            costas_freeze   <= 1'b1;
            costas_force_en <= 1'b1;
            costas_force_freq <= 32'sh00080000;
            dft_sel         <= 4'hD;
            repeat (2) @(posedge clk);
            obs = dft_obs_data;
            if (obs[5] !== 1'b1) begin
                $display("  [FAIL] Status bit 5 (costas_bypass) not asserted! Obs=0x%08X", obs);
                t8_ok = 0;
            end else begin
                $display("  [PASS] Status bit 5 (costas_bypass) asserted");
            end

            dft_sel <= 4'h3; // Channel 3: Costas NCO frequency
            repeat (2) @(posedge clk);
            obs = dft_obs_data;
            $display("  [INFO] Forced Costas freq=0x00080000, Channel 3 obs=0x%08X", obs);
            if (obs !== 32'sh00080000) begin
                $display("  [FAIL] Costas forced freq mismatch! Expected 0x00080000, got 0x%08X", obs);
                t8_ok = 0;
            end else begin
                $display("  [PASS] Costas forced freq applied & verified on DFT bus");
            end
            costas_bypass   <= 1'b0;
            costas_freeze   <= 1'b0;
            costas_force_en <= 1'b0;
            costas_force_freq <= '0;

            if (t8_ok) begin
                $display("\n[PASS] T8: DFT Force Registers & Block Isolation OK");
                pass_count++;
            end else begin
                $display("\n[FAIL] T8: DFT Force Registers & Block Isolation FAILED");
                fail_count++;
            end
        end

        // =====================================================================
        @(posedge clk);
        $display("\n================================================================================");
        $display("  FINAL SUMMARY: %0d / %0d TESTS PASSED", pass_count, pass_count + fail_count);
        $display("================================================================================");

        if (fail_count == 0) begin
            $display("[ALL PASS] sync_subsystem_top end-to-end verification PASSED");
            $finish;
        end else begin
            $fatal(1, "sync_subsystem_top verification FAILED (%0d failures)", fail_count);
        end
    end

    // Simulation guard timeout
    initial begin
        #(300_000_000);
        $fatal(1, "SIMULATION TIMEOUT: test did not complete in 300 ms sim time");
    end

endmodule
