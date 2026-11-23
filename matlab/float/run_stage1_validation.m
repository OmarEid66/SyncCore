function metrics = run_stage1_validation()
% RUN_STAGE1_VALIDATION Master Stage 1 floating-point STR validation script.
%   Compatible with MATLAB (any version) and GNU Octave (zero toolboxes).
%
%   Validates:
%     1. Cubic Farrow interpolation accuracy against exact analytical reference.
%     2. Gardner TED S-curve & exact gain Kd (QPSK & 16-QAM).
%     3. Analytical PI loop gain design (Bn*Ts = 0.0080, zeta = 0.707) & stability.
%     4. Acquisition time, residual timing error, RMS jitter at SNR = 15 dB.
%     5. SNR sensitivity sweep (0 to 20 dB).
%     6. Sampling clock mismatch sensitivity sweep (-500 to +500 ppm).
%     7. All 9 mandatory Stage 1 plots saved to ../plots/stage1/.

close all; clc;
out_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'plots', 'stage1');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

fprintf('============================================================\n');
fprintf('  STAGE 1: MATLAB FLOATING-POINT GOLDEN MODEL VALIDATION\n');
fprintf('============================================================\n\n');

alpha = 0.35;
Ns    = 4;

%% 1. Validate Cubic Farrow Interpolator
frep = validate_farrow_float(alpha);
fprintf('[1] Cubic Farrow Interpolator Validation (alpha=%.2f, Ns=%d):\n', alpha, Ns);
fprintf('    RC pulse max |err|   = %.3e, RMS err = %.3e (SQNR = %.2f dB)\n', ...
    frep.max_abs_err_rc, frep.rms_err_rc, frep.sqnr_db_rc);
fprintf('    QPSK signal RMS err  = %.3e, max err = %.3e (SNR  = %.2f dB)\n\n', ...
    frep.sig_rms_err, frep.sig_max_err, frep.sig_snr_db);

%% 2. Measure Gardner TED S-Curve & Exact Gain Kd
[tau_grid, sc_qpsk, Kd_qpsk]   = compute_ted_scurve('QPSK',  alpha, 4000, 101);
[~,        sc_16qam, Kd_16qam] = compute_ted_scurve('16QAM', alpha, 4000, 102);
Kd = 0.5 * (Kd_qpsk + Kd_16qam);
fprintf('[2] Gardner TED Gain Measurement (Es = 1, alpha = %.2f):\n', alpha);
fprintf('    Kd (QPSK)  = %.4f\n', Kd_qpsk);
fprintf('    Kd (16QAM) = %.4f\n', Kd_16qam);
fprintf('    Kd (mean)  = %.4f\n\n', Kd);

%% 3. Design Verified Optimal PI Loop Gains (Kp, Ki)
% Selected via Pareto sweep: Bn*Ts = 0.0080, zeta = 0.707, K0 = 2.0
% Achieves BOTH Acq < 150 symbols across [-0.45, +0.45] Ts AND
% RMS jitter <= 0.020 Ts at SNR = 15 dB for both QPSK and 16-QAM.
BnTs = 0.0080;
zeta = 0.707;
K0   = 2.0;
loop0 = design_pi_gains(BnTs, zeta, Kd, K0, 0);
loop2 = design_pi_gains(BnTs, zeta, Kd, K0, 2); % 2-symbol (8-clk) loop delay
fprintf('[3] PI Loop Filter Design (Bn*Ts = %.4f, zeta = %.3f, K0 = %.1f):\n', BnTs, zeta, K0);
fprintf('    wn*Ts = %.5f rad/sym\n', loop0.wnTs);
fprintf('    Kp    = %.6f\n', loop0.Kp);
fprintf('    Ki    = %.6f\n', loop0.Ki);
fprintf('    D=0 sym: max|pole|=%.5f, PM=%.1f deg, stable=%d\n', ...
    loop0.max_pole_mag, loop0.phase_margin_deg, loop0.is_stable);
fprintf('    D=2 sym: max|pole|=%.5f, PM=%.1f deg, stable=%d\n\n', ...
    loop2.max_pole_mag, loop2.phase_margin_deg, loop2.is_stable);

str_cfg.Kp              = loop0.Kp;
str_cfg.Ki              = loop0.Ki;
str_cfg.sps             = Ns;
str_cfg.exact_mu        = true;
str_cfg.init_eta        = 0.5;
str_cfg.loop_delay_clks = 0;
str_cfg.u_clamp         = 0.25;

%% 4. Nominal Validation at SNR = 15 dB (QPSK & 16-QAM, tau0 = +0.35 Ts)
sig_qpsk_cfg = struct('mod_type','QPSK','num_symbols',2500,'sps',Ns,'alpha',alpha, ...
    'rrc_span',12,'snr_db',15,'timing_offset_sym',0.35,'clk_ppm',0,'sig_scale',1.0,'seed',42);
[sig_q, res_q, m_q] = run_single_case(sig_qpsk_cfg, str_cfg);

sig_qam_cfg = struct('mod_type','16QAM','num_symbols',2500,'sps',Ns,'alpha',alpha, ...
    'rrc_span',12,'snr_db',15,'timing_offset_sym',0.35,'clk_ppm',0,'sig_scale',1.0,'seed',43);
[sig_m, res_m, m_m] = run_single_case(sig_qam_cfg, str_cfg);

% Clock mismatch test (+200 ppm) at 15 dB
sig_ppm_cfg = sig_qpsk_cfg;
sig_ppm_cfg.clk_ppm = 200;
sig_ppm_cfg.seed    = 44;
[~, res_ppm, m_ppm] = run_single_case(sig_ppm_cfg, str_cfg);

fprintf('[4] Reference Performance at SNR = 15 dB (tau0 = +0.35 Ts):\n');
fprintf('    QPSK  (0 ppm):   Acq = %3d syms | Mean err = %+.5f Ts | RMS jitter = %.5f Ts | EVM = %.2f%%\n', ...
    m_q.acq_syms, m_q.mean_err_ts, m_q.rms_jitter_ts, m_q.evm_pct);
fprintf('    16QAM (0 ppm):   Acq = %3d syms | Mean err = %+.5f Ts | RMS jitter = %.5f Ts | EVM = %.2f%%\n', ...
    m_m.acq_syms, m_m.mean_err_ts, m_m.rms_jitter_ts, m_m.evm_pct);
fprintf('    QPSK (+200 ppm): Acq = %3d syms | Mean err = %+.5f Ts | RMS jitter = %.5f Ts | EVM = %.2f%%\n\n', ...
    m_ppm.acq_syms, m_ppm.mean_err_ts, m_ppm.rms_jitter_ts, m_ppm.evm_pct);

%% 5. SNR Sensitivity Sweep (0 to 20 dB)
snr_list = [0, 5, 10, 12, 15, 18, 20];
rms_snr_qpsk = zeros(size(snr_list));
rms_snr_16q  = zeros(size(snr_list));

for i = 1:length(snr_list)
    c1 = sig_qpsk_cfg; c1.snr_db = snr_list(i); c1.seed = 200 + i;
    [~, ~, mq] = run_single_case(c1, str_cfg);
    rms_snr_qpsk(i) = mq.rms_jitter_ts;

    c2 = sig_qam_cfg; c2.snr_db = snr_list(i); c2.seed = 300 + i;
    [~, ~, mm] = run_single_case(c2, str_cfg);
    rms_snr_16q(i) = mm.rms_jitter_ts;
end

%% 6. Clock Mismatch Sensitivity Sweep (-500 to +500 ppm at 15 dB)
ppm_list = [-500, -200, -100, 0, 100, 200, 500];
rms_ppm_qpsk = zeros(size(ppm_list));
for i = 1:length(ppm_list)
    cp = sig_qpsk_cfg; cp.clk_ppm = ppm_list(i); cp.seed = 400 + i;
    [~, ~, mp] = run_single_case(cp, str_cfg);
    rms_ppm_qpsk(i) = mp.rms_jitter_ts;
end

%% 7. Generate All 9 Mandatory Plots (MATLAB + Octave compatible)
% Plot 1: Received vs Recovered Signal
f1 = figure('Visible','off','Position',[100 100 900 420]);
k_win = 350:390;
n_start = res_q.sym_sample_idx(k_win(1)) - 2;
n_end   = res_q.sym_sample_idx(k_win(end)) + 2;
t_rx = (n_start:n_end);
plot(t_rx, real(sig_q.rx_mf(t_rx + 1)), 'b.-'); hold on; grid on;
t_sym = res_q.sym_sample_idx(k_win) - 2 + res_q.sym_mu(k_win);
stem(t_sym, res_q.out_sym_i(k_win), 'r', 'filled');
xlabel('Input Sample Index n'); ylabel('Amplitude');
title('Plot 1: Received 4x-Oversampled vs Recovered Symbol Signal (QPSK, SNR=15 dB)');
legend('Received 4x-oversampled MF (I)', 'Recovered STR Symbols (I)', 'Location', 'best');
saveas(f1, fullfile(out_dir, '01_received_vs_recovered.png')); close(f1);

% Plot 2: Timing Error Evolution
f2 = figure('Visible','off','Position',[100 100 900 400]);
plot(res_q.ted_err, 'Color', [0.2 0.4 0.8]); hold on; grid on;
e_ma = conv(res_q.ted_err, ones(25,1)/25, 'same');
plot(e_ma, 'r-', 'LineWidth', 1.5);
xlabel('Symbol Index k'); ylabel('TED Error e[k]');
title('Plot 2: Gardner Timing Error Detector Evolution (SNR=15 dB, \tau_0=+0.35 T_s)');
legend('Gardner TED e[k] (QPSK)', '25-sym Moving Avg', 'Location', 'best');
saveas(f2, fullfile(out_dir, '02_timing_error_evolution.png')); close(f2);

% Plot 3: Fractional Delay Evolution (mu)
f3 = figure('Visible','off','Position',[100 100 900 440]);
subplot(2,1,1);
plot(res_q.sym_mu, 'b-', 'LineWidth', 1.0); grid on;
ylabel('\mu (samples)'); title('Plot 3: Fractional Delay \mu Evolution (0 ppm vs +200 ppm)');
legend('QPSK, 0 ppm, \tau_0=+0.35 T_s');
subplot(2,1,2);
plot(res_ppm.sym_mu, 'm-', 'LineWidth', 1.0); grid on;
xlabel('Symbol Index k'); ylabel('\mu (samples)');
legend('QPSK, +200 ppm (continuous phase wrap tracking)');
saveas(f3, fullfile(out_dir, '03_fractional_delay_evolution.png')); close(f3);

% Plot 4: Timing Phase Evolution (NCO eta and PI output u)
f4 = figure('Visible','off','Position',[100 100 900 460]);
subplot(2,1,1);
plot(1:200, res_q.eta_trace(1:200), 'b.-'); grid on;
xlabel('Input Sample Index n'); ylabel('\eta[n]');
title('Plot 4: Timing Phase Accumulator \eta[n] & PI Loop Filter Output u[k]');
legend('Mod-1 NCO Phase \eta[n] (first 200 samples)');
subplot(2,1,2);
plot(res_q.pi_out, 'b-'); hold on; grid on;
plot(res_ppm.pi_out, 'r-');
xlabel('Symbol Index k'); ylabel('PI Output u[k]');
legend('u[k] (0 ppm)', 'u[k] (+200 ppm)', 'Location', 'best');
saveas(f4, fullfile(out_dir, '04_timing_phase_evolution.png')); close(f4);

% Plot 5: Acquisition Behavior across Initial Offsets
f5 = figure('Visible','off','Position',[100 100 900 420]);
tau_tests = [-0.45, -0.25, 0.0, 0.25, 0.45];
hold on; grid on;
leg_entries = cell(length(tau_tests) + 2, 1);
for ti = 1:length(tau_tests)
    ct = sig_qpsk_cfg; ct.timing_offset_sym = tau_tests(ti); ct.seed = 500 + ti;
    [~, ~, mt] = run_single_case(ct, str_cfg);
    n_p = min(300, length(mt.tau_err_ts));
    plot(1:n_p, mt.tau_err_ts(1:n_p), 'LineWidth', 1.2);
    leg_entries{ti} = sprintf('\\tau_0 = %+.2f T_s (acq=%d sym)', tau_tests(ti), mt.acq_syms);
end
plot([1 300], [ 0.02  0.02], 'k--');
plot([1 300], [-0.02 -0.02], 'k--');
plot([150 150], [-0.5 0.5], 'r:', 'LineWidth', 1.5);
leg_entries{length(tau_tests)+1} = '\pm0.02 T_s';
leg_entries{length(tau_tests)+2} = '150-sym Limit';
xlabel('Symbol Index k'); ylabel('True Timing Error \tau_{err}[k] (T_s)');
title('Plot 5: Acquisition Behavior Across Initial Fractional Timing Offsets (SNR=15 dB)');
legend(leg_entries, 'Location', 'best');
saveas(f5, fullfile(out_dir, '05_acquisition_behavior.png')); close(f5);

% Plot 6: Residual Timing Error (Steady-State QPSK & 16-QAM)
f6 = figure('Visible','off','Position',[100 100 900 420]);
k_ss = 250:length(m_q.tau_err_ts);
plot(k_ss, m_q.tau_err_ts(k_ss), 'b-'); hold on; grid on;
k_ss_m = 250:length(m_m.tau_err_ts);
plot(k_ss_m, m_m.tau_err_ts(k_ss_m), 'r-');
plot([250 k_ss(end)], [ 0.02  0.02], 'k--');
plot([250 k_ss(end)], [-0.02 -0.02], 'k--');
xlabel('Symbol Index k'); ylabel('Residual Timing Error (T_s)');
title('Plot 6: Steady-State Residual Timing Error (SNR=15 dB)');
legend(sprintf('QPSK (RMS=%.4f T_s)', m_q.rms_jitter_ts), ...
       sprintf('16-QAM (RMS=%.4f T_s)', m_m.rms_jitter_ts), ...
       'Spec \pm0.02 T_s', 'Location', 'best');
saveas(f6, fullfile(out_dir, '06_residual_timing_error.png')); close(f6);

% Plot 7: RMS Timing Jitter vs SNR and vs Clock Mismatch
f7 = figure('Visible','off','Position',[100 100 960 400]);
subplot(1,2,1);
plot(snr_list, rms_snr_qpsk, 'bo-', 'LineWidth', 1.5); hold on; grid on;
plot(snr_list, rms_snr_16q,  'rs-', 'LineWidth', 1.5);
plot([snr_list(1) snr_list(end)], [0.02 0.02], 'k--', 'LineWidth', 1.2);
xlabel('SNR (E_s/N_0, dB)'); ylabel('RMS Timing Jitter (T_s)');
title('Plot 7a: RMS Timing Jitter vs SNR');
legend('QPSK', '16-QAM', 'Spec 0.02 T_s @ 15 dB', 'Location', 'northeast');
subplot(1,2,2);
plot(ppm_list, rms_ppm_qpsk, 'mo-', 'LineWidth', 1.5); hold on; grid on;
plot([ppm_list(1) ppm_list(end)], [0.02 0.02], 'k--', 'LineWidth', 1.2);
xlabel('Sampling Clock Offset (ppm)'); ylabel('RMS Timing Jitter (T_s)');
title('Plot 7b: RMS Jitter vs Clock Mismatch');
legend('QPSK @ 15 dB', 'Spec 0.02 T_s', 'Location', 'best');
saveas(f7, fullfile(out_dir, '07_rms_timing_jitter.png')); close(f7);

% Plot 8: Recovered Constellation for QPSK
f8 = figure('Visible','off','Position',[100 100 550 520]);
sym_ss_q = res_q.out_sym(300:end-20);
plot(real(sym_ss_q), imag(sym_ss_q), '.', 'Color', [0.1 0.4 0.8], 'MarkerSize', 6);
hold on; grid on; axis equal;
ideal_q = [1+1j, 1-1j, -1+1j, -1-1j] / sqrt(2);
plot(real(ideal_q), imag(ideal_q), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
xlim([-1.5 1.5]); ylim([-1.5 1.5]);
xlabel('In-Phase (I)'); ylabel('Quadrature (Q)');
title(sprintf('Plot 8: Recovered QPSK Constellation (SNR=15 dB, RMS Jitter=%.4f T_s)', m_q.rms_jitter_ts));
saveas(f8, fullfile(out_dir, '08_constellation_qpsk.png')); close(f8);

% Plot 9: Recovered Constellation for 16-QAM
f9 = figure('Visible','off','Position',[100 100 550 520]);
sym_ss_m = res_m.out_sym(300:end-20);
plot(real(sym_ss_m), imag(sym_ss_m), '.', 'Color', [0.8 0.2 0.2], 'MarkerSize', 6);
hold on; grid on; axis equal;
[gi, gq] = meshgrid([-3 -1 1 3]/sqrt(10), [-3 -1 1 3]/sqrt(10));
plot(gi(:), gq(:), 'kx', 'MarkerSize', 10, 'LineWidth', 1.8);
xlim([-1.5 1.5]); ylim([-1.5 1.5]);
xlabel('In-Phase (I)'); ylabel('Quadrature (Q)');
title(sprintf('Plot 9: Recovered 16-QAM Constellation (SNR=15 dB, RMS Jitter=%.4f T_s)', m_m.rms_jitter_ts));
saveas(f9, fullfile(out_dir, '09_constellation_16qam.png')); close(f9);

metrics.farrow       = frep;
metrics.Kd           = Kd;
metrics.loop         = loop0;
metrics.qpsk_15db    = m_q;
metrics.qam16_15db   = m_m;
metrics.qpsk_200ppm  = m_ppm;
metrics.snr_list     = snr_list;
metrics.rms_snr_qpsk = rms_snr_qpsk;
metrics.rms_snr_16q  = rms_snr_16q;
metrics.ppm_list     = ppm_list;
metrics.rms_ppm_qpsk = rms_ppm_qpsk;

fprintf('All 9 Stage 1 validation plots saved to:\n  %s\n', out_dir);
end

function [sig, res, m] = run_single_case(sig_cfg, str_cfg)
    sig = generate_str_signal(sig_cfg);
    % Skip initial RRC matched-filter warmup transient (rrc_span * sps samples)
    skip = sig_cfg.rrc_span * sig_cfg.sps;
    rx_trim = sig.rx_mf(skip + 1 : end);
    in_valid = true(length(rx_trim), 1);
    res = str_float_model(real(rx_trim), imag(rx_trim), in_valid, str_cfg);
    res.sym_sample_idx = res.sym_sample_idx + skip;

    eps_clk = sig_cfg.clk_ppm * 1e-6;
    Ns      = sig_cfg.sps;
    L_mf    = sig.mf_delay_samp;

    t_eff = (((res.sym_sample_idx - 2.0) + res.sym_mu) - L_mf) / Ns * (1.0 + eps_clk) ...
            + sig_cfg.timing_offset_sym;
    k_ideal = round(t_eff);
    tau_err = t_eff - k_ideal;

    valid_int = (k_ideal >= sig_cfg.rrc_span) & (k_ideal <= sig_cfg.num_symbols - sig_cfg.rrc_span - 1);
    tau_v = tau_err(valid_int);
    sym_v = res.out_sym(valid_int);
    ref_v = sig.tx_symbols(k_ideal(valid_int) + 1);

    % Acquisition time: first symbol where |tau_err| stays <= 0.05 Ts for 20 consecutive syms
    acq = length(tau_v);
    win = 20;
    for k = 1:(length(tau_v) - win)
        if all(abs(tau_v(k : k + win - 1)) <= 0.05)
            acq = k;
            break;
        end
    end

    ss_start = min(300, floor(length(tau_v)/2));
    tau_ss   = tau_v(ss_start:end);
    sym_ss   = sym_v(ss_start:end);
    ref_ss   = ref_v(ss_start:end);

    m.tau_err_ts    = tau_v;
    m.acq_syms      = acq;
    m.mean_err_ts   = mean(tau_ss);
    m.rms_jitter_ts = sqrt(mean(tau_ss.^2));
    m.std_jitter_ts = std(tau_ss);
    m.evm_pct       = 100.0 * sqrt(mean(abs(sym_ss - ref_ss).^2) / mean(abs(ref_ss).^2));
end
