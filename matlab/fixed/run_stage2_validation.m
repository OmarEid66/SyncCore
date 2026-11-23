function report = run_stage2_validation()
% RUN_STAGE2_VALIDATION Stage 2 Fixed-Point vs Stage 1 Floating-Point comparison.
%   Compatible with MATLAB and GNU Octave (zero toolboxes).
%
%   Measures all 7 mandatory comparison metrics:
%     1. Interpolation error (fixed vs float Farrow)
%     2. Gardner TED error
%     3. PI controller output error
%     4. Fractional-delay (mu) error
%     5. Symbol output error (Q1.15 vs float64)
%     6. Acquisition-time difference
%     7. RMS timing jitter difference
%
%   Also sweeps W_mu in {8, 10, 12} bits and generates comparison plots in
%   ../plots/stage2/.

close all; clc;
this_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(this_dir, '..', 'float'));
addpath(this_dir);

out_dir = fullfile(this_dir, '..', 'plots', 'stage2');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

fprintf('==================================================================\n');
fprintf('  STAGE 2: BIT-ACCURATE FIXED-POINT MODEL & FLOAT VS FIXED AUDIT\n');
fprintf('==================================================================\n\n');

alpha     = 0.35;
Ns        = 4;
sig_scale = 0.5;   % AGC scaling for Q1.11 headroom (Es = 0.25, zero clipping)

% Calibrated loop gains for sig_scale = 0.5 (Kd scales by sig_scale^2 = 0.25)
% From Stage 1 (Es=1): Kd_1 = 1.0697, Kp_1 = 0.009865, Ki_1 = 0.000105
Kd_scaled = 1.0697 * (sig_scale^2);
loop = design_pi_gains(0.0080, 0.707, Kd_scaled, 2.0, 0);
Kp_fl = loop.Kp;
Ki_fl = loop.Ki;

kp4_fx = round(Ns * Kp_fl * 65536);    % SQ1.16
ki_fx  = round(Ki_fl * 2097152);       % scaled 2^21

fprintf('[1] Fixed-Point Loop Coefficient Quantization (sig_scale = %.2f):\n', sig_scale);
fprintf('    4*Kp float = %.6f -> kp4_fx = %d (Q1.16 = %.6f, rel err = %.4f%%)\n', ...
    Ns*Kp_fl, kp4_fx, kp4_fx/65536, 100*abs(kp4_fx/65536 - Ns*Kp_fl)/(Ns*Kp_fl));
fprintf('    Ki   float = %.6f -> ki_fx  = %d (2^-21 = %.6f, rel err = %.4f%%)\n\n', ...
    Ki_fl, ki_fx, ki_fx/2097152, 100*abs(ki_fx/2097152 - Ki_fl)/Ki_fl);

%% 2. Open-Loop Block-Level Quantization Comparison (Farrow, TED, PI)
sig_ol = generate_str_signal(struct('mod_type','16QAM','num_symbols',1000,'sps',Ns, ...
    'alpha',alpha,'rrc_span',12,'snr_db',15,'timing_offset_sym',0.35,'clk_ppm',0, ...
    'sig_scale',sig_scale,'seed',77));
rx_i_fl  = real(sig_ol.rx_mf);
rx_q_fl  = imag(sig_ol.rx_mf);
rx_i_q11 = fx_sat(round(rx_i_fl * 2048), -2048, 2047);
rx_q_q11 = fx_sat(round(rx_q_fl * 2048), -2048, 2047);

% Check input clipping rate in Q1.11
clip_pct = 100.0 * mean((abs(rx_i_fl) >= 1.0) | (abs(rx_q_fl) >= 1.0));
fprintf('[2] Input Q1.11 Headroom Check (16-QAM, SNR=15 dB, sig_scale=%.2f):\n', sig_scale);
fprintf('    Max |I/Q| = %.4f | Clipping rate = %.4f%%\n\n', ...
    max(max(abs(rx_i_fl)), max(abs(rx_q_fl))), clip_pct);

% Compare Farrow fixed vs float across W_mu = 8, 10, 12
idx_t = (sig_ol.mf_delay_samp + 20) : (length(rx_i_fl) - sig_ol.mf_delay_samp - 20);
fprintf('[3] Open-Loop Cubic Farrow Interpolation Error vs W_mu (16-QAM, Q1.11->Q1.15):\n');
for W_mu = [8, 10, 12]
    mu_test_fl = 0.3731;
    mu_test_fx = round(mu_test_fl * 2^W_mu);
    % Compare against float Farrow on quantized Q1.11 inputs (pure Farrow arithmetic error)
    yi_fl_q = farrow_cubic_float(rx_i_q11(idx_t-1)/2048, rx_i_q11(idx_t)/2048, rx_i_q11(idx_t+1)/2048, rx_i_q11(idx_t+2)/2048, mu_test_fl);
    yi_fx   = farrow_cubic_fixed(rx_i_q11(idx_t-1), rx_i_q11(idx_t), rx_i_q11(idx_t+1), rx_i_q11(idx_t+2), mu_test_fx, W_mu) / 32768.0;
    % And total error vs unquantized float64 Farrow
    yi_fl_full = farrow_cubic_float(rx_i_fl(idx_t-1), rx_i_fl(idx_t), rx_i_fl(idx_t+1), rx_i_fl(idx_t+2), mu_test_fl);
    err_arith = yi_fx - yi_fl_q;
    err_total = yi_fx - yi_fl_full;
    fprintf('    W_mu=%2d bits: Farrow arith RMS=%.3e (max=%.3e) | Total (incl. Q1.11 ADC) RMS=%.3e (SQNR=%.2f dB)\n', ...
        W_mu, sqrt(mean(err_arith.^2)), max(abs(err_arith)), sqrt(mean(err_total.^2)), ...
        20*log10(sqrt(mean(yi_fl_full.^2)) / sqrt(mean(err_total.^2))));
end
fprintf('\n');

%% 3. Closed-Loop Float vs Fixed Comparison (QPSK, 16-QAM, +200 ppm)
W_mu_sel = 12;
fl_cfg = struct('Kp',Kp_fl,'Ki',Ki_fl,'sps',Ns,'exact_mu',true,'init_eta',0.5,'loop_delay_clks',0,'u_clamp',0.25);
fx_cfg = struct('kp4_fx',kp4_fx,'ki_fx',ki_fx,'W_mu',W_mu_sel,'init_eta_q22',2097152,'loop_delay_clks',0);

cases = {
    'QPSK  15dB   0ppm', struct('mod_type','QPSK', 'num_symbols',2500,'sps',Ns,'alpha',alpha,'rrc_span',12,'snr_db',15,'timing_offset_sym',0.35,'clk_ppm',0,  'sig_scale',sig_scale,'seed',42);
    '16QAM 15dB   0ppm', struct('mod_type','16QAM','num_symbols',2500,'sps',Ns,'alpha',alpha,'rrc_span',12,'snr_db',15,'timing_offset_sym',0.35,'clk_ppm',0,  'sig_scale',sig_scale,'seed',43);
    'QPSK  15dB +200ppm',struct('mod_type','QPSK', 'num_symbols',2500,'sps',Ns,'alpha',alpha,'rrc_span',12,'snr_db',15,'timing_offset_sym',0.35,'clk_ppm',200,'sig_scale',sig_scale,'seed',44);
    '16QAM 15dB -200ppm',struct('mod_type','16QAM','num_symbols',2500,'sps',Ns,'alpha',alpha,'rrc_span',12,'snr_db',15,'timing_offset_sym',-0.35,'clk_ppm',-200,'sig_scale',sig_scale,'seed',45);
    'QPSK  Inf    0ppm', struct('mod_type','QPSK', 'num_symbols',2500,'sps',Ns,'alpha',alpha,'rrc_span',12,'snr_db',Inf,'timing_offset_sym',0.35,'clk_ppm',0, 'sig_scale',sig_scale,'seed',46);
    '16QAM Inf    0ppm', struct('mod_type','16QAM','num_symbols',2500,'sps',Ns,'alpha',alpha,'rrc_span',12,'snr_db',Inf,'timing_offset_sym',0.35,'clk_ppm',0, 'sig_scale',sig_scale,'seed',47);
};

fprintf('[4] Closed-Loop Stage 1 (Float) vs Stage 2 (Fixed, W_mu=%d) Comparison:\n', W_mu_sel);
fprintf('    %-18s | Fl Acq / Fx Acq | Fl RMS / Fx RMS (Ts) | Sym RMS Err | TED RMS Err | PI RMS Err  | mu RMS Err\n', 'Test Case');
fprintf('    %s\n', repmat('-', 1, 112));

saved_cases = cell(size(cases, 1), 1);
for ci = 1:size(cases, 1)
    lbl = cases{ci, 1};
    sc  = cases{ci, 2};
    sig = generate_str_signal(sc);
    skip = sc.rrc_span * sc.sps;
    rx_trim = sig.rx_mf(skip + 1 : end);
    iv = true(length(rx_trim), 1);

    % Stage 1 Float run
    r_fl = str_float_model(real(rx_trim), imag(rx_trim), iv, fl_cfg);
    r_fl.sym_sample_idx = r_fl.sym_sample_idx + skip;
    m_fl = compute_metrics(sig, r_fl.sym_sample_idx, r_fl.sym_mu, r_fl.out_sym);

    % Stage 2 Fixed run (Q1.11 quantized inputs)
    in_i_q11 = fx_sat(round(real(rx_trim) * 2048), -2048, 2047);
    in_q_q11 = fx_sat(round(imag(rx_trim) * 2048), -2048, 2047);
    r_fx = str_fixed_model(in_i_q11, in_q_q11, iv, fx_cfg);
    r_fx.sym_sample_idx = r_fx.sym_sample_idx + skip;
    m_fx = compute_metrics(sig, r_fx.sym_sample_idx, r_fx.sym_mu_float, r_fx.out_sym_float);

    % Align symbol streams for direct difference metrics
    L = min(length(r_fl.out_sym), length(r_fx.out_sym_float));
    ss = min(300, floor(L/2));
    sym_err = sqrt(mean(abs(r_fx.out_sym_float(ss:L) - r_fl.out_sym(ss:L)).^2));
    ted_err = sqrt(mean((r_fx.ted_err_float(ss:L) - r_fl.ted_err(ss:L)).^2));
    pi_err  = sqrt(mean((r_fx.pi_out_float(ss:L) - r_fl.pi_out(ss:L)).^2));
    % Wrap-aware mu difference (accounts for mod-1 sample boundary)
    dmu = r_fx.sym_mu_float(ss:L) - r_fl.sym_mu(ss:L);
    dmu = dmu - round(dmu);
    mu_err  = sqrt(mean(dmu.^2));

    fprintf('    %-18s | %4d   / %4d   | %.5f / %.5f    | %.3e   | %.3e   | %.3e   | %.3e\n', ...
        lbl, m_fl.acq_syms, m_fx.acq_syms, m_fl.rms_jitter_ts, m_fx.rms_jitter_ts, ...
        sym_err, ted_err, pi_err, mu_err);

    saved_cases{ci} = struct('lbl',lbl,'sig',sig,'r_fl',r_fl,'r_fx',r_fx,'m_fl',m_fl,'m_fx',m_fx);
end

%% 4. Generate Stage 2 Float vs Fixed Comparison Plots
% Plot 1: Timing Error Trajectory (Float vs Fixed) for QPSK & 16-QAM
f1 = figure('Visible','off','Position',[100 100 960 460]);
subplot(2,1,1);
c1 = saved_cases{1};
plot(c1.m_fl.tau_err_ts(1:600), 'b-', 'LineWidth', 1.2); hold on; grid on;
plot(c1.m_fx.tau_err_ts(1:600), 'r--', 'LineWidth', 1.0);
ylabel('Timing Error (T_s)');
title('Stage 2 Float vs Fixed Timing Error Comparison (SNR = 15 dB, W_\mu = 12)');
legend(sprintf('QPSK Float (RMS=%.4f T_s)', c1.m_fl.rms_jitter_ts), ...
       sprintf('QPSK Fixed (RMS=%.4f T_s)', c1.m_fx.rms_jitter_ts), 'Location', 'best');
subplot(2,1,2);
c2 = saved_cases{2};
plot(c2.m_fl.tau_err_ts(1:600), 'b-', 'LineWidth', 1.2); hold on; grid on;
plot(c2.m_fx.tau_err_ts(1:600), 'r--', 'LineWidth', 1.0);
xlabel('Symbol Index k'); ylabel('Timing Error (T_s)');
legend(sprintf('16-QAM Float (RMS=%.4f T_s)', c2.m_fl.rms_jitter_ts), ...
       sprintf('16-QAM Fixed (RMS=%.4f T_s)', c2.m_fx.rms_jitter_ts), 'Location', 'best');
saveas(f1, fullfile(out_dir, '01_float_vs_fixed_timing_error.png')); close(f1);

% Plot 2: Fractional Delay mu & Symbol Error (Float vs Fixed)
f2 = figure('Visible','off','Position',[100 100 960 460]);
c3 = saved_cases{3};
L3 = min(length(c3.r_fl.sym_mu), length(c3.r_fx.sym_mu_float));
subplot(2,1,1);
plot(c3.r_fl.sym_mu(1:L3), 'b-', 'LineWidth', 1.2); hold on; grid on;
plot(c3.r_fx.sym_mu_float(1:L3), 'r--', 'LineWidth', 1.0);
ylabel('\mu (samples)');
title('Stage 2 Float vs Fixed Fractional Delay (+200 ppm) & Symbol Error');
legend('Float \mu', 'Fixed \mu (UQ0.12)', 'Location', 'best');
subplot(2,1,2);
L1 = min(length(c1.r_fl.out_sym), length(c1.r_fx.out_sym_float));
plot(abs(c1.r_fx.out_sym_float(1:L1) - c1.r_fl.out_sym(1:L1)), 'k-'); grid on;
xlabel('Symbol Index k'); ylabel('|y_{fixed} - y_{float}|');
legend('QPSK Complex Symbol Output Error (Q1.15 vs double)');
saveas(f2, fullfile(out_dir, '02_float_vs_fixed_mu_and_sym_err.png')); close(f2);

% Plot 3: Fixed-Point Constellations (QPSK & 16-QAM at SNR = 15 dB)
f3 = figure('Visible','off','Position',[100 100 980 460]);
subplot(1,2,1);
sq = c1.r_fx.out_sym_float(300:end-20);
plot(real(sq), imag(sq), '.', 'Color', [0.1 0.4 0.8], 'MarkerSize', 5); hold on; grid on; axis equal;
iq = sig_scale * [1+1j, 1-1j, -1+1j, -1-1j] / sqrt(2);
plot(real(iq), imag(iq), 'rx', 'MarkerSize', 11, 'LineWidth', 2);
xlim([-0.8 0.8]); ylim([-0.8 0.8]);
xlabel('I (Q1.15)'); ylabel('Q (Q1.15)');
title(sprintf('Fixed-Point QPSK (RMS=%.4f T_s)', c1.m_fx.rms_jitter_ts));
subplot(1,2,2);
sm = c2.r_fx.out_sym_float(300:end-20);
plot(real(sm), imag(sm), '.', 'Color', [0.8 0.2 0.2], 'MarkerSize', 5); hold on; grid on; axis equal;
[gi, gq] = meshgrid(sig_scale*[-3 -1 1 3]/sqrt(10), sig_scale*[-3 -1 1 3]/sqrt(10));
plot(gi(:), gq(:), 'kx', 'MarkerSize', 9, 'LineWidth', 1.8);
xlim([-0.8 0.8]); ylim([-0.8 0.8]);
xlabel('I (Q1.15)'); ylabel('Q (Q1.15)');
title(sprintf('Fixed-Point 16-QAM (RMS=%.4f T_s)', c2.m_fx.rms_jitter_ts));
saveas(f3, fullfile(out_dir, '03_fixed_constellations.png')); close(f3);

report.kp4_fx = kp4_fx;
report.ki_fx  = ki_fx;
report.W_mu   = W_mu_sel;
fprintf('\nStage 2 validation plots saved to:\n  %s\n', out_dir);
end

function m = compute_metrics(sig, sym_idx, sym_mu, out_sym)
    eps_clk = sig.cfg.clk_ppm * 1e-6;
    Ns      = sig.cfg.sps;
    L_mf    = sig.mf_delay_samp;
    t_eff   = (((sym_idx - 2.0) + sym_mu) - L_mf) / Ns * (1.0 + eps_clk) + sig.cfg.timing_offset_sym;
    k_ideal = round(t_eff);
    tau_err = t_eff - k_ideal;
    valid_int = (k_ideal >= sig.cfg.rrc_span) & (k_ideal <= sig.cfg.num_symbols - sig.cfg.rrc_span - 1);
    tau_v = tau_err(valid_int);
    sym_v = out_sym(valid_int);
    ref_v = sig.tx_symbols(k_ideal(valid_int) + 1);

    acq = length(tau_v);
    for k = 1:(length(tau_v) - 20)
        if all(abs(tau_v(k : k + 19)) <= 0.05)
            acq = k; break;
        end
    end
    ss = min(300, floor(length(tau_v)/2));
    m.tau_err_ts    = tau_v;
    m.acq_syms      = acq;
    m.mean_err_ts   = mean(tau_v(ss:end));
    m.rms_jitter_ts = sqrt(mean(tau_v(ss:end).^2));
    m.evm_pct       = 100.0 * sqrt(mean(abs(sym_v(ss:end) - ref_v(ss:end)).^2) / mean(abs(ref_v(ss:end)).^2));
end
