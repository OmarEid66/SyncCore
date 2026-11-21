%% Export Comprehensive Golden Test Vectors from MATLAB for SystemVerilog Verification
% Generates comprehensive vector sets exercising the complete feature set of the
% Coarse Carrier Frequency Offset (CFO) Estimator:
%   Case 1:  Zero CFO (delta_f = 0 Hz, SNR = 25 dB, Mode B)
%   Case 2:  Positive Moderate CFO (+0.05 Rsym = +1.5625 MHz, SNR = 20 dB, Mode B)
%   Case 3:  Negative Moderate CFO (-0.05 Rsym = -1.5625 MHz, SNR = 20 dB, Mode B)
%   Case 4:  Maximum Positive CFO Limit (+0.10 Rsym = +3.125 MHz, SNR = 18 dB, Mode B)
%   Case 5:  Maximum Negative CFO Limit (-0.10 Rsym = -3.125 MHz, SNR = 18 dB, Mode B)
%   Case 6:  Intermediate Positive CFO (+0.07 Rsym = +2.1875 MHz, SNR = 15 dB, Mode B)
%   Case 7:  Intermediate Negative CFO (-0.03 Rsym = -937.5 kHz, SNR = 15 dB, Mode B)
%   Case 8:  Low SNR Condition (+0.02 Rsym = +625 kHz, SNR = 6 dB, Mode B)
%   Case 9:  Autonomous Detection Mode with Noise Prefix (+0.03 Rsym, SNR = 12 dB, Mode A)
%   Case 10: Autonomous Detection Mode with High Negative CFO (-0.08 Rsym, SNR = 14 dB, Mode A)
%   Case 11: Streaming with Invalid-Sample Gaps (+0.04 Rsym, SNR = 20 dB, Mode B)

clear; clc;
rng(2026); % Fixed seed for deterministic repeatability

fprintf('========================================================================\n');
fprintf('  EXPORTING EXTENDED GOLDEN TEST VECTORS FOR SYSTEMVERILOG TESTBENCH    \n');
fprintf('========================================================================\n');

Fs   = 125.0e6;
Rsym = 31.25e6;
vec_dir = 'd:/SynchCore/tb/vectors/';

if ~exist(vec_dir, 'dir')
    mkdir(vec_dir);
end

L = 16;
num_repeats = 16; % 256 samples total
[zc_base, ~, ~] = generate_zc_preamble(L, 1, num_repeats, 1800);
t_base = (0:length(zc_base)-1)' / Fs;

params_ext.auto_mode_en = 0;
params_ext.L = 16;
params_ext.M = 192;

params_auto.auto_mode_en = 1;
params_auto.L = 16;
params_auto.M = 192;
params_auto.gamma_thresh = 0.55;
params_auto.energy_min = 1e6;
params_auto.N_consec = 2;

%% --- CASE 1: Zero CFO (0 Hz, SNR = 25 dB) ---
cfo1 = 0.0;
sig1 = zc_base .* exp(1j * 2 * pi * cfo1 * t_base);
sp1  = mean(abs(sig1).^2);
np1  = sp1 / (10^(25/10));
n1   = sqrt(np1/2) * (randn(size(sig1)) + 1j * randn(size(sig1)));
rx1  = sig1 + n1;
rx1_i = max(min(round(real(rx1)), 2047), -2048);
rx1_q = max(min(round(imag(rx1)), 2047), -2048);
rx1_vld = true(length(rx1), 1);
bs1 = false(length(rx1), 1); bs1(1) = true;
pe1 = false(length(rx1), 1); pe1(1) = true;
ae1 = zeros(length(rx1), 1);
export_case(1, 'Zero CFO (0 Hz, SNR = 25 dB)', rx1_i, rx1_q, rx1_vld, bs1, pe1, ae1, cfo1, params_ext, vec_dir, Fs);

%% --- CASE 2: Moderate Positive CFO (+0.05 Rsym = +1.5625 MHz, SNR = 20 dB) ---
cfo2 = +0.05 * Rsym;
sig2 = zc_base .* exp(1j * 2 * pi * cfo2 * t_base);
sp2  = mean(abs(sig2).^2);
np2  = sp2 / (10^(20/10));
n2   = sqrt(np2/2) * (randn(size(sig2)) + 1j * randn(size(sig2)));
rx2  = sig2 + n2;
rx2_i = max(min(round(real(rx2)), 2047), -2048);
rx2_q = max(min(round(imag(rx2)), 2047), -2048);
rx2_vld = true(length(rx2), 1);
bs2 = false(length(rx2), 1); bs2(1) = true;
pe2 = false(length(rx2), 1); pe2(1) = true;
ae2 = zeros(length(rx2), 1);
export_case(2, 'Moderate Positive CFO (+0.05 Rsym, SNR = 20 dB)', rx2_i, rx2_q, rx2_vld, bs2, pe2, ae2, cfo2, params_ext, vec_dir, Fs);

%% --- CASE 3: Moderate Negative CFO (-0.05 Rsym = -1.5625 MHz, SNR = 20 dB) ---
cfo3 = -0.05 * Rsym;
sig3 = zc_base .* exp(1j * 2 * pi * cfo3 * t_base);
sp3  = mean(abs(sig3).^2);
np3  = sp3 / (10^(20/10));
n3   = sqrt(np3/2) * (randn(size(sig3)) + 1j * randn(size(sig3)));
rx3  = sig3 + n3;
rx3_i = max(min(round(real(rx3)), 2047), -2048);
rx3_q = max(min(round(imag(rx3)), 2047), -2048);
rx3_vld = true(length(rx3), 1);
bs3 = false(length(rx3), 1); bs3(1) = true;
pe3 = false(length(rx3), 1); pe3(1) = true;
ae3 = zeros(length(rx3), 1);
export_case(3, 'Moderate Negative CFO (-0.05 Rsym, SNR = 20 dB)', rx3_i, rx3_q, rx3_vld, bs3, pe3, ae3, cfo3, params_ext, vec_dir, Fs);

%% --- CASE 4: Maximum Positive CFO Limit (+0.10 Rsym = +3.125 MHz, SNR = 18 dB) ---
cfo4 = +0.10 * Rsym;
sig4 = zc_base .* exp(1j * 2 * pi * cfo4 * t_base);
sp4  = mean(abs(sig4).^2);
np4  = sp4 / (10^(18/10));
n4   = sqrt(np4/2) * (randn(size(sig4)) + 1j * randn(size(sig4)));
rx4  = sig4 + n4;
rx4_i = max(min(round(real(rx4)), 2047), -2048);
rx4_q = max(min(round(imag(rx4)), 2047), -2048);
rx4_vld = true(length(rx4), 1);
bs4 = false(length(rx4), 1); bs4(1) = true;
pe4 = false(length(rx4), 1); pe4(1) = true;
ae4 = zeros(length(rx4), 1);
export_case(4, 'Max Positive CFO (+0.10 Rsym, SNR = 18 dB)', rx4_i, rx4_q, rx4_vld, bs4, pe4, ae4, cfo4, params_ext, vec_dir, Fs);

%% --- CASE 5: Maximum Negative CFO Limit (-0.10 Rsym = -3.125 MHz, SNR = 18 dB) ---
cfo5 = -0.10 * Rsym;
sig5 = zc_base .* exp(1j * 2 * pi * cfo5 * t_base);
sp5  = mean(abs(sig5).^2);
np5  = sp5 / (10^(18/10));
n5   = sqrt(np5/2) * (randn(size(sig5)) + 1j * randn(size(sig5)));
rx5  = sig5 + n5;
rx5_i = max(min(round(real(rx5)), 2047), -2048);
rx5_q = max(min(round(imag(rx5)), 2047), -2048);
rx5_vld = true(length(rx5), 1);
bs5 = false(length(rx5), 1); bs5(1) = true;
pe5 = false(length(rx5), 1); pe5(1) = true;
ae5 = zeros(length(rx5), 1);
export_case(5, 'Max Negative CFO (-0.10 Rsym, SNR = 18 dB)', rx5_i, rx5_q, rx5_vld, bs5, pe5, ae5, cfo5, params_ext, vec_dir, Fs);

%% --- CASE 6: Intermediate Positive CFO (+0.07 Rsym = +2.1875 MHz, SNR = 15 dB) ---
cfo6 = +0.07 * Rsym;
sig6 = zc_base .* exp(1j * 2 * pi * cfo6 * t_base);
sp6  = mean(abs(sig6).^2);
np6  = sp6 / (10^(15/10));
n6   = sqrt(np6/2) * (randn(size(sig6)) + 1j * randn(size(sig6)));
rx6  = sig6 + n6;
rx6_i = max(min(round(real(rx6)), 2047), -2048);
rx6_q = max(min(round(imag(rx6)), 2047), -2048);
rx6_vld = true(length(rx6), 1);
bs6 = false(length(rx6), 1); bs6(1) = true;
pe6 = false(length(rx6), 1); pe6(1) = true;
ae6 = zeros(length(rx6), 1);
export_case(6, 'Intermediate Positive CFO (+0.07 Rsym, SNR = 15 dB)', rx6_i, rx6_q, rx6_vld, bs6, pe6, ae6, cfo6, params_ext, vec_dir, Fs);

%% --- CASE 7: Intermediate Negative CFO (-0.03 Rsym = -937.5 kHz, SNR = 15 dB) ---
cfo7 = -0.03 * Rsym;
sig7 = zc_base .* exp(1j * 2 * pi * cfo7 * t_base);
sp7  = mean(abs(sig7).^2);
np7  = sp7 / (10^(15/10));
n7   = sqrt(np7/2) * (randn(size(sig7)) + 1j * randn(size(sig7)));
rx7  = sig7 + n7;
rx7_i = max(min(round(real(rx7)), 2047), -2048);
rx7_q = max(min(round(imag(rx7)), 2047), -2048);
rx7_vld = true(length(rx7), 1);
bs7 = false(length(rx7), 1); bs7(1) = true;
pe7 = false(length(rx7), 1); pe7(1) = true;
ae7 = zeros(length(rx7), 1);
export_case(7, 'Intermediate Negative CFO (-0.03 Rsym, SNR = 15 dB)', rx7_i, rx7_q, rx7_vld, bs7, pe7, ae7, cfo7, params_ext, vec_dir, Fs);

%% --- CASE 8: Low SNR Condition (+0.02 Rsym = +625 kHz, SNR = 6 dB) ---
cfo8 = +0.02 * Rsym;
sig8 = zc_base .* exp(1j * 2 * pi * cfo8 * t_base);
sp8  = mean(abs(sig8).^2);
np8  = sp8 / (10^(6/10));
n8   = sqrt(np8/2) * (randn(size(sig8)) + 1j * randn(size(sig8)));
rx8  = sig8 + n8;
rx8_i = max(min(round(real(rx8)), 2047), -2048);
rx8_q = max(min(round(imag(rx8)), 2047), -2048);
rx8_vld = true(length(rx8), 1);
bs8 = false(length(rx8), 1); bs8(1) = true;
pe8 = false(length(rx8), 1); pe8(1) = true;
ae8 = zeros(length(rx8), 1);
export_case(8, 'Low SNR Condition (+0.02 Rsym, SNR = 6 dB)', rx8_i, rx8_q, rx8_vld, bs8, pe8, ae8, cfo8, params_ext, vec_dir, Fs);

%% --- CASE 9: Autonomous Detection with Noise Prefix (+0.03 Rsym = +937.5 kHz, SNR = 12 dB) ---
cfo9 = +0.03 * Rsym;
noise_len9 = 64;
noise9_i = round(randn(noise_len9, 1) * 150);
noise9_q = round(randn(noise_len9, 1) * 150);
sig9 = zc_base .* exp(1j * 2 * pi * cfo9 * t_base);
sp9  = mean(abs(sig9).^2);
np9  = sp9 / (10^(12/10));
n9   = sqrt(np9/2) * (randn(size(sig9)) + 1j * randn(size(sig9)));
rx9  = sig9 + n9;
rx9_i = [noise9_i; max(min(round(real(rx9)), 2047), -2048); zeros(64,1)];
rx9_q = [noise9_q; max(min(round(imag(rx9)), 2047), -2048); zeros(64,1)];
rx9_vld = true(length(rx9_i), 1);
bs9 = false(length(rx9_i), 1); bs9(1) = true;
pe9 = false(length(rx9_i), 1);
ae9 = ones(length(rx9_i), 1);
export_case(9, 'Autonomous Mode A (+0.03 Rsym, SNR = 12 dB)', rx9_i, rx9_q, rx9_vld, bs9, pe9, ae9, cfo9, params_auto, vec_dir, Fs);

%% --- CASE 10: Autonomous Detection High Negative CFO (-0.08 Rsym = -2.50 MHz, SNR = 14 dB) ---
cfo10 = -0.08 * Rsym;
noise_len10 = 48;
noise10_i = round(randn(noise_len10, 1) * 150);
noise10_q = round(randn(noise_len10, 1) * 150);
sig10 = zc_base .* exp(1j * 2 * pi * cfo10 * t_base);
sp10  = mean(abs(sig10).^2);
np10  = sp10 / (10^(14/10));
n10   = sqrt(np10/2) * (randn(size(sig10)) + 1j * randn(size(sig10)));
rx10  = sig10 + n10;
rx10_i = [noise10_i; max(min(round(real(rx10)), 2047), -2048); zeros(64,1)];
rx10_q = [noise10_q; max(min(round(imag(rx10)), 2047), -2048); zeros(64,1)];
rx10_vld = true(length(rx10_i), 1);
bs10 = false(length(rx10_i), 1); bs10(1) = true;
pe10 = false(length(rx10_i), 1);
ae10 = ones(length(rx10_i), 1);
export_case(10, 'Autonomous Mode A (-0.08 Rsym, SNR = 14 dB)', rx10_i, rx10_q, rx10_vld, bs10, pe10, ae10, cfo10, params_auto, vec_dir, Fs);

%% --- CASE 11: Streaming with Invalid-Sample Gaps (+0.04 Rsym = +1.25 MHz, SNR = 20 dB) ---
cfo11 = +0.04 * Rsym;
sig11 = zc_base .* exp(1j * 2 * pi * cfo11 * t_base);
sp11  = mean(abs(sig11).^2);
np11  = sp11 / (10^(20/10));
n11   = sqrt(np11/2) * (randn(size(sig11)) + 1j * randn(size(sig11)));
rx11  = sig11 + n11;
rx11_i_sub = max(min(round(real(rx11)), 2047), -2048);
rx11_q_sub = max(min(round(imag(rx11)), 2047), -2048);

% Interleave invalid sample gaps (valid on every second clock)
total_gapped_len = length(rx11_i_sub) * 2;
rx11_i = zeros(total_gapped_len, 1);
rx11_q = zeros(total_gapped_len, 1);
rx11_vld = false(total_gapped_len, 1);
rx11_i(1:2:end) = rx11_i_sub;
rx11_q(1:2:end) = rx11_q_sub;
rx11_vld(1:2:end) = true;

bs11 = false(total_gapped_len, 1); bs11(1) = true;
pe11 = false(total_gapped_len, 1); pe11(1) = true;
ae11 = zeros(total_gapped_len, 1);
export_case(11, 'Streaming with 50% Gaps (+0.04 Rsym, SNR = 20 dB)', rx11_i, rx11_q, rx11_vld, bs11, pe11, ae11, cfo11, params_ext, vec_dir, Fs);

fprintf('========================================================================\n');
fprintf('  ALL 11 VECTOR SETS SUCCESSFULLY GENERATED & EXPORTED WITH GOLDEN REFS \n');
fprintf('========================================================================\n');

%% Helper function to export one vector pair (placed at end of script)
function export_case(case_num, case_name, rx_i, rx_q, rx_vld, b_start, p_en, auto_en, actual_cfo, params, vec_dir, Fs)
    [est_q23, vld, ftw, lat_sym, ~] = cfo_coarse_golden_model(rx_i, rx_q, rx_vld, b_start, p_en, params);
    
    idx = find(vld, 1);
    if isempty(idx)
        error('Error in Case %d (%s): Golden model did not produce valid strobe!', case_num, case_name);
    end
    
    matlab_est_q23 = est_q23(idx);
    matlab_ftw     = ftw(idx);
    matlab_cfo_hz  = double(matlab_est_q23) / 8388608.0 * (Fs / 2.0);
    err_hz         = abs(matlab_cfo_hz - actual_cfo);
    
    % Write stimulus file
    stim_path = sprintf('%sstimulus_case%d.txt', vec_dir, case_num);
    fid_stim = fopen(stim_path, 'w');
    for k = 1:length(rx_i)
        fprintf(fid_stim, '%d %d %d %d %d %d\n', rx_i(k), rx_q(k), rx_vld(k), b_start(k), p_en(k), auto_en(k));
    end
    fclose(fid_stim);
    
    % Write expected reference file
    exp_path = sprintf('%sexpected_case%d.txt', vec_dir, case_num);
    fid_exp = fopen(exp_path, 'w');
    fprintf(fid_exp, 'cfo_est=%d\n', matlab_est_q23);
    fprintf(fid_exp, 'nco_ftw=%d\n', matlab_ftw);
    fprintf(fid_exp, 'actual_cfo_hz=%.2f\n', actual_cfo);
    fprintf(fid_exp, 'matlab_cfo_hz=%.2f\n', matlab_cfo_hz);
    fprintf(fid_exp, 'valid_sample_idx=%d\n', idx);
    fclose(fid_exp);
    
    fprintf('  [Case %2d] %-48s | Golden Q1.23: %8d | Est: %8.2f kHz | Err: %6.2f kHz\n', ...
        case_num, case_name, matlab_est_q23, matlab_cfo_hz/1e3, err_hz/1e3);
end
