function [zc_samples, zc_i, zc_q] = generate_zc_preamble(N_zc, u, num_repeats, scale_factor)
% GENERATE_ZC_PREAMBLE Generates a periodic Zadoff-Chu training sequence
%
% Inputs:
%   N_zc         - Length of the base sequence (default: 16)
%   u            - Root index, coprime to N_zc (default: 1)
%   num_repeats  - Number of repetitions (default: 16 -> 256 samples = 64 symbols)
%   scale_factor - Amplitude scaling in Q1.11 (default: 1800 out of 2047)
%
% Outputs:
%   zc_samples   - Complex floating point samples
%   zc_i, zc_q   - Signed 12-bit integer arrays (Q1.11)

    if nargin < 1, N_zc = 16; end
    if nargin < 2, u = 1; end
    if nargin < 3, num_repeats = 16; end
    if nargin < 4, scale_factor = 1800; end

    k = (0:N_zc-1)';
    % Zadoff-Chu sequence definition for even N_zc
    phase = -pi * u * (k .* (k + 1)) / N_zc;
    base_zc = exp(1j * phase);

    % Repeat to form the continuous burst preamble
    zc_samples = repmat(base_zc, num_repeats, 1) * scale_factor;

    % Quantize to signed Q1.11 (12 bits: -2048 to +2047)
    zc_i = round(real(zc_samples));
    zc_q = round(imag(zc_samples));

    % Saturate to signed 12-bit range [-2048, 2047]
    zc_i = max(min(zc_i, 2047), -2048);
    zc_q = max(min(zc_q, 2047), -2048);
end
