function [tau_grid, scurve, Kd] = compute_ted_scurve(mod_type, alpha, num_symbols, seed)
% COMPUTE_TED_SCURVE Numerically evaluates the Gardner TED S-curve E[e|tau]
%   and measures the exact TED gain Kd = dS/dtau|_{tau=0} (in units of
%   TED output per symbol period Ts).
%
%   [tau_grid, scurve, Kd] = compute_ted_scurve(mod_type, alpha, num_symbols, seed)

if nargin < 1, mod_type    = 'QPSK'; end
if nargin < 2, alpha       = 0.35;   end
if nargin < 3, num_symbols = 4000;   end
if nargin < 4, seed        = 101;    end

tau_grid = linspace(-0.5, 0.5, 41);
scurve   = zeros(size(tau_grid));
Ns       = 4;
span     = 12;

for idx = 1:length(tau_grid)
    cfg.mod_type          = mod_type;
    cfg.num_symbols       = num_symbols;
    cfg.sps               = Ns;
    cfg.alpha             = alpha;
    cfg.rrc_span          = span;
    cfg.snr_db            = Inf;
    cfg.timing_offset_sym = tau_grid(idx);
    cfg.clk_ppm           = 0;
    cfg.sig_scale         = 1.0;
    cfg.seed              = seed;

    sig = generate_str_signal(cfg);
    rx  = sig.rx_mf;
    d   = sig.mf_delay_samp;

    % Sample at symbol indices k (skip edge transients)
    k_start = span + 2;
    k_end   = num_symbols - span - 2;
    k_vec   = (k_start : k_end).';

    idx_curr = d + k_vec * Ns + 1;          % 1-based MATLAB index
    idx_prev = d + (k_vec - 1) * Ns + 1;
    idx_mid  = d + k_vec * Ns - (Ns/2) + 1; % k - 1/2

    e_vec = gardner_ted_float( ...
        real(rx(idx_curr)), imag(rx(idx_curr)), ...
        real(rx(idx_prev)), imag(rx(idx_prev)), ...
        real(rx(idx_mid)),  imag(rx(idx_mid)) );

    scurve(idx) = mean(e_vec);
end

% Measure slope Kd around tau = 0 via central difference at +/- 0.025 Ts
[~, i0] = min(abs(tau_grid));
Kd = (scurve(i0 + 1) - scurve(i0 - 1)) / (tau_grid(i0 + 1) - tau_grid(i0 - 1));

end
