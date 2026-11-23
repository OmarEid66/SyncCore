function farrow_rep = validate_farrow_float(alpha)
% VALIDATE_FARROW_FLOAT Validates the Cubic Lagrange Farrow interpolator
%   accuracy against exact analytical Raised-Cosine pulses and multi-tone
%   bandlimited baseband signals at 4x oversampling (Ns = 4).
%
%   farrow_rep = validate_farrow_float(alpha)

if nargin < 1, alpha = 0.35; end

Ns = 4;
mu_grid = linspace(0, 1 - 1e-4, 200);

% 1. Exact analytical Raised-Cosine pulse (RRC * RRC = RC)
rc_pulse = @(u) (abs(abs(2*alpha*u) - 1) < 1e-10) .* (pi/4 * sinc_local(1/(2*alpha))) + ...
                (abs(abs(2*alpha*u) - 1) >= 1e-10) .* (sinc_local(u) .* cos(pi*alpha*u) ./ max(1 - (2*alpha*u).^2, 1e-15));

% Evaluate across an interval of 40 symbols around peak
m_base = -20:20;
err_rc = zeros(length(m_base), length(mu_grid));
for mi = 1:length(m_base)
    m = m_base(mi);
    xm1 = rc_pulse((m - 1) / Ns);
    x0  = rc_pulse((m    ) / Ns);
    x1  = rc_pulse((m + 1) / Ns);
    x2  = rc_pulse((m + 2) / Ns);
    y_farrow = farrow_cubic_float(xm1, x0, x1, x2, mu_grid);
    y_exact  = rc_pulse((m + mu_grid) / Ns);
    err_rc(mi, :) = y_farrow - y_exact;
end

farrow_rep.max_abs_err_rc = max(abs(err_rc(:)));
farrow_rep.rms_err_rc     = sqrt(mean(err_rc(:).^2));
farrow_rep.sqnr_db_rc     = 20 * log10(sqrt(mean(rc_pulse(m_base/Ns).^2)) / farrow_rep.rms_err_rc);
farrow_rep.mu_grid        = mu_grid;
farrow_rep.max_err_vs_mu  = max(abs(err_rc), [], 1);

% 2. Bandlimited random QPSK matched-filtered signal test
cfg.mod_type = 'QPSK'; cfg.num_symbols = 500; cfg.sps = Ns;
cfg.alpha = alpha; cfg.rrc_span = 12; cfg.snr_db = Inf;
cfg.timing_offset_sym = 0.0; cfg.clk_ppm = 0; cfg.seed = 77;
sig0 = generate_str_signal(cfg);

% Exact shifted signal at fractional sample offset mu_test = 0.37 (0.0925 Ts)
mu_test = 0.37;
cfg.timing_offset_sym = mu_test / Ns;
sig_ref = generate_str_signal(cfg);

idx_range = (sig0.mf_delay_samp + 40) : (length(sig0.rx_mf) - sig0.mf_delay_samp - 40);
x = sig0.rx_mf;
y_i = farrow_cubic_float(real(x(idx_range-1)), real(x(idx_range)), real(x(idx_range+1)), real(x(idx_range+2)), mu_test);
y_q = farrow_cubic_float(imag(x(idx_range-1)), imag(x(idx_range)), imag(x(idx_range+1)), imag(x(idx_range+2)), mu_test);
y_c = y_i + 1j * y_q;
ref_c = sig_ref.rx_mf(idx_range);
diff_sig = y_c - ref_c;

farrow_rep.sig_rms_err = sqrt(mean(abs(diff_sig).^2));
farrow_rep.sig_max_err = max(abs(diff_sig));
farrow_rep.sig_snr_db  = 10 * log10(mean(abs(ref_c).^2) / mean(abs(diff_sig).^2));

end

function y = sinc_local(x)
% Normalized sinc(x) = sin(pi*x)/(pi*x) without Signal Processing Toolbox
y = ones(size(x));
nz = abs(x) > 1e-12;
y(nz) = sin(pi * x(nz)) ./ (pi * x(nz));
end
