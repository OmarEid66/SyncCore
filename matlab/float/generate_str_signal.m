function sig = generate_str_signal(cfg)
% GENERATE_STR_SIGNAL Generates 4x-oversampled RRC-filtered QPSK / 16-QAM
%   baseband I/Q signal with exact fractional timing offset, sampling clock
%   mismatch (ppm), and calibrated AWGN.
%
%   sig = generate_str_signal(cfg)
%
%   cfg fields (with defaults):
%       mod_type          : 'QPSK' or '16QAM' (default 'QPSK')
%       num_symbols       : Number of transmitted symbols (default 2000)
%       sps               : Oversampling factor (default 4)
%       alpha             : RRC roll-off factor (default 0.35)
%       rrc_span          : RRC filter span in symbols (default 12)
%       snr_db            : Symbol SNR Es/N0 in dB, Inf for noiseless (default 15)
%       timing_offset_sym : Initial fractional timing offset in Ts (default 0.25)
%       clk_ppm           : Sampling clock offset in ppm (default 0)
%       sig_scale         : Overall signal amplitude scale (default 1.0)
%       seed              : RNG seed (default 42)
%
%   Output struct sig:
%       rx_mf             : Complex matched-filter output (input to STR)
%       rx_adc            : Complex ADC samples before RX matched filter
%       tx_symbols        : Complex unit-energy transmitted symbols (scaled by sig_scale)
%       h_rrc             : Unit-energy discrete RRC filter taps
%       mf_delay_samp     : Matched filter group delay in samples (rrc_span*sps/2)
%       t_mf_sym          : Exact continuous time (in Ts) of each rx_mf sample
%       cfg               : Effective configuration struct

if ~isfield(cfg, 'mod_type'),          cfg.mod_type = 'QPSK'; end
if ~isfield(cfg, 'num_symbols'),       cfg.num_symbols = 2000; end
if ~isfield(cfg, 'sps'),               cfg.sps = 4; end
if ~isfield(cfg, 'alpha'),             cfg.alpha = 0.35; end
if ~isfield(cfg, 'rrc_span'),          cfg.rrc_span = 12; end
if ~isfield(cfg, 'snr_db'),            cfg.snr_db = 15; end
if ~isfield(cfg, 'timing_offset_sym'), cfg.timing_offset_sym = 0.25; end
if ~isfield(cfg, 'clk_ppm'),           cfg.clk_ppm = 0; end
if ~isfield(cfg, 'sig_scale'),         cfg.sig_scale = 1.0; end
if ~isfield(cfg, 'seed'),              cfg.seed = 42; end

try
    rng(cfg.seed);
catch
    rand('state', cfg.seed);
    randn('state', cfg.seed);
end

N_sym = cfg.num_symbols;
Ns    = cfg.sps;
alpha = cfg.alpha;
span  = cfg.rrc_span;
half_span = span / 2;

% 1. Generate unit-average-energy symbols (Es = 1)
switch upper(cfg.mod_type)
    case 'QPSK'
        bits_i = 2 * randi([0, 1], N_sym, 1) - 1;
        bits_q = 2 * randi([0, 1], N_sym, 1) - 1;
        tx_sym = (bits_i + 1j * bits_q) / sqrt(2.0);
    case '16QAM'
        levels = [-3, -1, 1, 3];
        idx_i  = randi([1, 4], N_sym, 1);
        idx_q  = randi([1, 4], N_sym, 1);
        tx_sym = (levels(idx_i).' + 1j * levels(idx_q).') / sqrt(10.0);
    otherwise
        error('Unsupported mod_type: %s. Use ''QPSK'' or ''16QAM''.', cfg.mod_type);
end

% 2. Build discrete unit-energy RRC matched filter
mf_delay_samp = (span * Ns) / 2;
m_grid = (-mf_delay_samp : mf_delay_samp).' / Ns;
h_raw  = rrc_pulse(m_grid, alpha);
norm_f = sqrt(sum(h_raw.^2));
h_rrc  = h_raw / norm_f;

% 3. Exact continuous-time TX waveform evaluated at ADC instants
%    t_adc[n] = (n / Ns) * (1 + eps_clk) + timing_offset_sym  (n = 0..N_samp-1)
eps_clk = cfg.clk_ppm * 1e-6;
N_samp  = N_sym * Ns;
n_vec   = (0 : N_samp - 1).';
t_adc   = (n_vec / Ns) * (1.0 + eps_clk) + cfg.timing_offset_sym;

% Pad symbol array with zeros on both sides for finite-span pulse shaping
pad = half_span + 2;
tx_pad = [zeros(pad, 1); tx_sym; zeros(pad, 1)];

% For each sample n, sum over k in [floor(t_adc)-half_span .. floor(t_adc)+half_span+1]
k_center = floor(t_adc);
k_offs   = (-half_span : half_span + 1);  % 1 x (span+2)

rx_clean = zeros(N_samp, 1);
for j = 1:length(k_offs)
    k_idx = k_center + k_offs(j);          % 0-based symbol index
    u_val = t_adc - k_idx;                 % normalized time offset
    in_span = abs(u_val) <= (half_span + 1e-9);
    pulse_val = zeros(N_samp, 1);
    pulse_val(in_span) = rrc_pulse(u_val(in_span), alpha) / norm_f;
    arr_idx = k_idx + pad + 1;             % 1-based index into tx_pad
    valid_k = (arr_idx >= 1) & (arr_idx <= length(tx_pad));
    sym_val = zeros(N_samp, 1);
    sym_val(valid_k) = tx_pad(arr_idx(valid_k));
    rx_clean = rx_clean + sym_val .* pulse_val;
end

% 4. Add calibrated complex AWGN at ADC rate (Es/N0 = snr_db after MF)
if isfinite(cfg.snr_db)
    n0 = 10.0^(-cfg.snr_db / 10.0);
    sigma = sqrt(n0 / 2.0);
    noise = sigma * (randn(N_samp, 1) + 1j * randn(N_samp, 1));
    rx_adc = rx_clean + noise;
else
    rx_adc = rx_clean;
end

% Apply optional amplitude scaling (for fixed-point headroom analysis)
rx_adc = cfg.sig_scale * rx_adc;
tx_sym_scaled = cfg.sig_scale * tx_sym;

% 5. Apply RX matched RRC filter (causal convolution, keep same length)
rx_mf_full = conv(rx_adc, h_rrc);
rx_mf = rx_mf_full(1:N_samp);

% Exact continuous time (in symbol periods Ts) for each rx_mf sample (0-based n)
t_mf_sym = ((n_vec - mf_delay_samp) / Ns) * (1.0 + eps_clk) + cfg.timing_offset_sym;

sig.rx_mf         = rx_mf;
sig.rx_adc        = rx_adc;
sig.tx_symbols    = tx_sym_scaled;
sig.h_rrc         = h_rrc;
sig.mf_delay_samp = mf_delay_samp;
sig.t_mf_sym      = t_mf_sym;
sig.cfg           = cfg;

end
