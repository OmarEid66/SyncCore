function res = str_float_model(rx_i, rx_q, in_valid, cfg)
% STR_FLOAT_MODEL Complete floating-point (double) streaming Symbol Timing
%   Recovery (STR) model.
%
%   res = str_float_model(rx_i, rx_q, in_valid, cfg)
%
%   Architecture (1 complex sample per clock when in_valid == 1):
%     Input Shift Register (4 taps) -> Cubic Farrow Interpolator (I & Q) ->
%     Timing Sample Extraction (Alternating Midpoint / Symbol) ->
%     Gardner TED -> PI Loop Filter ->
%     Decrementing Mod-1 Timing Phase Accumulator / Scheduler
%
%   Timing Phase Accumulator Update (bias-free Rice/Gardner NCO):
%     - Integral path u_i = Ki * I[k] sets the continuous NCO frequency word
%       W_freq = W_nom + u_i (tracks sampling clock mismatch in ppm).
%     - Proportional path u_p = Ns * Kp * e[k] applies a one-shot phase
%       adjustment once per symbol (after the midpoint strobe), eliminating
%       3-vs-5-clock boundary rectification bias and midpoint cross-correlation.
%     - Total phase advance per symbol = 4*(Kp*e[k] + Ki*I[k]) half-symbols
%       = 2*(Kp*e[k] + Ki*I[k]) symbol periods => K0 = 2.0 Ts/unit-u.
%
%   cfg fields:
%       Kp              : Proportional gain (from design_pi_gains)
%       Ki              : Integral gain (from design_pi_gains)
%       sps             : Oversampling factor (default 4)
%       exact_mu        : true -> mu = eta_k/W_freq, false -> mu = clamp(2*eta_k,0,1)
%                         (default true)
%       init_eta        : Initial NCO phase in [0,1) (default 0.5)
%       loop_delay_clks : Optional feedback pipeline delay in clocks (default 0)
%       u_clamp         : Saturation limit on |u_p| and |u_i| (default 0.25)

if ~isfield(cfg, 'sps'),             cfg.sps = 4; end
if ~isfield(cfg, 'exact_mu'),        cfg.exact_mu = true; end
if ~isfield(cfg, 'init_eta'),        cfg.init_eta = 0.5; end
if ~isfield(cfg, 'loop_delay_clks'), cfg.loop_delay_clks = 0; end
if ~isfield(cfg, 'u_clamp'),         cfg.u_clamp = 0.25; end

N     = length(rx_i);
Ns    = cfg.sps;
W_nom = 2.0 / Ns;   % 0.5 for Ns=4 => 2 strobes/symbol (midpoint + symbol)

% Preallocate traces
max_syms = ceil(N / (Ns / 2)) + 16;
out_sym_i      = zeros(max_syms, 1);
out_sym_q      = zeros(max_syms, 1);
sym_sample_idx = zeros(max_syms, 1);  % 0-based input sample index n
sym_mu         = zeros(max_syms, 1);
ted_err        = zeros(max_syms, 1);
pi_out         = zeros(max_syms, 1);
pi_integ_log   = zeros(max_syms, 1);
out_valid_clk  = false(N, 1);

eta_trace      = zeros(N, 1);
w_trace        = zeros(N, 1);
strobe_trace   = false(N, 1);
mu_all_trace   = nan(N, 1);

% 4-sample shift registers: [x[n], x[n-1], x[n-2], x[n-3]] = [x2, x1, x0, xm1]
sr_i = zeros(4, 1);
sr_q = zeros(4, 1);
valid_count = 0;

eta              = cfg.init_eta;
is_symbol_strobe = false;   % Alternates: midpoint (false) -> symbol (true)
has_mid          = false;
has_prev_sym     = false;

mid_i      = 0.0; mid_q      = 0.0;
prev_sym_i = 0.0; prev_sym_q = 0.0;

pi_integ       = 0.0;
u_i_curr       = 0.0;
kick_after_mid = 0.0;

% Optional feedback pipeline delay (for Stage 3 latency evaluation)
D_loop = cfg.loop_delay_clks;
if D_loop > 0
    pipe_p = zeros(D_loop, 1);
    pipe_i = zeros(D_loop, 1);
    pipe_v = false(D_loop, 1);
end

sym_cnt = 0;

for idx = 1:N
    if ~in_valid(idx)
        eta_trace(idx) = eta;
        w_trace(idx)   = W_nom + u_i_curr;
        continue;
    end

    % 1. Shift incoming sample into 4-tap Farrow delay line
    sr_i = [rx_i(idx); sr_i(1:3)];
    sr_q = [rx_q(idx); sr_q(1:3)];
    valid_count = valid_count + 1;

    if valid_count < 4
        eta_trace(idx) = eta;
        w_trace(idx)   = W_nom;
        continue;
    end

    % Advance optional feedback delay pipeline
    if D_loop > 0
        if pipe_v(end)
            kick_after_mid = pipe_p(end);
            u_i_curr       = pipe_i(end);
        end
        pipe_p = [0.0;      pipe_p(1:end-1)];
        pipe_i = [u_i_curr; pipe_i(1:end-1)];
        pipe_v = [false;    pipe_v(1:end-1)];
    end

    % 2. Smooth NCO frequency word + one-shot proportional phase adjustment
    W_freq = min(max(W_nom + u_i_curr, 0.25), 0.75);
    kick   = 0.0;
    if is_symbol_strobe && (kick_after_mid ~= 0.0)
        kick = kick_after_mid;
        kick_after_mid = 0.0;
    end

    eta_k   = eta - kick;
    eta_sub = eta_k - W_freq;

    eta_trace(idx) = eta;
    w_trace(idx)   = W_freq;

    % 3. Mod-1 Underflow Check -> Strobe & Fractional Delay mu
    if eta_sub < 0.0
        strobe_trace(idx) = true;
        if cfg.exact_mu
            mu = eta_k / W_freq;
        else
            mu = 2.0 * eta_k;
        end
        mu = min(max(mu, 0.0), 1.0 - 1e-12);
        mu_all_trace(idx) = mu;

        eta = eta_sub + 1.0;

        % 4. Cubic Farrow Interpolator (Horner's form, I & Q independently)
        y_i = farrow_cubic_float(sr_i(4), sr_i(3), sr_i(2), sr_i(1), mu);
        y_q = farrow_cubic_float(sr_q(4), sr_q(3), sr_q(2), sr_q(1), mu);

        % 5. Timing Sample Extraction (Midpoint vs Symbol)
        if ~is_symbol_strobe
            mid_i   = y_i;
            mid_q   = y_q;
            has_mid = true;
            is_symbol_strobe = true;
        else
            sym_cnt = sym_cnt + 1;
            out_sym_i(sym_cnt)      = y_i;
            out_sym_q(sym_cnt)      = y_q;
            sym_sample_idx(sym_cnt) = idx - 1;   % 0-based input index n
            sym_mu(sym_cnt)         = mu;
            out_valid_clk(idx)      = true;

            % 6. Gardner TED + PI Loop Filter (updated once per symbol)
            if has_prev_sym && has_mid
                e = gardner_ted_float(y_i, y_q, prev_sym_i, prev_sym_q, mid_i, mid_q);
                [u_sym, pi_integ] = pi_controller_float(e, pi_integ, cfg.Kp, cfg.Ki);
                u_p = min(max(Ns * cfg.Kp * e, -cfg.u_clamp), cfg.u_clamp);
                u_i = min(max(cfg.Ki * pi_integ, -cfg.u_clamp), cfg.u_clamp);
                if D_loop > 0
                    pipe_p(1) = u_p;
                    pipe_i(1) = u_i;
                    pipe_v(1) = true;
                else
                    kick_after_mid = u_p;
                    u_i_curr       = u_i;
                end
            else
                e     = 0.0;
                u_sym = 0.0;
            end

            ted_err(sym_cnt)      = e;
            pi_out(sym_cnt)       = u_sym;
            pi_integ_log(sym_cnt) = pi_integ;

            prev_sym_i   = y_i;
            prev_sym_q   = y_q;
            has_prev_sym = true;
            is_symbol_strobe = false;
        end
    else
        eta = eta_sub;
    end
end

% Trim outputs
res.out_sym_i      = out_sym_i(1:sym_cnt);
res.out_sym_q      = out_sym_q(1:sym_cnt);
res.out_sym        = res.out_sym_i + 1j * res.out_sym_q;
res.sym_sample_idx = sym_sample_idx(1:sym_cnt);
res.sym_mu         = sym_mu(1:sym_cnt);
res.ted_err        = ted_err(1:sym_cnt);
res.pi_out         = pi_out(1:sym_cnt);
res.pi_integ       = pi_integ_log(1:sym_cnt);
res.out_valid_clk  = out_valid_clk;
res.eta_trace      = eta_trace;
res.w_trace        = w_trace;
res.strobe_trace   = strobe_trace;
res.mu_all_trace   = mu_all_trace;
res.num_syms_out   = sym_cnt;

end
