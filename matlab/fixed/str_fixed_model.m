function res = str_fixed_model(rx_i_q11, rx_q_q11, in_valid, cfg)
% STR_FIXED_MODEL Bit-accurate fixed-point streaming Symbol Timing Recovery
%   (STR) golden reference model for SystemVerilog RTL verification.
%
%   res = str_fixed_model(rx_i_q11, rx_q_q11, in_valid, cfg)
%
%   Interface formats (exact RTL port contract):
%       rx_i_q11, rx_q_q11 : Signed 12-bit Q1.11 integers in [-2048, +2047]
%       in_valid           : Logical/boolean vector (1 sample/clock when true)
%       out_sym_i_q15      : Signed 16-bit Q1.15 integers in [-32768, +32767]
%       out_sym_q_q15      : Signed 16-bit Q1.15 integers in [-32768, +32767]
%
%   cfg fields:
%       kp4_fx          : Signed 18-bit round(4 * Kp * 2^16)
%       ki_fx           : Signed 18-bit round(Ki * 2^21)
%       W_mu            : Fractional delay precision in bits (8..12, default 12)
%       init_eta_q22    : Initial NCO phase in UQ0.22 (default 2^21 = 0.5)
%       loop_delay_clks : Exact feedback pipeline latency in valid clocks
%                         (0 for ideal arithmetic comparison, or RTL latency for
%                          cycle-accurate bit-exact RTL golden reference)

if ~isfield(cfg, 'W_mu'),            cfg.W_mu = 12; end
if ~isfield(cfg, 'init_eta_q22'),    cfg.init_eta_q22 = 2097152; end % 0.5 * 2^22
if ~isfield(cfg, 'loop_delay_clks'), cfg.loop_delay_clks = 0; end

W_ETA     = 22;
ETA_MOD   = 4194304;  % 2^22
W_NOM     = 2097152;  % 0.50 * 2^22
W_MIN     = 1048576;  % 0.25 * 2^22
W_MAX     = 3145728;  % 0.75 * 2^22
W_mu      = cfg.W_mu;
MU_MAX    = 2^W_mu - 1;
MU_SHIFT  = W_ETA - W_mu;

N = length(rx_i_q11);
max_syms = ceil(N / 2) + 16;

out_sym_i_q15  = zeros(max_syms, 1);
out_sym_q_q15  = zeros(max_syms, 1);
sym_sample_idx = zeros(max_syms, 1);
sym_mu_fx      = zeros(max_syms, 1);
ted_err_q15    = zeros(max_syms, 1);
u_p_log_q22    = zeros(max_syms, 1);
u_i_log_q22    = zeros(max_syms, 1);
pi_integ_q15   = zeros(max_syms, 1);
out_valid_clk  = false(N, 1);
eta_trace_q22  = zeros(N, 1);

% 4-tap input shift registers (Signed 12-bit Q1.11)
sr_i = zeros(4, 1);
sr_q = zeros(4, 1);
valid_count = 0;

eta              = cfg.init_eta_q22;
is_symbol_strobe = false;
has_mid          = false;
has_prev_sym     = false;

mid_i_q15      = 0; mid_q_q15      = 0;
prev_sym_i_q15 = 0; prev_sym_q_q15 = 0;

pi_integ     = 0;
u_i_curr     = 0;
kick_reg     = 0;
kick_pending = false;

D_loop = cfg.loop_delay_clks;
if D_loop > 0
    pipe_p = zeros(D_loop, 1);
    pipe_i = zeros(D_loop, 1);
    pipe_v = false(D_loop, 1);
end

sym_cnt = 0;

for idx = 1:N
    if ~in_valid(idx)
        eta_trace_q22(idx) = eta;
        continue;
    end

    % 1. Shift 12-bit Q1.11 input sample into 4-tap Farrow buffer
    in_i = fx_sat(round(rx_i_q11(idx)), -2048, 2047);
    in_q = fx_sat(round(rx_q_q11(idx)), -2048, 2047);
    sr_i = [in_i; sr_i(1:3)];
    sr_q = [in_q; sr_q(1:3)];
    valid_count = valid_count + 1;

    if valid_count < 4
        eta_trace_q22(idx) = eta;
        continue;
    end

    % Advance optional feedback latency FIFO (models exact RTL loop latency)
    if D_loop > 0
        if pipe_v(end)
            kick_reg     = pipe_p(end);
            kick_pending = true;
            u_i_curr     = pipe_i(end);
        end
        pipe_p = [0;        pipe_p(1:end-1)];
        pipe_i = [u_i_curr; pipe_i(1:end-1)];
        pipe_v = [false;    pipe_v(1:end-1)];
    end

    % 2. Timing Phase Accumulator (Decrementing Mod-1 UQ0.22 NCO)
    w_freq = fx_sat(W_NOM + u_i_curr, W_MIN, W_MAX);
    if is_symbol_strobe && kick_pending
        kick = kick_reg;
        kick_pending = false;
    else
        kick = 0;
    end

    eta_k   = eta - kick;
    eta_sub = eta_k - w_freq;
    eta_trace_q22(idx) = eta;

    if eta_sub < 0
        % Multiplier-free fractional delay mu = clamp(round(2 * eta_k), 0, 2^W_mu - 1)
        eta_2x = 2 * eta_k;
        mu_raw = fx_round_shift(eta_2x, MU_SHIFT);
        mu_fx  = fx_sat(mu_raw, 0, MU_MAX);

        % Wrap mod 2^22 (proven invariant: 0 <= eta_next < 2^22)
        eta = fx_sat(eta_sub + ETA_MOD, 0, ETA_MOD - 1);

        % 3. Bit-Accurate Cubic Farrow Interpolator (I & Q)
        y_i_q15 = farrow_cubic_fixed(sr_i(4), sr_i(3), sr_i(2), sr_i(1), mu_fx, W_mu);
        y_q_q15 = farrow_cubic_fixed(sr_q(4), sr_q(3), sr_q(2), sr_q(1), mu_fx, W_mu);

        % 4. Timing Sample Extraction (Midpoint vs Symbol)
        if ~is_symbol_strobe
            mid_i_q15 = y_i_q15;
            mid_q_q15 = y_q_q15;
            has_mid   = true;
            is_symbol_strobe = true;
        else
            sym_cnt = sym_cnt + 1;
            out_sym_i_q15(sym_cnt)  = y_i_q15;
            out_sym_q_q15(sym_cnt)  = y_q_q15;
            sym_sample_idx(sym_cnt) = idx - 1;
            sym_mu_fx(sym_cnt)      = mu_fx;
            out_valid_clk(idx)      = true;

            % 5. Bit-Accurate Gardner TED + PI Loop Filter
            if has_prev_sym && has_mid
                e_fx = gardner_ted_fixed(y_i_q15, y_q_q15, prev_sym_i_q15, prev_sym_q_q15, mid_i_q15, mid_q_q15);
                [u_p_fx, u_i_fx, pi_integ] = pi_controller_fixed(e_fx, pi_integ, cfg.kp4_fx, cfg.ki_fx);
                if D_loop > 0
                    pipe_p(1) = u_p_fx;
                    pipe_i(1) = u_i_fx;
                    pipe_v(1) = true;
                else
                    kick_reg     = u_p_fx;
                    kick_pending = true;
                    u_i_curr     = u_i_fx;
                end
            else
                e_fx   = 0;
                u_p_fx = 0;
                u_i_fx = u_i_curr;
            end

            ted_err_q15(sym_cnt)  = e_fx;
            u_p_log_q22(sym_cnt)  = u_p_fx;
            u_i_log_q22(sym_cnt)  = u_i_fx;
            pi_integ_q15(sym_cnt) = pi_integ;

            prev_sym_i_q15 = y_i_q15;
            prev_sym_q_q15 = y_q_q15;
            has_prev_sym   = true;
            is_symbol_strobe = false;
        end
    else
        eta = fx_sat(eta_sub, 0, ETA_MOD - 1);
    end
end

% Trim outputs
res.out_sym_i_q15  = out_sym_i_q15(1:sym_cnt);
res.out_sym_q_q15  = out_sym_q_q15(1:sym_cnt);
res.out_sym_float  = (res.out_sym_i_q15 + 1j * res.out_sym_q_q15) / 32768.0;
res.sym_sample_idx = sym_sample_idx(1:sym_cnt);
res.sym_mu_fx      = sym_mu_fx(1:sym_cnt);
res.sym_mu_float   = res.sym_mu_fx / (2^W_mu);
res.ted_err_q15    = ted_err_q15(1:sym_cnt);
res.ted_err_float  = res.ted_err_q15 / 32768.0;
res.u_p_q22        = u_p_log_q22(1:sym_cnt);
res.u_i_q22        = u_i_log_q22(1:sym_cnt);
res.pi_out_float   = (res.u_p_q22 / 4.0 + res.u_i_q22) / 4194304.0;
res.pi_integ_q15   = pi_integ_q15(1:sym_cnt);
res.out_valid_clk  = out_valid_clk;
res.eta_trace_q22  = eta_trace_q22;
res.num_syms_out   = sym_cnt;

end
