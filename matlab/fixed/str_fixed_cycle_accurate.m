function out = str_fixed_cycle_accurate(rst_n_vec, in_valid_vec, in_i_vec, in_q_vec, kp4_fx, ki_fx)
% STR_FIXED_CYCLE_ACCURATE Cycle-accurate, bit-accurate MATLAB fixed-point
%   golden model matching rtl/str_top.sv (Stages S0..S15) clock-for-clock.
%
%   out = str_fixed_cycle_accurate(rst_n_vec, in_valid_vec, in_i_vec, in_q_vec, kp4_fx, ki_fx)
%
%   Models every pipeline register boundary (S0..S15), input-valid gaps,
%   asynchronous active-low reset during operation, and bit-exact arithmetic
%   from Stage 2 (farrow_cubic_fixed, gardner_ted_fixed, pi_controller_fixed).
%   Expected mismatch against QuestaSim rtl/str_top.sv: 0 LSB on every cycle.

if nargin < 5, kp4_fx = 10344; end
if nargin < 6, ki_fx  = 883;   end

N = length(in_i_vec);
out_sym_i     = zeros(N, 1);
out_sym_q     = zeros(N, 1);
out_sym_valid = false(N, 1);

% Constants
ETA_INIT = 2097152;
W_NOM    = 2097152;
W_MIN    = 1048576;
W_MAX    = 3145728;
ETA_MOD  = 4194304;

% Reset helper state
sr_i = zeros(4, 1); sr_q = zeros(4, 1); fill_cnt = 0; buf_valid = false;
eta_reg = ETA_INIT; is_symbol_strobe = false;
w_freq_reg = W_NOM; kick_reg = 0; w_plus_kick_reg = W_NOM; kick_pending = false;
eta_k_s1 = 0; strobe_s1 = false; is_sym_s1 = false;
mu_s2 = 0; strobe_s2 = false; is_sym_s2 = false;

% Farrow pipe: S1..S8 (we run farrow_cubic_fixed on aligned sr_s0 at S2 and delay 6 clks to S8)
sr_i_s1 = zeros(4, 1); sr_q_s1 = zeros(4, 1);
sr_i_s2 = zeros(4, 1); sr_q_s2 = zeros(4, 1);
f_yi_pipe = zeros(6, 1); f_yq_pipe = zeros(6, 1);
f_val_pipe = false(6, 1); f_sym_pipe = false(6, 1);

% S9 Output & Scheduler
oy_i = 0; oy_q = 0; oy_v = false;
mid_i = 0; mid_q = 0; prev_i = 0; prev_q = 0; has_mid = false; has_prev = false;
ted_di = 0; ted_dq = 0; ted_mi = 0; ted_mq = 0; ted_in_v = false;

% S10-S11 TED
prod_i_s10 = 0; prod_q_s10 = 0; val_s10 = false;
err_s11 = 0; val_s11 = false;

% S12-S14 PI
integ_s12 = 0; p_prod_s12 = 0; val_s12 = false;
i_prod_s13 = 0; u_p_s13 = 0; val_s13 = false;
u_p_s14 = 0; u_i_s14 = 0; val_s14 = false;

for c = 1:N
    if ~rst_n_vec(c)
        sr_i(:) = 0; sr_q(:) = 0; fill_cnt = 0; buf_valid = false;
        eta_reg = ETA_INIT; is_symbol_strobe = false;
        w_freq_reg = W_NOM; kick_reg = 0; w_plus_kick_reg = W_NOM; kick_pending = false;
        eta_k_s1 = 0; strobe_s1 = false; is_sym_s1 = false;
        mu_s2 = 0; strobe_s2 = false; is_sym_s2 = false;
        sr_i_s1(:) = 0; sr_q_s1(:) = 0; sr_i_s2(:) = 0; sr_q_s2(:) = 0;
        f_yi_pipe(:) = 0; f_yq_pipe(:) = 0; f_val_pipe(:) = false; f_sym_pipe(:) = false;
        oy_i = 0; oy_q = 0; oy_v = false;
        mid_i = 0; mid_q = 0; prev_i = 0; prev_q = 0; has_mid = false; has_prev = false;
        ted_di = 0; ted_dq = 0; ted_mi = 0; ted_mq = 0; ted_in_v = false;
        prod_i_s10 = 0; prod_q_s10 = 0; val_s10 = false;
        err_s11 = 0; val_s11 = false;
        integ_s12 = 0; p_prod_s12 = 0; val_s12 = false;
        i_prod_s13 = 0; u_p_s13 = 0; val_s13 = false;
        u_p_s14 = 0; u_i_s14 = 0; val_s14 = false;
    else
        % --- Capture current registered outputs before posedge updates ---
        % S14 -> S15
        n_w_freq_reg = w_freq_reg; n_kick_reg = kick_reg;
        n_w_plus_kick = w_plus_kick_reg; n_kick_pending = kick_pending;

        use_kick = buf_valid && is_symbol_strobe && kick_pending;
        if val_s14
            wc = fx_sat(W_NOM + u_i_s14, W_MIN, W_MAX);
            n_w_freq_reg   = wc;
            n_kick_reg     = u_p_s14;
            n_w_plus_kick  = wc + u_p_s14;
            n_kick_pending = true;
        elseif use_kick
            n_kick_pending = false;
        end

        % S13 -> S14
        n_u_p_s14 = u_p_s14; n_u_i_s14 = u_i_s14; n_val_s14 = val_s13;
        if val_s13
            n_u_p_s14 = u_p_s13;
            n_u_i_s14 = fx_sat(fx_round_shift(i_prod_s13, 14), -1048576, 1048576);
        end

        % S12 -> S13
        n_i_prod_s13 = i_prod_s13; n_u_p_s13 = u_p_s13; n_val_s13 = val_s12;
        if val_s12
            n_i_prod_s13 = integ_s12 * ki_fx;
            n_u_p_s13    = fx_sat(fx_round_shift(p_prod_s12, 9), -1048576, 1048576);
        end

        % S11 -> S12
        n_integ_s12 = integ_s12; n_p_prod_s12 = p_prod_s12; n_val_s12 = val_s11;
        if val_s11
            n_integ_s12  = fx_sat(integ_s12 + err_s11, -8388608, 8388607);
            n_p_prod_s12 = err_s11 * kp4_fx;
        end

        % S10 -> S11
        n_err_s11 = fx_sat(fx_round_shift(prod_i_s10 + prod_q_s10, 15), -131072, 131071);
        n_val_s11 = val_s10;

        % S9 -> S10
        n_prod_i_s10 = ted_di * ted_mi;
        n_prod_q_s10 = ted_dq * ted_mq;
        n_val_s10    = ted_in_v;

        % S8 -> S9 (Output reg + Scheduler)
        fy_i = f_yi_pipe(6); fy_q = f_yq_pipe(6);
        f_v  = f_val_pipe(6); f_s = f_sym_pipe(6);
        n_oy_i = oy_i; n_oy_q = oy_q; n_oy_v = false;
        if f_v && f_s
            n_oy_i = fy_i; n_oy_q = fy_q; n_oy_v = true;
        end

        n_mid_i = mid_i; n_mid_q = mid_q; n_has_mid = has_mid;
        n_prev_i = prev_i; n_prev_q = prev_q; n_has_prev = has_prev;
        n_ted_di = ted_di; n_ted_dq = ted_dq; n_ted_mi = ted_mi; n_ted_mq = ted_mq;
        n_ted_in_v = false;
        if f_v
            if ~f_s
                n_mid_i = fy_i; n_mid_q = fy_q; n_has_mid = true;
            else
                n_prev_i = fy_i; n_prev_q = fy_q; n_has_prev = true;
                if has_prev && has_mid
                    n_ted_di   = fy_i - prev_i;
                    n_ted_dq   = fy_q - prev_q;
                    n_ted_mi   = mid_i;
                    n_ted_mq   = mid_q;
                    n_ted_in_v = true;
                end
            end
        end

        % S2 -> S3..S8 (Farrow evaluation bit-identical to S3..S8 Horner stages)
        yi_s2 = farrow_cubic_fixed(sr_i_s2(4), sr_i_s2(3), sr_i_s2(2), sr_i_s2(1), mu_s2, 12);
        yq_s2 = farrow_cubic_fixed(sr_q_s2(4), sr_q_s2(3), sr_q_s2(2), sr_q_s2(1), mu_s2, 12);
        n_f_yi_pipe  = [yi_s2;     f_yi_pipe(1:5)];
        n_f_yq_pipe  = [yq_s2;     f_yq_pipe(1:5)];
        n_f_val_pipe = [strobe_s2; f_val_pipe(1:5)];
        n_f_sym_pipe = [is_sym_s2; f_sym_pipe(1:5)];

        % S1 -> S2
        n_sr_i_s2   = sr_i_s1;
        n_sr_q_s2   = sr_q_s1;
        n_mu_s2     = fx_sat(fx_round_shift(2 * eta_k_s1, 10), 0, 4095);
        n_strobe_s2 = strobe_s1;
        n_is_sym_s2 = is_sym_s1;

        % S0 -> S1 (NCO + Farrow S1 sample capture)
        n_sr_i_s1 = sr_i;
        n_sr_q_s1 = sr_q;
        n_eta_reg = eta_reg; n_is_symbol_strobe = is_symbol_strobe;
        n_eta_k_s1 = eta_k_s1; n_strobe_s1 = false; n_is_sym_s1 = false;
        if buf_valid
            if use_kick
                step_eff = w_plus_kick_reg;
                kick_eff = kick_reg;
            else
                step_eff = w_freq_reg;
                kick_eff = 0;
            end
            eta_sub = eta_reg - step_eff;
            eta_k   = eta_reg - kick_eff;
            if eta_sub < 0
                n_eta_reg = fx_sat(eta_sub + ETA_MOD, 0, ETA_MOD - 1);
                n_strobe_s1 = true;
                n_is_symbol_strobe = ~is_symbol_strobe;
            else
                n_eta_reg = fx_sat(eta_sub, 0, ETA_MOD - 1);
                n_strobe_s1 = false;
            end
            n_eta_k_s1 = eta_k;
            n_is_sym_s1 = is_symbol_strobe;
        end

        % Input -> S0
        n_sr_i = sr_i; n_sr_q = sr_q; n_fill_cnt = fill_cnt; n_buf_valid = false;
        if in_valid_vec(c)
            n_sr_i = [fx_sat(in_i_vec(c), -2048, 2047); sr_i(1:3)];
            n_sr_q = [fx_sat(in_q_vec(c), -2048, 2047); sr_q(1:3)];
            if fill_cnt < 3
                n_fill_cnt  = fill_cnt + 1;
                n_buf_valid = false;
            else
                n_buf_valid = true;
            end
        end

        % Commit all registers
        sr_i = n_sr_i; sr_q = n_sr_q; fill_cnt = n_fill_cnt; buf_valid = n_buf_valid;
        eta_reg = n_eta_reg; is_symbol_strobe = n_is_symbol_strobe;
        w_freq_reg = n_w_freq_reg; kick_reg = n_kick_reg;
        w_plus_kick_reg = n_w_plus_kick; kick_pending = n_kick_pending;
        eta_k_s1 = n_eta_k_s1; strobe_s1 = n_strobe_s1; is_sym_s1 = n_is_sym_s1;
        mu_s2 = n_mu_s2; strobe_s2 = n_strobe_s2; is_sym_s2 = n_is_sym_s2;
        sr_i_s1 = n_sr_i_s1; sr_q_s1 = n_sr_q_s1; sr_i_s2 = n_sr_i_s2; sr_q_s2 = n_sr_q_s2;
        f_yi_pipe = n_f_yi_pipe; f_yq_pipe = n_f_yq_pipe;
        f_val_pipe = n_f_val_pipe; f_sym_pipe = n_f_sym_pipe;
        oy_i = n_oy_i; oy_q = n_oy_q; oy_v = n_oy_v;
        mid_i = n_mid_i; mid_q = n_mid_q; has_mid = n_has_mid;
        prev_i = n_prev_i; prev_q = n_prev_q; has_prev = n_has_prev;
        ted_di = n_ted_di; ted_dq = n_ted_dq; ted_mi = n_ted_mi; ted_mq = n_ted_mq;
        ted_in_v = n_ted_in_v;
        prod_i_s10 = n_prod_i_s10; prod_q_s10 = n_prod_q_s10; val_s10 = n_val_s10;
        err_s11 = n_err_s11; val_s11 = n_val_s11;
        integ_s12 = n_integ_s12; p_prod_s12 = n_p_prod_s12; val_s12 = n_val_s12;
        i_prod_s13 = n_i_prod_s13; u_p_s13 = n_u_p_s13; val_s13 = n_val_s13;
        u_p_s14 = n_u_p_s14; u_i_s14 = n_u_i_s14; val_s14 = n_val_s14;
    end

    out_sym_i(c)     = oy_i;
    out_sym_q(c)     = oy_q;
    out_sym_valid(c) = oy_v;
end

out.out_sym_i     = out_sym_i;
out.out_sym_q     = out_sym_q;
out.out_sym_valid = out_sym_valid;
end
