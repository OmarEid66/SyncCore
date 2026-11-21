function [cfo_est_q23, cfo_valid, nco_ftw, latency_symbols, sim_trace] = cfo_coarse_golden_model(rx_i, rx_q, rx_valid, burst_start, preamble_enable, params)
% CFO_COARSE_GOLDEN_MODEL Bit-accurate fixed-point Golden Reference Model
% for Preamble-Assisted Coarse CFO Estimation Unit
%
% Conforms to System Specifications:
% - Input: 12-bit signed Q1.11 per channel
% - Output: 24-bit signed Q1.23 normalized CFO (f_norm = 2*delta_f / Fs)
% - Delay: L = 16 samples (power-of-2)
% - Accumulation: M samples (configurable, default 192 = 48 symbols)
% - CORDIC: 16-stage pipelined vectoring (atan2)
% - CFO calculation: f_norm = theta / (16*pi) -> arithmetic right shift by 4 bits
% - NCO FTW: 32-bit signed (coarse_cfo_est <<< 8)

    if nargin < 6, params = struct(); end
    if ~isfield(params, 'L'),             params.L = 16; end
    if ~isfield(params, 'M'),             params.M = 192; end
    if ~isfield(params, 'auto_mode_en'),  params.auto_mode_en = 1; end
    if ~isfield(params, 'gamma_thresh'),  params.gamma_thresh = 0.65; end
    if ~isfield(params, 'energy_min'),    params.energy_min = 1e6; end
    if ~isfield(params, 'N_consec'),      params.N_consec = 2; end

    L = params.L;
    M = params.M;
    N_samples = length(rx_i);

    % Delay Line Storage (L samples)
    delay_i = zeros(L, 1, 'int64');
    delay_q = zeros(L, 1, 'int64');

    % Sliding Window for Autonomous Detection (length L)
    win_curr_i = zeros(L, 1, 'int64');
    win_curr_q = zeros(L, 1, 'int64');
    win_del_i  = zeros(L, 1, 'int64');
    win_del_q  = zeros(L, 1, 'int64');

    % State Machine States
    STATE_IDLE       = 0;
    STATE_SEARCHING  = 1;
    STATE_ACCUM      = 2;
    STATE_CORDIC     = 3;
    STATE_DONE       = 4;

    state = STATE_IDLE;

    % Accumulator registers
    R_i = int64(0);
    R_q = int64(0);
    accum_count = 0;

    % Consecutive detection counter
    consec_count = 0;
    max_consec = 0;

    % Output variables
    cfo_est_q23 = zeros(N_samples, 1, 'int64');
    cfo_valid   = zeros(N_samples, 1, 'logical');
    nco_ftw     = zeros(N_samples, 1, 'int64');
    metric_out  = zeros(N_samples, 1);

    start_sample_idx = 0;
    done_sample_idx = 0;

    % Pipeline delay for CORDIC (16 cycles) + Output Stage (2 cycles) = 18 cycles
    CORDIC_PIPE_LEN = 18;
    cordic_pipe_cnt = 0;
    latched_R_i = int64(0);
    latched_R_q = int64(0);

    for n = 1:N_samples
        in_vld = rx_valid(n);
        b_start = burst_start(n);
        p_en = preamble_enable(n);

        % Handle reset / burst start
        if b_start
            state = STATE_SEARCHING;
            accum_count = 0;
            consec_count = 0;
            R_i = int64(0);
            R_q = int64(0);
            cordic_pipe_cnt = 0;
            delay_i = zeros(L, 1, 'int64');
            delay_q = zeros(L, 1, 'int64');
            start_sample_idx = n;
        end

        if in_vld
            curr_i = int64(rx_i(n));
            curr_q = int64(rx_q(n));

            % Read delayed sample from tap L-1
            del_i = delay_i(end);
            del_q = delay_q(end);

            % Shift delay line
            delay_i = [curr_i; delay_i(1:end-1)];
            delay_q = [curr_q; delay_q(1:end-1)];

            % Update sliding detection windows
            win_curr_i = [curr_i; win_curr_i(1:end-1)];
            win_curr_q = [curr_q; win_curr_q(1:end-1)];
            win_del_i  = [del_i; win_del_i(1:end-1)];
            win_del_q  = [del_q; win_del_q(1:end-1)];

            % Compute detection metric C[n], E1[n], E2[n]
            C_re = sum(win_curr_i .* win_del_i + win_curr_q .* win_del_q);
            C_im = sum(win_curr_q .* win_del_i - win_curr_i .* win_del_q);
            C_mag2 = double(C_re)^2 + double(C_im)^2;

            E1 = double(sum(win_curr_i.^2 + win_curr_q.^2));
            E2 = double(sum(win_del_i.^2 + win_del_q.^2));

            det_metric = C_mag2 / (E1 * E2 + 1e-6);
            metric_out(n) = det_metric;

            % Detection decision
            is_detected = 0;
            if params.auto_mode_en
                if (det_metric >= params.gamma_thresh) && (E1 >= params.energy_min)
                    consec_count = consec_count + 1;
                    if consec_count > max_consec, max_consec = consec_count; end
                else
                    consec_count = 0;
                end
                if consec_count >= params.N_consec
                    is_detected = 1;
                end
            else
                if p_en
                    is_detected = 1;
                end
            end

            % State Machine Processing
            switch state
                case STATE_IDLE
                    if p_en || (params.auto_mode_en && is_detected)
                        state = STATE_ACCUM;
                        accum_count = 0;
                        R_i = int64(0);
                        R_q = int64(0);
                        if start_sample_idx == 0
                            start_sample_idx = n;
                        end
                    end

                case STATE_SEARCHING
                    if p_en || is_detected
                        state = STATE_ACCUM;
                        accum_count = 0;
                        R_i = int64(0);
                        R_q = int64(0);
                        start_sample_idx = n;
                    end

                case STATE_ACCUM
                    % Complex conjugate product: P[n] = r[n] * conj(r[n-L])
                    P_i = curr_i * del_i + curr_q * del_q;
                    P_q = curr_q * del_i - curr_i * del_q;

                    R_i = R_i + P_i;
                    R_q = R_q + P_q;
                    accum_count = accum_count + 1;

                    if accum_count >= M
                        state = STATE_CORDIC;
                        cordic_pipe_cnt = CORDIC_PIPE_LEN;
                        latched_R_i = R_i;
                        latched_R_q = R_q;
                    end

                case STATE_CORDIC
                    % Wait for pipelined CORDIC and output calculation
                    % Samples still arrive, but accumulator has completed
                    % (handled below)

                case STATE_DONE
                    % Idle waiting for next burst
            end
        end

        % Pipelined CORDIC execution (clock driven)
        if state == STATE_CORDIC
            if cordic_pipe_cnt > 0
                cordic_pipe_cnt = cordic_pipe_cnt - 1;
            end

            if cordic_pipe_cnt == 0
                % Calculate Phase via CORDIC
                % Output theta_fixed is in Q1.23 format where 2^23 = pi
                [theta_fixed, ~, ~] = cordic_atan2_fixed(latched_R_q, latched_R_i, 16, 24);

                % CFO Calculator: f_norm = theta / (16*pi)
                % In Q1.23: theta_fixed represents theta/pi
                % Therefore, dividing by 16 is an exact 4-bit arithmetic right shift!
                est_val = bitshift(int64(theta_fixed), -4);

                % Saturate to signed 24-bit range [-2^23, 2^23 - 1]
                if est_val > 8388607, est_val = int64(8388607); end
                if est_val < -8388608, est_val = int64(-8388608); end

                % Downstream 32-bit NCO Frequency Tuning Word:
                % assign nco_ftw = $signed(coarse_cfo_est) <<< 8;
                ftw_val = bitshift(est_val, 8);

                cfo_est_q23(n) = est_val;
                cfo_valid(n)   = true;
                nco_ftw(n)     = ftw_val;

                done_sample_idx = n;
                state = STATE_DONE;
            end
        end
    end

    if done_sample_idx > 0 && start_sample_idx > 0
        total_latency_cycles = done_sample_idx - start_sample_idx;
        latency_symbols = total_latency_cycles / 4.0; % 4 samples per symbol
    else
        total_latency_cycles = 0;
        latency_symbols = 0;
    end

    sim_trace.metric = metric_out;
    sim_trace.latency_cycles = total_latency_cycles;
    sim_trace.latency_symbols = latency_symbols;
    sim_trace.final_state = state;
    sim_trace.final_accum = accum_count;
    sim_trace.consec_count = consec_count;
    sim_trace.max_consec = max_consec;
end
