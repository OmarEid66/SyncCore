function [u_p_q22, u_i_q22, integ_next] = pi_controller_fixed(e_sq3_15, integ_prev, kp4_fx, ki_fx)
% PI_CONTROLLER_FIXED Bit-accurate fixed-point PI Timing Loop Filter.
%
%   [u_p_q22, u_i_q22, integ_next] = pi_controller_fixed(e_sq3_15, integ_prev, kp4_fx, ki_fx)
%
%   Fixed-point formats:
%       e_sq3_15   : Signed 18-bit SQ3.15     (Gardner TED error)
%       integ_prev : Signed 24-bit SQ9.15     (PI integrator [-256, +256))
%       kp4_fx     : Signed 18-bit SQ1.16     (round(4 * Kp * 2^16))
%       ki_fx      : Signed 18-bit scaled 2^21 (round(Ki * 2^21))
%       u_p_q22    : Signed 24-bit SQ2.22     (one-shot phase kick, clamped [-0.25,+0.25])
%       u_i_q22    : Signed 24-bit SQ2.22     (frequency offset word, clamped [-0.25,+0.25])
%       integ_next : Signed 24-bit SQ9.15     (saturated integrator state)
%
%   Hardware mapping: 2 DSP48E1 slices (18s x 18s for Kp, 24s x 18s for Ki).

INTEG_MIN = -8388608;  % -2^23
INTEG_MAX =  8388607;  %  2^23 - 1
U_CLAMP   =  1048576;  %  0.25 * 2^22

% Integrator accumulation with saturation (SQ9.15)
integ_next = fx_sat(integ_prev + e_sq3_15, INTEG_MIN, INTEG_MAX);

% Proportional branch (DSP48E1 #1): SQ3.15 * SQ1.16 = SQ4.31 -> >> 9 -> SQ2.22
p_prod  = e_sq3_15 * kp4_fx;
u_p_raw = fx_round_shift(p_prod, 9);
u_p_q22 = fx_sat(u_p_raw, -U_CLAMP, U_CLAMP);

% Integral branch (DSP48E1 #2): SQ9.15 * (Ki*2^21) -> 36 frac bits -> >> 14 -> SQ2.22
i_prod  = integ_next * ki_fx;
u_i_raw = fx_round_shift(i_prod, 14);
u_i_q22 = fx_sat(u_i_raw, -U_CLAMP, U_CLAMP);

end
