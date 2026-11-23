function [e_sq3_15, dbg] = gardner_ted_fixed(i_curr, q_curr, i_prev, q_prev, i_mid, q_mid)
% GARDNER_TED_FIXED Bit-accurate fixed-point Gardner Timing Error Detector.
%
%   [e_sq3_15, dbg] = gardner_ted_fixed(i_curr, q_curr, i_prev, q_prev, i_mid, q_mid)
%
%   Fixed-point formats:
%       Inputs i_*, q_* : Signed 16-bit Q1.15   (int in [-32768, +32767])
%       Differences di,dq: Signed 17-bit SQ2.15 (int in [-65536, +65535])
%       Products prod_* : Signed 33-bit SQ3.30  (2 DSP48E1 slices)
%       Sum sum_iq      : Signed 34-bit SQ4.30
%       Output e_sq3_15 : Signed 18-bit SQ3.15  (int in [-131072, +131071])

SQ3_15_MIN = -131072;  % -2^17
SQ3_15_MAX =  131071;  %  2^17 - 1

% 17-bit signed differences (SQ2.15)
di = i_curr - i_prev;
dq = q_curr - q_prev;

% 17s x 16s = 33-bit signed products (SQ3.30) -> 2 DSP48E1 slices
prod_i = di .* i_mid;
prod_q = dq .* q_mid;

% 34-bit signed sum (SQ4.30)
sum_iq = prod_i + prod_q;

% Round right-shift by 15 -> SQ4.15, then saturate to 18-bit signed SQ3.15
e_round  = fx_round_shift(sum_iq, 15);
e_sq3_15 = fx_sat(e_round, SQ3_15_MIN, SQ3_15_MAX);

if nargout > 1
    dbg.di = di; dbg.dq = dq;
    dbg.prod_i = prod_i; dbg.prod_q = prod_q;
    dbg.sum_iq = sum_iq;
end

end
