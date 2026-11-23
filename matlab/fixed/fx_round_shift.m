function y = fx_round_shift(x, sh)
% FX_ROUND_SHIFT Bit-accurate signed arithmetic right-shift with round-half-up.
%
%   y = fx_round_shift(x, sh)
%
%   Exact hardware equivalent in SystemVerilog:
%       (x + (1 <<< (sh - 1))) >>> sh
%   Using exact integer arithmetic (no floating-point precision loss for |x| < 2^52).

if sh <= 0
    y = x * (2^(-sh));
else
    half = 2^(sh - 1);
    y = floor((x + half) / (2^sh));
end

end
