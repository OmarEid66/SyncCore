function y = fx_div6(x)
% FX_DIV6 Bit-accurate multiplier-free constant multiplication by 1/6.
%
%   y = fx_div6(x)
%
%   Hardware identity:
%       21845 = 5 * 17 * 257 = (1 + 2^2) * (1 + 2^4) * (1 + 2^8)
%       21845 / 2^17 = 0.1666641235...  (exact 1/6 = 0.1666666667...)
%       Error = -2.54e-6 (< 0.09 LSB at Q1.15), ZERO DSP48E1 slices needed!
%
%   Implemented identically in RTL using 3 shift-add stages + round shift >> 17.

t1 = x  + x  * 4;     % x * 5
t2 = t1 + t1 * 16;    % x * 85
t3 = t2 + t2 * 256;   % x * 21845
y  = fx_round_shift(t3, 17);

end
