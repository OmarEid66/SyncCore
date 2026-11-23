function [y_q15, dbg] = farrow_cubic_fixed(xm1_q11, x0_q11, x1_q11, x2_q11, mu_fx, W_mu)
% FARROW_CUBIC_FIXED Bit-accurate fixed-point Cubic Lagrange Farrow interpolator.
%
%   [y_q15, dbg] = farrow_cubic_fixed(xm1_q11, x0_q11, x1_q11, x2_q11, mu_fx, W_mu)
%
%   Fixed-point formats:
%       Inputs xm1, x0, x1, x2 : Signed 12-bit Q1.11  (int in [-2048, +2047])
%       Input  mu_fx           : Unsigned W_mu-bit UQ0.W_mu (int in [0, 2^W_mu - 1])
%       Internal c0..c3, h2,h1 : Signed 18-bit SQ3.15 (int in [-131072, +131071])
%       Output y_q15           : Signed 16-bit Q1.15  (int in [-32768, +32767])
%
%   Horner evaluation (maps to 3 DSP48E1 slices per I/Q channel):
%       h2    = c2 + round((c3 * mu_fx) >> W_mu)
%       h1    = c1 + round((h2 * mu_fx) >> W_mu)
%       y_raw = c0 + round((h1 * mu_fx) >> W_mu)
%       y_q15 = sat_q15(y_raw)

if nargin < 6, W_mu = 12; end

SQ3_15_MIN = -131072;  % -2^17
SQ3_15_MAX =  131071;  %  2^17 - 1
Q1_15_MIN  = -32768;   % -2^15
Q1_15_MAX  =  32767;   %  2^15 - 1

% Exact alignment from Q1.11 to Q1.15 (left shift by 4 bits)
xm1 = xm1_q11 * 16;
x0  = x0_q11  * 16;
x1  = x1_q11  * 16;
x2  = x2_q11  * 16;

% Branch coefficients in SQ3.15 (zero DSP48E1s; only shifts, adds, and fx_div6)
%   c0 = x0
%   c1 = x1 - (1/2)*x0 - (1/6)*(2*xm1 + x2)
%   c2 = (1/2)*(xm1 + x1) - x0
%   c3 = (1/6)*(x2 - xm1) + (1/2)*(x0 - x1)
c0 = x0;
c1 = x1 - floor(x0 / 2) - fx_div6(2 * xm1 + x2);
c2 = floor((xm1 + x1) / 2) - x0;
c3 = fx_div6(x2 - xm1) + floor((x0 - x1) / 2);

c0 = fx_sat(c0, SQ3_15_MIN, SQ3_15_MAX);
c1 = fx_sat(c1, SQ3_15_MIN, SQ3_15_MAX);
c2 = fx_sat(c2, SQ3_15_MIN, SQ3_15_MAX);
c3 = fx_sat(c3, SQ3_15_MIN, SQ3_15_MAX);

% Horner Stage 1 (DSP48E1 #1): h2 = c2 + round((c3 * mu) >> W_mu)
p3 = c3 .* mu_fx;
h2 = fx_sat(c2 + fx_round_shift(p3, W_mu), SQ3_15_MIN, SQ3_15_MAX);

% Horner Stage 2 (DSP48E1 #2): h1 = c1 + round((h2 * mu) >> W_mu)
p2 = h2 .* mu_fx;
h1 = fx_sat(c1 + fx_round_shift(p2, W_mu), SQ3_15_MIN, SQ3_15_MAX);

% Horner Stage 3 (DSP48E1 #3): y_raw = c0 + round((h1 * mu) >> W_mu)
p1 = h1 .* mu_fx;
y_raw = fx_sat(c0 + fx_round_shift(p1, W_mu), SQ3_15_MIN, SQ3_15_MAX);

% Final saturation to Signed 16-bit Q1.15
y_q15 = fx_sat(y_raw, Q1_15_MIN, Q1_15_MAX);

if nargout > 1
    dbg.c0 = c0; dbg.c1 = c1; dbg.c2 = c2; dbg.c3 = c3;
    dbg.h2 = h2; dbg.h1 = h1; dbg.y_raw = y_raw;
end

end
