function [y, coeffs] = farrow_cubic_float(xm1, x0, x1, x2, mu)
% FARROW_CUBIC_FLOAT Cubic Lagrange Farrow interpolator using Horner's form.
%
%   [y, coeffs] = farrow_cubic_float(xm1, x0, x1, x2, mu)
%
%   Interpolates between x0 (at t=0) and x1 (at t=1) at fractional delay
%   mu in [0, 1) using four neighboring samples:
%       xm1 = x[m-1], x0 = x[m], x1 = x[m+1], x2 = x[m+2]
%
%   Polynomial:
%       y(mu) = c0 + mu * (c1 + mu * (c2 + mu * c3))
%
%   Cubic Lagrange coefficients:
%       c0 = x0
%       c1 = (-1/3)*xm1 - (1/2)*x0 + x1 - (1/6)*x2
%       c2 = (1/2)*xm1  - x0       + (1/2)*x1
%       c3 = (-1/6)*xm1 + (1/2)*x0 - (1/2)*x1 + (1/6)*x2
%
%   Works for real (I or Q independently) or vectorized double inputs.

c0 = x0;
c1 = (-1.0/3.0) * xm1 - 0.5 * x0 + x1 - (1.0/6.0) * x2;
c2 =  0.5 * xm1       - x0       + 0.5 * x1;
c3 = (-1.0/6.0) * xm1 + 0.5 * x0 - 0.5 * x1 + (1.0/6.0) * x2;

% Horner's rule evaluation: c0 + mu*(c1 + mu*(c2 + mu*c3))
h2 = c2 + mu .* c3;
h1 = c1 + mu .* h2;
y  = c0 + mu .* h1;

if nargout > 1
    coeffs.c0 = c0;
    coeffs.c1 = c1;
    coeffs.c2 = c2;
    coeffs.c3 = c3;
end

end
