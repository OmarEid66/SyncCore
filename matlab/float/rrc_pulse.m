function p = rrc_pulse(u, alpha)
% RRC_PULSE Exact analytical continuous-time Root-Raised Cosine (RRC) pulse.
%
%   p = rrc_pulse(u, alpha)
%
%   Inputs:
%       u     - Normalized time vector/matrix (t / T_sym)
%       alpha - Roll-off factor in (0, 1] (default: 0.35)
%
%   Output:
%       p     - Continuous-time RRC pulse amplitude evaluated at u
%               (unnormalized continuous amplitude; normalize discrete filter
%                by sqrt(sum(h.^2)) for unit discrete energy).
%
%   Handles removable singularities at u = 0 and u = +/- 1/(4*alpha)
%   without requiring any MATLAB toolbox.

if nargin < 2
    alpha = 0.35;
end

p = zeros(size(u));
eps_tol = 1e-10;

% Case 1: u == 0
idx0 = abs(u) < eps_tol;
if any(idx0(:))
    p(idx0) = 1.0 - alpha + (4.0 * alpha / pi);
end

% Case 2: |u| == 1 / (4*alpha) (only when alpha > 0)
if alpha > 0
    u_sing = 1.0 / (4.0 * alpha);
    idx_s = (~idx0) & (abs(abs(u) - u_sing) < eps_tol);
    if any(idx_s(:))
        p(idx_s) = (alpha / sqrt(2.0)) * ( ...
            (1.0 + 2.0 / pi) * sin(pi / (4.0 * alpha)) + ...
            (1.0 - 2.0 / pi) * cos(pi / (4.0 * alpha)) );
    end
else
    idx_s = false(size(u));
end

% Case 3: General points
idx_g = (~idx0) & (~idx_s);
if any(idx_g(:))
    ug = u(idx_g);
    num = sin(pi * ug * (1.0 - alpha)) + 4.0 * alpha * ug .* cos(pi * ug * (1.0 + alpha));
    den = pi * ug .* (1.0 - (4.0 * alpha * ug).^2);
    p(idx_g) = num ./ den;
end

end
