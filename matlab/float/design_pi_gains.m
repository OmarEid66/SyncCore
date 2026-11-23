function loop = design_pi_gains(BnTs, zeta, Kd, K0, D_sym)
% DESIGN_PI_GAINS Analytical design of discrete-time 2nd-order STR PI loop
%   filter gains (Kp, Ki) from target noise bandwidth Bn*Ts and damping zeta.
%
%   loop = design_pi_gains(BnTs, zeta, Kd, K0, D_sym)
%
%   Inputs:
%       BnTs  - Normalized loop noise bandwidth (Bn * Ts), e.g., 0.015 .. 0.03
%       zeta  - Damping factor, e.g., 0.707 .. 1.0
%       Kd    - Gardner TED gain (TED output per symbol period Ts)
%       K0    - NCO gain (symbol periods per symbol per unit u; K0 = 2.0)
%       D_sym - Extra loop transport delay in symbols (default 0)
%
%   Closed-loop model (PI: I[k]=I[k-1]+e[k], u[k]=Kp*e[k]+Ki*I[k]):
%       F(z) = Kp + Ki / (1 - z^-1)
%       N(z) = K0 * z^-1 / (1 - z^-1)
%       Den(z) = z^2 + (Kd*K0*(Kp+Ki) - 2)*z + (1 - Kd*K0*Kp)

if nargin < 4, K0    = 2.0; end
if nargin < 5, D_sym = 0;   end

% Relationship between Bn*Ts and natural frequency wn*Ts:
%   Bn*Ts = (wn*Ts / 2) * (zeta + 1/(4*zeta))
wnTs = (2.0 * BnTs) / (zeta + 1.0 / (4.0 * zeta));

% Exact pole placement matching continuous-time prototype s^2 + 2*zeta*wn*s + wn^2
if zeta < 1.0
    wdTs = wnTs * sqrt(1.0 - zeta^2);
    p_prod = exp(-2.0 * zeta * wnTs);
    p_sum  = 2.0 * exp(-zeta * wnTs) * cos(wdTs);
else
    r1 = exp((-zeta + sqrt(zeta^2 - 1.0)) * wnTs);
    r2 = exp((-zeta - sqrt(zeta^2 - 1.0)) * wnTs);
    p_prod = r1 * r2;
    p_sum  = r1 + r2;
end

% Map to Kd*K0*Kp and Kd*K0*Ki
A = 1.0 - p_prod;           % Kd * K0 * Kp
B = 1.0 + p_prod - p_sum;   % Kd * K0 * Ki

Kp = A / (Kd * K0);
Ki = B / (Kd * K0);

% Evaluate stability with optional extra integer delay D_sym (symbols)
% Open-loop L(z) = Kd * K0 * (Kp + Ki/(1 - z^-1)) * z^{-(1 + D_sym)} / (1 - z^-1)
w = logspace(-4, log10(pi), 4000);
z = exp(1j * w);
L = Kd * K0 * (Kp + Ki ./ (1.0 - z.^(-1))) .* (z.^(-(1 + D_sym))) ./ (1.0 - z.^(-1));

mag_L = abs(L);
ph_L  = angle(L);   % rad

% Find gain crossover frequency (|L| = 1)
[~, idx_gc] = min(abs(mag_L - 1.0));
phase_margin_deg = 180.0 + (180.0 / pi) * ph_L(idx_gc);
% Normalize phase margin into (-180, 180]
phase_margin_deg = mod(phase_margin_deg + 180.0, 360.0) - 180.0;

% Closed-loop characteristic polynomial for integer D_sym
D_int = round(D_sym);
% (1 - z^-1)^2 + Kd*K0 * ((Kp+Ki) - Kp*z^-1) * z^{-(1+D_int)} = 0
poly_len = max(3, 3 + D_int);
char_poly = zeros(1, poly_len);
char_poly(1) =  1.0;
char_poly(2) = -2.0;
char_poly(3) =  1.0;
char_poly(2 + D_int) = char_poly(2 + D_int) + Kd * K0 * (Kp + Ki);
char_poly(3 + D_int) = char_poly(3 + D_int) - Kd * K0 * Kp;
cl_poles = roots(char_poly);
max_pole_mag = max(abs(cl_poles));

loop.BnTs             = BnTs;
loop.zeta             = zeta;
loop.wnTs             = wnTs;
loop.Kd               = Kd;
loop.K0               = K0;
loop.Kp               = Kp;
loop.Ki               = Ki;
loop.D_sym            = D_sym;
loop.cl_poles         = cl_poles;
loop.max_pole_mag     = max_pole_mag;
loop.is_stable        = (max_pole_mag < 1.0);
loop.phase_margin_deg = phase_margin_deg;

end
