function [u, integ_next] = pi_controller_float(e, integ_prev, Kp, Ki)
% PI_CONTROLLER_FLOAT Proportional-Integral timing loop filter (double).
%
%   [u, integ_next] = pi_controller_float(e, integ_prev, Kp, Ki)
%
%   Implements:
%       I[k] = I[k-1] + e[k]
%       u[k] = Kp * e[k] + Ki * I[k]

integ_next = integ_prev + e;
u = Kp * e + Ki * integ_next;

end
