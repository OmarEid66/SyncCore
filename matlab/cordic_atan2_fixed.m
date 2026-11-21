function [theta_fixed, x_out, y_out] = cordic_atan2_fixed(y_in, x_in, N_stages, TOTAL_BITS)
% CORDIC_ATAN2_FIXED Bit-accurate 16-stage CORDIC vectoring in fixed-point
%
% Inputs:
%   y_in       - Signed imaginary component (numerator)
%   x_in       - Signed real component (denominator)
%   N_stages   - Number of pipelined iterations (default: 16)
%   TOTAL_BITS - Phase accumulator bit-width (default: 24, Q1.23 format)
%
% Outputs:
%   theta_fixed - Signed fixed-point phase normalized to [-pi, +pi) as [-2^(TOTAL_BITS-1), 2^(TOTAL_BITS-1)-1]
%                 Where 2^(TOTAL_BITS-1) corresponds to pi.

    if nargin < 3, N_stages = 16; end
    if nargin < 4, TOTAL_BITS = 24; end

    SCALE = 2^(TOTAL_BITS - 1); % 2^23 for 24-bit Q1.23

    % Elementary angles: alpha_i = atan(2^(-i)) / pi * SCALE
    alpha = zeros(N_stages, 1);
    for i = 0:N_stages-1
        alpha(i+1) = round((atan(2^(-i)) / pi) * SCALE);
    end

    % Coarse quadrant mapping (initial rotation)
    % Pre-rotation brings vector into right half-plane (x >= 0)
    x = int64(x_in);
    y = int64(y_in);
    z = int64(0);

    if x < 0
        if y >= 0
            x = -x;
            y = -y;
            z = int64(SCALE); % +pi
        else
            x = -x;
            y = -y;
            z = -int64(SCALE); % -pi
        end
    end

    % Iterative vectoring pipeline
    for i = 0:N_stages-1
        x_shift = bitshift(x, -i);
        y_shift = bitshift(y, -i);

        if y >= 0
            x = x + y_shift;
            y = y - x_shift;
            z = z + alpha(i+1);
        else
            x = x - y_shift;
            y = y + x_shift;
            z = z - alpha(i+1);
        end
    end

    % Saturate z to signed TOTAL_BITS range [-SCALE, SCALE - 1]
    max_val = SCALE - 1;
    min_val = -SCALE;
    if z > max_val, z = max_val; end
    if z < min_val, z = min_val; end

    theta_fixed = double(z);
    x_out = double(x);
    y_out = double(y);
end
