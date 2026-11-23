function e = gardner_ted_float(i_curr, q_curr, i_prev, q_prev, i_mid, q_mid)
% GARDNER_TED_FLOAT Non-data-aided Gardner Timing Error Detector (double).
%
%   e = gardner_ted_float(i_curr, q_curr, i_prev, q_prev, i_mid, q_mid)
%
%   Implements:
%       e[k] = (I_k - I_{k-1}) * I_{k - 1/2} + (Q_k - Q_{k-1}) * Q_{k - 1/2}
%
%   Sign convention:
%       e[k] > 0  => sampling LATE  (sampling instant too large)
%       e[k] < 0  => sampling EARLY (sampling instant too small)

e = (i_curr - i_prev) .* i_mid + (q_curr - q_prev) .* q_mid;

end
