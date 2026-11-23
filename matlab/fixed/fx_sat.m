function y = fx_sat(x, min_val, max_val)
% FX_SAT Bit-accurate hardware saturation clamp to [min_val, max_val].
%
%   y = fx_sat(x, min_val, max_val)

y = min(max(x, min_val), max_val);

end
