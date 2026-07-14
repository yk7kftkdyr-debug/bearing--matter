function curve = calculate_debris_formula_curve(Q_N, debris)
%CALCULATE_DEBRIS_FORMULA_CURVE Evaluate paper equation (2-60) for diagnostics.

Q_N = Q_N(:);
[ud_m, info] = debris_ud_model(Q_N, debris);
if ~info.valid
    error('calculate_debris_formula_curve:InvalidPaperInputs', ...
        'Paper equation inputs are invalid: %s', info.warning_flag);
end

curve = struct();
curve.Q_N = Q_N;
curve.w_norm = info.w_norm(:);
curve.ud_um = info.ud_um(:);
curve.ud_m = ud_m(:);
curve.output_unit = 'um_then_converted_to_m';
curve.equation = ['0.0124*E_delta^0.11*(exp(-5.432e-5*' ...
    '(w_norm+0.0186)^2)-0.01*w_norm)'];
end
