function [ud_m, info] = debris_ud_model(Q2, debris)
%DEBRIS_UD_MODEL Paper equation (2-60) for loaded ball-inner-race contacts.

ud_m = zeros(size(Q2));
formulaSource = 'text_equation';
if isfield(debris, 'formula_source') && ~isempty(debris.formula_source)
    formulaSource = char(debris.formula_source);
end
info = struct('enabled', logical(debris.enabled), 'valid', true, 'warning_flag', '', ...
    'loaded_mask', false(size(Q2)), 'w_norm', zeros(size(Q2)), 'ud_um', zeros(size(Q2)), ...
    'w_norm_min', NaN, 'w_norm_max', NaN, 'max_ud_m', 0, 'mean_ud_m', 0, ...
    'formula_source', formulaSource);
if ~info.enabled || debris.eta_d == 0
    return;
end

info.loaded_mask = Q2 > debris.Q_floor;
if ~any(info.loaded_mask(:))
    return;
end

ratio = debris.E_impurity / debris.E_matrix;
if ~all(isfinite([debris.E_impurity, debris.E_matrix, debris.E_delta])) || ...
        debris.E_impurity <= 0 || debris.E_matrix <= 0 || ...
        abs(debris.E_delta - ratio) > 100 * eps(max(abs([debris.E_delta, ratio])))
    info.valid = false;
    info.warning_flag = 'invalid_debris_config';
    return;
end
switch formulaSource
    case 'text_equation'
        if ~all(isfinite([debris.w_mean, debris.sigma_w])) || debris.sigma_w <= debris.sigma_floor
            info.valid = false;
            info.warning_flag = 'invalid_debris_config';
            return;
        end
        info.w_norm(info.loaded_mask) = (Q2(info.loaded_mask) - debris.w_mean) ./ debris.sigma_w;
        info.w_norm_min = min(info.w_norm(info.loaded_mask));
        info.w_norm_max = max(info.w_norm(info.loaded_mask));
        if any(Q2(info.loaded_mask) > 8000)
            info.valid = false;
            info.warning_flag = 'paper_load_range_warning';
            return;
        end
        info.ud_um(info.loaded_mask) = 0.0124 * debris.E_delta^0.11 .* ...
            (exp(-5.432e-5 * (info.w_norm(info.loaded_mask) + 0.0186).^2) - 0.01 * info.w_norm(info.loaded_mask));
    case 'figure_2_34_calibrated'
        % Source: digitized curve from Fig. 2.34
        % Unit: w in N, ud in um
        fig234_w_N = [96.7, 347.2, 843.0, 1230.9, 1462.6, 2273.0, ...
            2992.1, 3410.3, 3812.0, 4395.5, 4617.5, 5327.7, 6052.3, ...
            6691.2, 7201.3, 7561.0, 7752.3, 7886.1];
        fig234_ud_um = [0.2646, 0.5061, 0.9474, 1.2669, 1.4572, 2.0018, ...
            2.4255, 2.6434, 2.8450, 3.1124, 3.1986, 3.5007, 3.7966, ...
            4.0566, 4.2580, 4.4201, 4.5033, 4.5714];
        loadedQ = Q2(info.loaded_mask);
        if isempty(fig234_w_N) || isempty(fig234_ud_um)
            info.valid = false;
            info.warning_flag = 'missing_fig234_digitized_points';
            return;
        end
        if any(loadedQ < fig234_w_N(1) | loadedQ > fig234_w_N(end))
            info.valid = false;
            info.warning_flag = 'fig234_load_out_of_domain';
            return;
        end
        info.ud_um(info.loaded_mask) = interp1(fig234_w_N, fig234_ud_um, loadedQ, 'pchip');
    otherwise
        info.valid = false;
        info.warning_flag = 'unknown_formula_source';
        return;
end
ud_m = info.ud_um * 1e-6;
info.max_ud_m = max(ud_m(info.loaded_mask));
info.mean_ud_m = mean(ud_m(info.loaded_mask));
if any(ud_m(info.loaded_mask) > 10e-6)
    info.warning_flag = 'paper_curve_mismatch_warning';
elseif any(ud_m(info.loaded_mask) <= 0)
    info.warning_flag = 'nonpositive_ud_warning';
end
end
