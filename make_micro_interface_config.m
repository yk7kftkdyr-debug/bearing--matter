function micro_config = make_micro_interface_config(varargin)
%MAKE_MICRO_INTERFACE_CONFIG Unified micro-interface parameter defaults.
% Defaults describe the baseline undisturbed case used by the base examples.

micro_config = struct();

micro_config.runtime = struct();
micro_config.runtime.silent_mode = false;
micro_config.runtime.disable_legacy_txt = false;
micro_config.runtime.disable_legacy_fig = false;
micro_config.runtime.disable_intermediate_mat = false;

micro_config.thermal = struct();
micro_config.thermal.enabled = false;
micro_config.thermal.temperature_C = 20;
micro_config.thermal.viscosity_factor = 1;
micro_config.thermal.thermal_clearance_factor = 1;
micro_config.thermal.Ct1 = 1;
micro_config.thermal.Ct2 = 1;

micro_config.texture = struct();
micro_config.texture.enabled = false;
micro_config.texture.texture_type = 'none';
micro_config.texture.texture_depth = 0;
micro_config.texture.texture_width = 0;
micro_config.texture.texture_density = 0;
micro_config.texture.Cr = 1;
micro_config.texture.wenli = 6;

micro_config.debris = struct();
micro_config.debris.enabled = false;
micro_config.debris.E_impurity = 210e9;
micro_config.debris.E_matrix = 210e9;
micro_config.debris.E_delta = 1;
micro_config.debris.eta_d = 0;
micro_config.debris.w_mean = NaN;
micro_config.debris.sigma_w = NaN;
micro_config.debris.Q_floor = 1e-9;
micro_config.debris.sigma_floor = 1e-12;
micro_config.debris.formula_source = 'text_equation';
micro_config.debris.coupling_mode = 'mechanical_gap';
micro_config.debris.alpha_oil = 1;
micro_config.debris.h_min_floor = 1e-9;
micro_config.debris.dynamic_statistics_enabled = true;
micro_config.debris.N_theta = 360;
micro_config.debris.cage_phase_rad = 0;
micro_config.debris.statistics_frozen = false;
micro_config.debris.statistics_source = 'unset';
micro_config.debris.outer_tol = 1e-6;
micro_config.debris.max_outer_iter = 10;
micro_config.debris.ud_frozen_m = [];
micro_config.debris.ud_frozen_loadi = [];

micro_config.lubricant = struct();
micro_config.lubricant.enabled = false;
micro_config.lubricant.eta0 = [];
micro_config.lubricant.alpha_p = [];
micro_config.lubricant.ehl_correction_factor = 1;

if nargin == 0
    return;
end

if nargin == 1 && isstruct(varargin{1})
    micro_config = merge_micro_config(micro_config, varargin{1});
else
    if mod(nargin, 2) ~= 0
        error('make_micro_interface_config:InvalidInput', ...
            'Overrides must be provided as name-value pairs or one struct.');
    end
    overrides = struct();
    for k = 1:2:nargin
        overrides.(varargin{k}) = varargin{k + 1};
    end
    micro_config = merge_micro_config(micro_config, overrides);
end
end

function base = merge_micro_config(base, overrides)
names = fieldnames(overrides);
for i = 1:numel(names)
    name = names{i};
    if isfield(base, name) && isstruct(base.(name)) && isstruct(overrides.(name))
        base.(name) = merge_micro_config(base.(name), overrides.(name));
    else
        base.(name) = overrides.(name);
    end
end
end
