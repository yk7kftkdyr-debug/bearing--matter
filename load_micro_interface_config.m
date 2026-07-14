function micro_config = load_micro_interface_config()
%LOAD_MICRO_INTERFACE_CONFIG Load runtime micro-interface configuration.

if exist('micro_config_runtime.mat', 'file') == 2
    data = load('micro_config_runtime.mat', 'micro_config');
    if isfield(data, 'micro_config')
        micro_config = normalize_micro_config(data.micro_config);
        return;
    end
end

micro_config = make_micro_interface_config();
end

function micro_config = normalize_micro_config(micro_config)
micro_config = merge_micro_config(make_micro_interface_config(), micro_config);
micro_config.debris.enabled = logical(micro_config.debris.enabled);
micro_config.debris.dynamic_statistics_enabled = logical(micro_config.debris.dynamic_statistics_enabled);
micro_config.debris.statistics_frozen = logical(micro_config.debris.statistics_frozen);
end

function base = merge_micro_config(base, overrides)
if ~isstruct(overrides)
    return;
end
for i = 1:numel(fieldnames(overrides))
    name = fieldnames(overrides);
    name = name{i};
    if isfield(base, name) && isstruct(base.(name)) && isstruct(overrides.(name))
        base.(name) = merge_micro_config(base.(name), overrides.(name));
    else
        base.(name) = overrides.(name);
    end
end
end
