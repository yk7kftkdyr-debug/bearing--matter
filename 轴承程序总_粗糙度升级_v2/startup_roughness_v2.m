%STARTUP_ROUGHNESS_V2 Establish an isolated, reproducible project path.
restoredefaultpath;
clear functions;
rehash toolboxcache;

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot);
addpath(fullfile(projectRoot, '球轴承程序'));
addpath(fullfile(projectRoot, '滚子轴承程序'));
addpath(fullfile(projectRoot, 'tests'));
addpath(fullfile(projectRoot, 'cases'));

requiredFunctions = {'qiujieend', 'qiujieall', 'ffLOAD', 'ff2', 'ff3', ...
    'make_micro_interface_config'};
for k = 1:numel(requiredFunctions)
    resolved = which(requiredFunctions{k}, '-all');
    if isempty(resolved)
        error('startup_roughness_v2:MissingFunction', ...
            'Required function "%s" is not on the project path.', requiredFunctions{k});
    end
    fprintf('%s\n', strjoin(resolved, newline));
end
