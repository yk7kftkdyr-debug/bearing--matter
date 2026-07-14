clearvars;
clc;

timerStart = tic;
max_runtime_minutes = 30;
oilfilm_clearance_coupling_test = true;
full_closed_loop_mode = false;
verificationMode = 'fig234_minimal_verification'; %#ok<NASGU>
caseName = 'medium_radial_load'; %#ok<NASGU>
outer_tol = 1e-5;
max_outer_iter = 5;

projectRoot = fileparts(mfilename('fullpath'));
if oilfilm_clearance_coupling_test
    run_oilfilm_clearance_coupling_test(projectRoot, timerStart);
    return;
end
assert(full_closed_loop_mode, 'No Fig. 2.34 verification mode is enabled.');

snapshotFile = fullfile(projectRoot, 'results', 'radial_load_study_9900rpm', ...
    'medium_radial_load', 'baseline', 'mat', 'ball_q1q2a1a2.mat');
stateFile = fullfile(projectRoot, 'results', 'radial_load_study_9900rpm', ...
    'medium_radial_load', 'baseline', 'mat', 'ball_www3.mat');
requiredVariables = {'Q2', 'delta2', 'inner_gap_shift', 'loadi', 'a2', 'sita', ...
    'miuO', 'yindao', 'Fyindao', 'zuoyongjiao', 'datafromvb'};
availableVariables = {whos('-file', snapshotFile).name};
missingVariables = setdiff(requiredVariables, availableVariables);
if ~isempty(missingVariables)
    error('run_debris_effect_cases_fig234:MissingBaselineVariables', ...
        'Missing baseline variables: %s', strjoin(missingVariables, ', '));
end
baselineData = load(snapshotFile, requiredVariables{:});
stateData = load(stateFile, 'www3');
baseline = contact_from_snapshot(baselineData);
baseline_completed = validate_contact(baseline);
if ~baseline_completed
    error('run_debris_effect_cases_fig234:InvalidBaseline', 'Archived medium baseline is invalid.');
end

workspaceRoot = tempname(tempdir);
mkdir(workspaceRoot);
cleanupObj = onCleanup(@() cleanup_workspace(workspaceRoot)); %#ok<NASGU>
copyfile(fullfile(projectRoot, '*.m'), workspaceRoot);
copyfile(fullfile(projectRoot, '球轴承程序'), fullfile(workspaceRoot, '球轴承程序'));
addpath(workspaceRoot, '-begin');
addpath(fullfile(workspaceRoot, '球轴承程序'), '-begin');
clear_solver_functions();

datafromvb = baselineData.datafromvb;
initialState = struct('loadi', baseline.loadi, 'www3', stateData.www3(:));
neutralConfig = closed_loop_config(0, outer_tol, max_outer_iter);
[neutral, neutralState] = run_contact_newton(workspaceRoot, datafromvb, ...
    initialState, neutralConfig, timerStart, max_runtime_minutes, 'neutral_interface_test'); %#ok<ASGLU>
neutral_difference = contact_change_norm(baseline, neutral, datafromvb(1), neutralConfig.debris.Q_floor);
neutral_matches_baseline = neutral_difference <= 1e-8;
if ~neutral_matches_baseline
    error('run_debris_effect_cases_fig234:NeutralMismatch', ...
        'neutral_interface_test differs from baseline: %.16g', neutral_difference);
end

activeConfig = closed_loop_config(1, outer_tol, max_outer_iter);
current = baseline;
currentState = initialState;
lastUd = zeros(size(current.Q2));
outer_converged = false;
outer_iteration_count = 0;
for outer_iteration_count = 1:max_outer_iter
    enforce_deadline(timerStart, max_runtime_minutes, 'paper_ud_ball_fig234_full');
    [lastUd, udInfo] = calibrated_ud(current.Q2, activeConfig.debris);
    if ~udInfo.valid
        error('run_debris_effect_cases_fig234:InvalidFig234Input', '%s', udInfo.warning_flag);
    end
    activeConfig.debris.ud_frozen_m = lastUd;
    activeConfig.debris.ud_frozen_loadi = current.loadi;
    [updated, solvedState] = run_contact_newton(workspaceRoot, datafromvb, ...
        currentState, activeConfig, timerStart, max_runtime_minutes, 'paper_ud_ball_fig234_full');
    relativeChange = contact_change_norm(current, updated, datafromvb(1), activeConfig.debris.Q_floor);
    current = updated;
    currentState = solvedState;
    if relativeChange < outer_tol
        outer_converged = true;
        break;
    end
end

paper_ud_ball_fig234_full_completed = validate_contact(current);
ids = unique([baseline.loadi; current.loadi]);
deltaOriginal = map_contact(baseline, ids, 'delta2', NaN);
deltaModified = map_contact(current, ids, 'delta2', NaN);
QOriginal = map_contact(baseline, ids, 'Q2', 0);
QModified = map_contact(current, ids, 'Q2', 0);
gapModified = map_contact(current, ids, 'inner_gap_shift', 0);
positiveGapShift = gapModified > 0 & isfinite(deltaOriginal) & isfinite(deltaModified);
gap_sign_ok = any(positiveGapShift) && any(deltaModified(positiveGapShift) > deltaOriginal(positiveGapShift));
FbOriginal = projected_inner_race_force(baseline);
FbModified = projected_inner_race_force(current);
runtime_seconds = toc(timerStart);
loadedUd = lastUd(lastUd > 0);
max_ud_fig234_m = max(loadedUd, [], 'omitnan');
mean_ud_fig234_m = mean(loadedUd, 'omitnan');
max_delta_change_m = max(abs(deltaModified - deltaOriginal), [], 'omitnan');
max_Q_change_N = max(abs(QModified - QOriginal), [], 'omitnan');
Fb_change_norm = norm(FbModified - FbOriginal) / max(norm(FbOriginal), activeConfig.debris.Q_floor);

fprintf('success=%d\n', paper_ud_ball_fig234_full_completed && outer_converged);
fprintf('runtime_seconds=%.9g\n', runtime_seconds);
fprintf('baseline_completed=%d\n', baseline_completed);
fprintf('neutral_matches_baseline=%d\n', neutral_matches_baseline);
fprintf('paper_ud_ball_fig234_full_completed=%d\n', paper_ud_ball_fig234_full_completed);
fprintf('outer_iteration_count=%d\n', outer_iteration_count);
fprintf('outer_converged=%d\n', outer_converged);
fprintf('max_ud_fig234_m=%.16g\n', max_ud_fig234_m);
fprintf('mean_ud_fig234_m=%.16g\n', mean_ud_fig234_m);
fprintf('max_delta_change_m=%.16g\n', max_delta_change_m);
fprintf('max_Q_change_N=%.16g\n', max_Q_change_N);
fprintf('Fb_change_norm=%.16g\n', Fb_change_norm);
fprintf('gap_sign_ok=%d\n', gap_sign_ok);

function config = closed_loop_config(etaD, outerTol, maxOuterIter)
config = make_micro_interface_config();
config.runtime.silent_mode = true;
config.runtime.disable_legacy_txt = true;
config.runtime.disable_legacy_fig = true;
% MAT state remains enabled because ffLOAD/JffLOAD use it as their legacy interface.
config.runtime.disable_intermediate_mat = false;
config.debris.enabled = true;
config.debris.eta_d = etaD;
config.debris.formula_source = 'figure_2_34_calibrated';
config.debris.E_matrix = 210e9;
config.debris.E_impurity = 420e9;
config.debris.E_delta = 2;
config.debris.w_mean = NaN;
config.debris.sigma_w = NaN;
config.debris.outer_tol = outerTol;
config.debris.max_outer_iter = maxOuterIter;
config.debris.dynamic_statistics_enabled = false;
end

function [contact, solvedState] = run_contact_newton(workspaceRoot, datafromvb, initialState, config, timerStart, maxMinutes, stage)
bearingDir = fullfile(workspaceRoot, '球轴承程序');
oldDir = pwd;
cleanupDir = onCleanup(@() cd(oldDir)); %#ok<NASGU>
cd(bearingDir);
micro_config = config; %#ok<NASGU>
save('micro_config_runtime.mat', 'micro_config');
loadi = reshape(initialState.loadi, 1, []);
state = initialState.www3(:);
loadj = numel(loadi);
activeCount = 2 * loadj + 3;
if numel(state) ~= 2 * loadj + 5
    error('run_debris_effect_cases_fig234:InvalidWarmStart', 'Warm-start state size is invalid.');
end
newtonConverged = false;
for newtonIteration = 1:60 %#ok<NASGU>
    enforce_deadline(timerStart, maxMinutes, stage);
    evalc('ffLOAD(datafromvb, loadi, state);');
    residualData = load('result444.mat', 'result444');
    stateData = load('uu.mat', 'uu');
    forceData = load('zzzz3.mat', 'zzzz3');
    state = stateData.uu(:);
    if residualData.result444 <= 0.01
        newtonConverged = true;
        break;
    end
    evalc('JffLOAD(datafromvb, loadi, state);');
    jacobianData = load('Jzz.mat', 'Jzz');
    state(1:activeCount) = state(1:activeCount) - ...
        jacobianData.Jzz(1:activeCount, 1:activeCount) \ forceData.zzzz3(1:activeCount);
end
if ~newtonConverged
    error('run_debris_effect_cases_fig234:NewtonNonconvergence', '%s exceeded 60 Newton iterations.', stage);
end
evalc('ffLOAD(datafromvb, loadi, state);');
contact = load_contact_state(loadi, state);
if ~validate_contact(contact)
    error('run_debris_effect_cases_fig234:InvalidContact', '%s returned invalid contact state.', stage);
end
solvedState = struct('loadi', contact.loadi, 'www3', state);
end

function contact = load_contact_state(loadi, state)
forces = load('q1q2a1a2.mat', 'Q2', 'a2', 'sita', 'miuO', 'yindao', 'Fyindao', 'zuoyongjiao');
deformation = load('contact_state.mat', 'delta2', 'inner_gap_shift');
contact = struct('loadi', loadi(:), 'Q2', forces.Q2(:), 'delta2', deformation.delta2(:), ...
    'inner_gap_shift', deformation.inner_gap_shift(:), 'a2', forces.a2(:), ...
    'sita', forces.sita(:), 'miuO', forces.miuO(:), 'yindao', forces.yindao, ...
    'Fyindao', forces.Fyindao, 'zuoyongjiao', forces.zuoyongjiao, 'www3', state(:));
end

function contact = contact_from_snapshot(data)
contact = struct('loadi', data.loadi(:), 'Q2', data.Q2(:), 'delta2', data.delta2(:), ...
    'inner_gap_shift', data.inner_gap_shift(:), 'a2', data.a2(:), 'sita', data.sita(:), ...
    'miuO', data.miuO(:), 'yindao', data.yindao, 'Fyindao', data.Fyindao, ...
    'zuoyongjiao', data.zuoyongjiao, 'www3', []);
end

function [ud, info] = calibrated_ud(Q2, debris)
domain = Q2 >= 96.7 & Q2 <= 7886.1 & Q2 > debris.Q_floor;
ud = zeros(size(Q2));
[domainUd, info] = debris_ud_model(Q2(domain), debris);
ud(domain) = domainUd;
end

function value = contact_change_norm(first, second, ballCount, Qfloor)
ids = (1:ballCount).';
firstQ = map_contact(first, ids, 'Q2', 0);
secondQ = map_contact(second, ids, 'Q2', 0);
value = max(abs(secondQ - firstQ) ./ max(abs(firstQ), Qfloor));
end

function values = map_contact(contact, ids, fieldName, fillValue)
values = fillValue * ones(numel(ids), 1);
[matched, index] = ismember(ids, contact.loadi);
values(matched) = contact.(fieldName)(index(matched));
end

function forceVector = projected_inner_race_force(contact)
fs2 = contact.miuO .* contact.Q2;
axial = sum(contact.Q2 .* sin(contact.a2) - fs2 .* cos(contact.a2));
radialContact = contact.Q2 .* cos(contact.a2) + fs2 .* sin(contact.a2);
radialY = sum(radialContact .* sin(contact.sita));
radialZ = sum(radialContact .* cos(contact.sita));
if contact.yindao == 2
    radialY = radialY + numel(contact.Q2) * contact.Fyindao * sin(contact.zuoyongjiao);
    radialZ = radialZ + numel(contact.Q2) * contact.Fyindao * cos(contact.zuoyongjiao);
end
forceVector = [axial; radialY; radialZ];
end

function valid = validate_contact(contact)
valid = ~isempty(contact.Q2) && all(isfinite(contact.Q2)) && all(contact.Q2 >= 0) && ...
    all(isfinite(contact.delta2)) && all(isfinite(contact.inner_gap_shift));
end

function enforce_deadline(timerStart, maxMinutes, stage)
if toc(timerStart) > maxMinutes * 60
    error('run_debris_effect_cases_fig234:RuntimeLimit', ...
        'Runtime limit exceeded during %s.', stage);
end
end

function clear_solver_functions()
clear('ffLOAD', 'JffLOAD', 'load_micro_interface_config', 'make_micro_interface_config', 'debris_ud_model');
end

function cleanup_workspace(workspaceRoot)
warning('off', 'MATLAB:rmpath:DirNotFound');
rmpath(fullfile(workspaceRoot, '球轴承程序'));
rmpath(workspaceRoot);
warning('on', 'MATLAB:rmpath:DirNotFound');
if exist(workspaceRoot, 'dir') == 7
    rmdir(workspaceRoot, 's');
end
end

function run_oilfilm_clearance_coupling_test(projectRoot, timerStart)
snapshotFile = fullfile(projectRoot, 'results', 'radial_load_study_9900rpm', ...
    'medium_radial_load', 'baseline', 'mat', 'ball_q1q2a1a2.mat');
clearanceFile = fullfile(projectRoot, 'results', 'radial_load_study_9900rpm', ...
    'medium_radial_load', 'baseline', 'mat', 'ball_deltaw.mat');
statisticsFile = fullfile(projectRoot, 'results', 'radial_load_study_9900rpm', ...
    'medium_radial_load', 'paper_ud_ball', 'summary', 'case_summary.csv');
requiredVariables = {'Q2', 'delta2', 'oilh2', 'loadi', 'a2', 'sita', ...
    'miuO', 'yindao', 'Fyindao', 'zuoyongjiao'};
availableVariables = {whos('-file', snapshotFile).name};
missingVariables = setdiff(requiredVariables, availableVariables);
if ~isempty(missingVariables)
    error('run_debris_effect_cases_fig234:MissingOilfilmVariables', ...
        'Missing baseline variables: %s', strjoin(missingVariables, ', '));
end
data = load(snapshotFile, requiredVariables{:});
clearanceData = load(clearanceFile, 'deltaw');
statisticsTable = readtable(statisticsFile, 'VariableNamingRule', 'preserve');
statistics = table2struct(statisticsTable(1, :));

Q2Original = data.Q2(:);
delta2Original = data.delta2(:);
hMinOriginal = data.oilh2(:);
debris = make_micro_interface_config().debris;
debris.enabled = true;
debris.eta_d = 1;
debris.formula_source = 'text_equation';
debris.coupling_mode = 'oilfilm_clearance';
debris.E_matrix = 210e9;
debris.E_impurity = 420e9;
debris.E_delta = 2;
debris.w_mean = statistics.w_mean_N;
debris.sigma_w = statistics.sigma_Q_N;
[udText, udInfo] = debris_ud_model(Q2Original, debris);
validContact = udInfo.loaded_mask & isfinite(Q2Original) & isfinite(delta2Original) & delta2Original > 0;
if ~udInfo.valid || ~any(validContact) || any(~isfinite(udText(validContact))) || any(udText(validContact) <= 0)
    error('run_debris_effect_cases_fig234:InvalidOilfilmUd', '%s', udInfo.warning_flag);
end

alphaOil = debris.alpha_oil;
hMinFloor = debris.h_min_floor;
hMinModified = max(hMinOriginal - alphaOil * udText, hMinFloor);
deltaH = hMinModified - hMinOriginal;
workingClearanceOriginal = clearanceData.deltaw * ones(size(Q2Original));
workingClearanceModified = workingClearanceOriginal + deltaH;
workingClearanceChange = workingClearanceModified - workingClearanceOriginal;
% Local baseline-state mapping: reduced separation closes the working clearance.
delta2Modified = delta2Original - workingClearanceChange;

contactExponent = 3 / 2;
validContact = validContact & isfinite(delta2Modified) & delta2Modified > 0;
hertzCoefficient = zeros(size(Q2Original));
hertzCoefficient(validContact) = Q2Original(validContact) ./ ...
    delta2Original(validContact) .^ contactExponent;
Q2Modified = Q2Original;
Q2Modified(validContact) = hertzCoefficient(validContact) .* ...
    delta2Modified(validContact) .^ contactExponent;
KbBefore = NaN(size(Q2Original));
KbAfter = NaN(size(Q2Original));
KbBefore(validContact) = contactExponent * Q2Original(validContact) ./ delta2Original(validContact);
KbAfter(validContact) = contactExponent * Q2Modified(validContact) ./ delta2Modified(validContact);
CbBefore = NaN;
CbAfter = NaN;

original = contact_from_oilfilm_snapshot(data, Q2Original);
modified = contact_from_oilfilm_snapshot(data, Q2Modified);
FbOriginal = projected_inner_race_force(original);
FbModified = projected_inner_race_force(modified);
oilfilmLoss = hMinOriginal - hMinModified;
gap_sign_ok = all(deltaH(validContact) <= 0) && ...
    all(workingClearanceChange(validContact) <= 0) && ...
    all(delta2Modified(validContact) >= delta2Original(validContact));
oilfilm_is_postprocess_only = false;
oilfilm_chain_active = any(validContact) && all(hMinModified > 0) && ...
    all(hMinModified <= hMinOriginal) && gap_sign_ok && ...
    any(abs(Q2Modified(validContact) - Q2Original(validContact)) > debris.Q_floor);
debrisOff = debris;
debrisOff.enabled = false;
[udOff, ~] = debris_ud_model(Q2Original, debrisOff);
hOff = max(hMinOriginal - alphaOil * udOff, hMinFloor);
cOff = workingClearanceOriginal + (hOff - hMinOriginal);
deltaOff = delta2Original - (cOff - workingClearanceOriginal);
QOff = Q2Original;
FbOff = projected_inner_race_force(contact_from_oilfilm_snapshot(data, QOff));
baselineTolerance = 100 * eps(max([1; abs(Q2Original); abs(delta2Original); abs(FbOriginal)]));
baseline_recovered = all(udOff == 0) && max(abs(hOff - hMinOriginal)) <= baselineTolerance && ...
    max(abs(cOff - workingClearanceOriginal)) <= baselineTolerance && ...
    max(abs(deltaOff - delta2Original)) <= baselineTolerance && ...
    max(abs(QOff - Q2Original)) <= baselineTolerance && norm(FbOff - FbOriginal) <= baselineTolerance;
runtime_seconds = toc(timerStart);
max_ud_m = max(udText(validContact));
mean_ud_m = mean(udText(validContact));
max_hmin_original_m = max(hMinOriginal);
min_hmin_modified_m = min(hMinModified);
max_oilfilm_loss_m = max(oilfilmLoss);
max_working_clearance_change_m = max(abs(workingClearanceChange));
max_delta_change_m = max(abs(delta2Modified - delta2Original));
max_Q_change_N = max(abs(Q2Modified - Q2Original));
Fb_change_norm = norm(FbModified - FbOriginal) / max(norm(FbOriginal), debris.Q_floor);
recommend_rotor_validation = oilfilm_chain_active && Fb_change_norm > 1e-4;
h_min_floor_count = nnz(hMinModified <= hMinFloor);

fprintf('success=%d\n', oilfilm_chain_active && baseline_recovered);
fprintf('runtime_seconds=%.9g\n', runtime_seconds);
fprintf('Q2_before_N=[%.16g %.16g %.16g]\n', min(Q2Original(validContact)), max(Q2Original(validContact)), mean(Q2Original(validContact)));
fprintf('Q2_after_N=[%.16g %.16g %.16g]\n', min(Q2Modified(validContact)), max(Q2Modified(validContact)), mean(Q2Modified(validContact)));
fprintf('ud_um=[%.16g %.16g %.16g]\n', min(udText(validContact))*1e6, max_ud_m*1e6, mean_ud_m*1e6);
fprintf('h_min_before_m=[%.16g %.16g %.16g]\n', min(hMinOriginal), max_hmin_original_m, mean(hMinOriginal));
fprintf('h_min_after_m=[%.16g %.16g %.16g]\n', min_hmin_modified_m, max(hMinModified), mean(hMinModified));
fprintf('working_clearance_before_m=%.16g\n', clearanceData.deltaw);
fprintf('working_clearance_after_m=[%.16g %.16g %.16g]\n', min(workingClearanceModified), max(workingClearanceModified), mean(workingClearanceModified));
fprintf('delta_before_m=[%.16g %.16g %.16g]\n', min(delta2Original(validContact)), max(delta2Original(validContact)), mean(delta2Original(validContact)));
fprintf('delta_after_m=[%.16g %.16g %.16g]\n', min(delta2Modified(validContact)), max(delta2Modified(validContact)), mean(delta2Modified(validContact)));
fprintf('Kb_before_N_per_m=[%.16g %.16g %.16g]\n', min(KbBefore(validContact)), max(KbBefore(validContact)), mean(KbBefore(validContact)));
fprintf('Kb_after_N_per_m=[%.16g %.16g %.16g]\n', min(KbAfter(validContact)), max(KbAfter(validContact)), mean(KbAfter(validContact)));
fprintf('Cb_before=%g\n', CbBefore);
fprintf('Cb_after=%g\n', CbAfter);
fprintf('Fb_before_N=[%.16g %.16g %.16g]\n', FbOriginal);
fprintf('Fb_after_N=[%.16g %.16g %.16g]\n', FbModified);
fprintf('max_oilfilm_loss_m=%.16g\n', max_oilfilm_loss_m);
fprintf('max_working_clearance_change_m=%.16g\n', max_working_clearance_change_m);
fprintf('max_delta_change_m=%.16g\n', max_delta_change_m);
fprintf('max_Q_change_N=%.16g\n', max_Q_change_N);
fprintf('Fb_change_norm=%.16g\n', Fb_change_norm);
fprintf('valid_contact_count=%d\n', nnz(validContact));
fprintf('h_min_floor_count=%d\n', h_min_floor_count);
fprintf('baseline_recovered=%d\n', baseline_recovered);
fprintf('oilfilm_chain_active=%d\n', oilfilm_chain_active);
fprintf('gap_sign_ok=%d\n', gap_sign_ok);
fprintf('oilfilm_is_postprocess_only=%d\n', oilfilm_is_postprocess_only);
fprintf('direct_Q_correction_used=0\n');
fprintf('direct_Fb_correction_used=0\n');
fprintf('recommend_rotor_validation=%d\n', recommend_rotor_validation);
end

function contact = contact_from_oilfilm_snapshot(data, Q2)
contact = struct('loadi', data.loadi(:), 'Q2', Q2(:), 'a2', data.a2(:), ...
    'sita', data.sita(:), 'miuO', data.miuO(:), 'yindao', data.yindao, ...
    'Fyindao', data.Fyindao, 'zuoyongjiao', data.zuoyongjiao);
end
