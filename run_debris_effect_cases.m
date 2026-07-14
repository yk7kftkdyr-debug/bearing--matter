clear; clc;

projectRoot = fileparts(mfilename('fullpath'));
resultsRoot = fullfile(projectRoot, 'results', 'radial_load_study_9900rpm');
ensure_dir(resultsRoot);
cases = build_cases();
records = repmat(empty_record(), 0, 1);

for iCase = 1:numel(cases)
    caseInfo = cases(iCase);
    caseDirs = prepare_case_dirs(fullfile(resultsRoot, caseInfo.group_name), caseInfo.case_name);
    workspaceRoot = fullfile(caseDirs.root, 'workspace');
    copy_project_workspace(projectRoot, workspaceRoot);
    add_workspace_paths(workspaceRoot);
    clear_workspace_functions();

    try
        [ballRecord, ballContact] = run_ball_case(workspaceRoot, caseInfo);
        archive_case_outputs(workspaceRoot, 'ball', caseDirs);
        write_contact_diagnostics(caseDirs.summary, ballContact, ballRecord, caseInfo.eta_d);
        write_dynamic_statistics(caseDirs.summary, ballContact);
        caseState = struct('ball_contact', ballContact, 'ball_record', ballRecord);
        save(fullfile(caseDirs.summary, 'case_state.mat'), 'caseState');
        writetable(struct2table(ballRecord), fullfile(caseDirs.summary, 'case_summary.csv'), 'Encoding', 'UTF-8');
        records(end + 1, 1) = ballRecord; %#ok<SAGROW>
    catch ME
        write_error_note(caseDirs.summary, ME);
        records(end + 1, 1) = error_record('ball', caseInfo, ME); %#ok<SAGROW>
    end

    remove_workspace_paths(workspaceRoot);
    if exist(workspaceRoot, 'dir') == 7
        rmdir(workspaceRoot, 's');
    end
    write_case_note(caseDirs, caseInfo);
end

records = validate_neutral_interface(records);
groupNames = unique({cases.group_name}, 'stable');
for iGroup = 1:numel(groupNames)
    groupRecords = records(strcmp({records.group_name}, groupNames{iGroup}));
    writetable(struct2table(groupRecords), fullfile(resultsRoot, groupNames{iGroup}, 'summary.csv'), 'Encoding', 'UTF-8');
end
writetable(struct2table(records), fullfile(resultsRoot, 'summary.csv'), 'Encoding', 'UTF-8');
write_plot_csv(resultsRoot, records);
write_run_note(resultsRoot);
disp('DEBRIS_EFFECT_CASES_DONE');
disp(fullfile(resultsRoot, 'summary.csv'));

function cases = build_cases()
study = [struct('group_name', 'low_radial_load', 'radial_load_N', 10000); ...
    struct('group_name', 'medium_radial_load', 'radial_load_N', 15000); ...
    struct('group_name', 'high_radial_load', 'radial_load_N', 17000)];
cases = repmat(struct('group_name', '', 'case_name', '', 'enabled', false, 'E_delta', 1, ...
    'eta_d', 0, 'speed_rpm', 9900, 'axial_load_N', 20000, 'radial_load_N', 0, 'temperature_C', 20), 0, 1);
for i = 1:numel(study)
    baseline = struct('group_name', study(i).group_name, 'case_name', 'baseline', 'enabled', false, ...
        'E_delta', 1, 'eta_d', 0, 'speed_rpm', 9900, 'axial_load_N', 20000, ...
        'radial_load_N', study(i).radial_load_N, 'temperature_C', 20);
    paper = baseline;
    paper.case_name = 'paper_ud_ball';
    paper.enabled = true;
    paper.E_delta = 2;
    paper.eta_d = 1;
    neutral = baseline;
    neutral.case_name = 'neutral_interface_test';
    neutral.enabled = true;
    cases(end + 1, 1) = baseline; %#ok<SAGROW>
    cases(end + 1, 1) = neutral; %#ok<SAGROW>
    cases(end + 1, 1) = paper; %#ok<SAGROW>
end
end

function [record, contactPair] = run_ball_case(workspaceRoot, caseInfo)
datafromvb = default_ball_input();
datafromvb(15) = caseInfo.speed_rpm;
datafromvb(16) = caseInfo.axial_load_N;
datafromvb(17) = 0;
datafromvb(18) = caseInfo.radial_load_N;
datafromvb(27) = caseInfo.temperature_C;
if ~strcmp(caseInfo.case_name, 'paper_ud_ball')
    config = case_config(caseInfo);
    [record, modified] = run_ball_solver(workspaceRoot, datafromvb, config, caseInfo);
    record.Fb_original = sum(modified.Q2);
    record.Fb_modified = record.Fb_original;
    record = fill_contact_change_metrics(record, modified, modified, config.debris.Q_floor);
    contactPair = struct('original', modified, 'modified', modified, 'ud_info', empty_ud_info(), ...
        'ud_loadi', modified.loadi);
    return;
end

baselineInfo = caseInfo;
baselineInfo.enabled = false;
baselineInfo.eta_d = 0;
[baselineRecord, original] = run_ball_solver(workspaceRoot, datafromvb, case_config(baselineInfo), baselineInfo);
contactPair = struct('original', original, 'modified', original, 'ud_info', empty_ud_info(), 'ud_loadi', original.loadi);
record = baselineRecord;
record.case_name = caseInfo.case_name;
record.model_scope = 'paper_ud_ball_inner_race_only';
record.E_delta = caseInfo.E_delta;
record.eta_d = caseInfo.eta_d;
record.Fb_original = sum(original.Q2);

activeConfig = case_config(caseInfo);
debris = activeConfig.debris;
loadedQ = original.Q2(original.Q2 > debris.Q_floor);
if numel(loadedQ) < 2
    record = invalid_record(record, 'invalid_load_statistics', 'fewer than two loaded ball-inner-race contacts');
    return;
end
oldDebris = debris;
oldDebris.w_mean = mean(loadedQ);
oldDebris.sigma_w = std(loadedQ, 0);
oldDebris.statistics_source = 'single_steady_spatial_sample';
[udOld, ~] = debris_ud_model(original.Q2, oldDebris);

cageSpeed = estimate_ball_cage_speed(datafromvb);
dynamicSolver = @(phaseRad) solve_baseline_at_cage_phase(workspaceRoot, datafromvb, ...
    baselineInfo, phaseRad);
dynamicStats = calculate_dynamic_debris_statistics(datafromvb, datafromvb(1), cageSpeed, original, debris, dynamicSolver);
contactPair.dynamic_stats = dynamicStats;
debris.w_mean = dynamicStats.w_mean;
debris.sigma_w = dynamicStats.sigma_Q;
debris.statistics_frozen = true;
debris.statistics_source = 'dynamic_cage_period';
if ~isfinite(debris.sigma_w) || debris.sigma_w <= debris.sigma_floor
    record = invalid_record(record, 'invalid_load_statistics', 'dynamic baseline Q2 standard deviation is too small');
    return;
end
formulaQ = linspace(dynamicStats.Q_min, dynamicStats.Q_max, 361).';
dynamicStats.formula_curve = calculate_debris_formula_curve(formulaQ, debris);
contactPair.dynamic_stats = dynamicStats;
record.formula_ud_min_um = min(dynamicStats.formula_curve.ud_um, [], 'omitnan');
record.formula_ud_max_um = max(dynamicStats.formula_curve.ud_um, [], 'omitnan');
record.w_mean_old = oldDebris.w_mean;
record.sigma_w_old = oldDebris.sigma_w;
record.max_ud_old_m = max(udOld(:), [], 'omitnan');
record.w_mean_N = debris.w_mean;
record.sigma_Q_N = debris.sigma_w;
record.Q_sample_count = dynamicStats.Q_sample_count;
record.Q_out_of_domain_count = dynamicStats.Q_out_of_domain_count;
record.Q_min_N = dynamicStats.Q_min;
record.Q_max_N = dynamicStats.Q_max;
record.w_norm_min = dynamicStats.w_norm_min;
record.w_norm_max = dynamicStats.w_norm_max;
if record.formula_ud_max_um < 0.1
    record.warning_flag = merge_warning(record.warning_flag, 'paper_figure_scale_mismatch_warning');
    record.comment = append_comment(record.comment, ...
        'equation (2-60) with runtime w_mean/sigma_w yields <0.1 um, inconsistent with Fig. 2.34 scale; no scaling applied');
end
statisticsRecord = struct('w_mean_old', oldDebris.w_mean, 'sigma_w_old', oldDebris.sigma_w, ...
    'max_ud_old_m', record.max_ud_old_m, 'w_mean_N', debris.w_mean, 'sigma_Q_N', debris.sigma_w, ...
    'Q_sample_count', dynamicStats.Q_sample_count, 'Q_min_N', dynamicStats.Q_min, 'Q_max_N', dynamicStats.Q_max, ...
    'Q_out_of_domain_count', dynamicStats.Q_out_of_domain_count, ...
    'formula_ud_min_um', record.formula_ud_min_um, 'formula_ud_max_um', record.formula_ud_max_um, ...
    'w_norm_min', dynamicStats.w_norm_min, 'w_norm_max', dynamicStats.w_norm_max);
if isfinite(dynamicStats.w_norm_min) && isfinite(dynamicStats.w_norm_max) && ...
        dynamicStats.w_norm_min > -1.2 && dynamicStats.w_norm_max < 1.2
    record.warning_flag = merge_warning(record.warning_flag, 'dynamic_window_range_warning');
    record.comment = append_comment(record.comment, 'dynamic w_norm range remains close to [-1,1]');
end

Q2 = original.Q2;
loadi = original.loadi;
lastInfo = empty_ud_info();
relativeChange = Inf;
for outerIter = 1:debris.max_outer_iter
    [ud_m, lastInfo] = debris_ud_model(Q2, debris);
    if ~lastInfo.valid
        record = invalid_record(record, lastInfo.warning_flag, 'paper ud model input is invalid');
        record.w_mean = debris.w_mean;
        record.sigma_w = debris.sigma_w;
        record.w_mean_N = debris.w_mean;
        record.sigma_Q_N = debris.sigma_w;
        return;
    end
    activeConfig.debris = debris;
    activeConfig.debris.ud_frozen_m = ud_m;
    activeConfig.debris.ud_frozen_loadi = loadi;
    [record, modified] = run_ball_solver(workspaceRoot, datafromvb, activeConfig, caseInfo);
    relativeChange = relative_contact_change(Q2, loadi, modified.Q2, modified.loadi, datafromvb(1), debris.Q_floor);
    contactPair.modified = modified;
    contactPair.ud_info = lastInfo;
    contactPair.ud_loadi = loadi;
    if relativeChange < debris.outer_tol
        break;
    end
    Q2 = modified.Q2;
    loadi = modified.loadi;
end

record.w_mean = debris.w_mean;
record.sigma_w = debris.sigma_w;
record.w_mean_old = statisticsRecord.w_mean_old;
record.sigma_w_old = statisticsRecord.sigma_w_old;
record.w_mean_N = debris.w_mean;
record.sigma_Q_N = debris.sigma_w;
record.Q_sample_count = statisticsRecord.Q_sample_count;
record.Q_out_of_domain_count = statisticsRecord.Q_out_of_domain_count;
record.Q_min_N = statisticsRecord.Q_min_N;
record.Q_max_N = statisticsRecord.Q_max_N;
record.max_ud_old_m = statisticsRecord.max_ud_old_m;
record.max_ud_new_m = lastInfo.max_ud_m;
record.w_norm_min = min([statisticsRecord.w_norm_min, lastInfo.w_norm_min], [], 'omitnan');
record.w_norm_max = max([statisticsRecord.w_norm_max, lastInfo.w_norm_max], [], 'omitnan');
record.formula_ud_min_um = statisticsRecord.formula_ud_min_um;
record.formula_ud_max_um = statisticsRecord.formula_ud_max_um;
if record.formula_ud_max_um < 0.1
    record.warning_flag = merge_warning(record.warning_flag, 'paper_figure_scale_mismatch_warning');
    record.comment = append_comment(record.comment, ...
        'equation (2-60) with runtime w_mean/sigma_w yields <0.1 um, inconsistent with Fig. 2.34 scale; no scaling applied');
end
record.max_ud_m = lastInfo.max_ud_m;
record.mean_ud_m = lastInfo.mean_ud_m;
record.outer_iterations = outerIter;
record.outer_converged = relativeChange < debris.outer_tol;
record.Fb_original = sum(original.Q2);
record.Fb_modified = sum(contactPair.modified.Q2);
record = fill_contact_change_metrics(record, original, contactPair.modified, debris.Q_floor);
record = merge_ud_warning(record, lastInfo.warning_flag);
if ~record.outer_converged
    record = invalid_record(record, 'outer_nonconvergence', 'outer debris fixed point did not converge');
end
if record.min_oil_film_thickness < baselineRecord.min_oil_film_thickness * 0.5
    record.warning_flag = merge_warning(record.warning_flag, 'oil_film_reduction_warning');
    record.comment = append_comment(record.comment, 'oil film thickness decreased by more than 50 percent');
end
end

function config = case_config(caseInfo)
config = make_micro_interface_config();
config.debris.enabled = caseInfo.enabled;
config.debris.E_matrix = 210e9;
config.debris.E_impurity = caseInfo.E_delta * config.debris.E_matrix;
config.debris.E_delta = caseInfo.E_delta;
config.debris.eta_d = caseInfo.eta_d;
end

function [record, contact] = run_ball_solver(workspaceRoot, datafromvb, config, caseInfo)
bearingDir = fullfile(workspaceRoot, '球轴承程序');
oldDir = pwd;
cleanupObj = onCleanup(@() cd(oldDir)); %#ok<NASGU>
cd(bearingDir);
record = base_record('ball', caseInfo, datafromvb, config);
lastwarn('');
try
    runLog = evalc('[~, returndata] = qiujieend(datafromvb, config);');
    [record.warning_flag, record.comment] = classify_warning(runLog);
    record = fill_metrics(record, 'ball', datafromvb, returndata);
    contact = load_ball_contact_state();
    record.Fb_modified = sum(contact.Q2);
catch ME
    record = error_record('ball', caseInfo, ME);
    contact = empty_contact();
end
record = validate_record(record);
end

function contact = load_ball_contact_state()
loads = load('loadi.mat', 'loadi');
forces = load('q1q2a1a2.mat', 'q1q2a1a2');
state = load('contact_state.mat', 'delta2', 'inner_gap_shift');
loadj = numel(loads.loadi);
contact = struct('loadi', loads.loadi(:), 'Q2', forces.q1q2a1a2(1, loadj + (1:loadj)).', ...
    'delta2', state.delta2(:), 'inner_gap_shift', state.inner_gap_shift(:));
end

function contact = empty_contact()
contact = struct('loadi', [], 'Q2', [], 'delta2', [], 'inner_gap_shift', []);
end

function cageSpeed = estimate_ball_cage_speed(datafromvb)
Dw = datafromvb(2);
Dm = datafromvb(3);
a0 = datafromvb(6) * pi / 180;
innerSpeed = datafromvb(15) * 2 * pi / 60;
cageSpeed = 0.5 * innerSpeed * (1 - Dw / Dm * cos(a0));
end

function contact = solve_baseline_at_cage_phase(workspaceRoot, datafromvb, baselineInfo, phaseRad)
phaseConfig = case_config(baselineInfo);
phaseConfig.debris.cage_phase_rad = phaseRad;
[record, contact] = run_ball_solver(workspaceRoot, datafromvb, phaseConfig, baselineInfo);
if ~record.valid_result || isempty(contact.Q2) || any(~isfinite(contact.Q2))
    error('run_debris_effect_cases:DynamicContactSolveFailed', ...
        'Baseline contact solve failed at cage phase %.16g rad: %s', phaseRad, record.warning_flag);
end
end

function record = base_record(bearingType, caseInfo, datafromvb, config)
record = empty_record();
record.bearing_type = bearingType;
record.group_name = caseInfo.group_name;
record.case_name = caseInfo.case_name;
record.model_scope = 'paper_ud_ball_inner_race_only';
if strcmp(bearingType, 'ball')
    record.speed_rpm = datafromvb(15);
    record.radial_load = hypot(datafromvb(17), datafromvb(18));
else
    record.speed_rpm = datafromvb(18);
    record.radial_load = hypot(datafromvb(19), datafromvb(20));
end
record.temperature = datafromvb(27);
record.lubricant = sprintf('rho=%.6g;eta=%.6g;alpha=%.6g', datafromvb(26), datafromvb(29), datafromvb(30));
record.E_delta = config.debris.E_delta;
record.eta_d = config.debris.eta_d;
record.axial_load = datafromvb(16);
record.valid_result = true;
record.warning_flag = 'none';
record.comment = 'completed';
end

function record = fill_metrics(record, bearingType, datafromvb, returndata)
oilhValues = numeric_values_from_mats({'oilh1.mat', 'oilh2.mat'}, {'oilh1', 'oilh2'});
record.min_oil_film_thickness = min(oilhValues, [], 'omitnan');
record.average_oil_film_thickness = mean(oilhValues, 'omitnan');
record.PV_value = max(numeric_values_from_mats({'pvzhi1.mat', 'pvzhi2.mat', 'pvzhinonload.mat', 'pvzhi1nonload.mat'}, ...
    {'pvzhi1', 'pvzhi2', 'pvzhinonload', 'pvzhi1nonload'}), [], 'omitnan');
sigma = sqrt(sum(datafromvb(23:25) .^ 2));
record.lambda_ratio = record.min_oil_film_thickness / sigma;
record.equivalent_stiffness_x = value_or_nan(returndata, 1);
record.equivalent_stiffness_y = value_or_nan(returndata, 2);
record.cage_slip_ratio = value_or_nan(returndata, 6 - strcmp(bearingType, 'roller'));
end

function values = numeric_values_from_mats(fileNames, variableNames)
values = [];
for i = 1:numel(fileNames)
    if exist(fileNames{i}, 'file') ~= 2
        continue;
    end
    available = {whos('-file', fileNames{i}).name};
    for j = 1:numel(variableNames)
        if ~any(strcmp(available, variableNames{j}))
            continue;
        end
        data = load(fileNames{i}, variableNames{j});
        if isfield(data, variableNames{j}) && isnumeric(data.(variableNames{j}))
            values = [values; data.(variableNames{j})(:)]; %#ok<AGROW>
        end
    end
end
if isempty(values)
    values = NaN;
end
end

function record = validate_record(record)
core = [record.min_oil_film_thickness, record.average_oil_film_thickness, record.PV_value, ...
    record.equivalent_stiffness_x, record.equivalent_stiffness_y, record.cage_slip_ratio];
if any(isnan(core))
    record = invalid_record(record, 'invalid_nan', 'core metric contains NaN');
end
if any(isinf(core))
    record = invalid_record(record, 'invalid_inf', 'core metric contains Inf');
end
if record.min_oil_film_thickness <= 0
    record = invalid_record(record, 'invalid_oilfilm', 'oil film thickness is nonpositive');
end
if ~isfinite(record.lambda_ratio) || record.lambda_ratio < 0.1
    record.warning_flag = merge_warning(record.warning_flag, 'lambda_warning');
    record.comment = append_comment(record.comment, 'lambda ratio is abnormal or unavailable');
end
end

function record = invalid_record(record, warningFlag, comment)
record.valid_result = false;
record.warning_flag = merge_warning(record.warning_flag, warningFlag);
record.comment = append_comment(record.comment, comment);
end

function record = merge_ud_warning(record, warningFlag)
if ~isempty(warningFlag)
    record.warning_flag = merge_warning(record.warning_flag, warningFlag);
end
end

function records = validate_neutral_interface(records)
groupNames = unique({records.group_name}, 'stable');
for i = 1:numel(groupNames)
    groupMask = strcmp({records.group_name}, groupNames{i});
    baselineIndex = find(groupMask & strcmp({records.case_name}, 'baseline'), 1);
    neutralIndex = find(groupMask & strcmp({records.case_name}, 'neutral_interface_test'), 1);
    if isempty(baselineIndex) || isempty(neutralIndex)
        continue;
    end
    baseline = records(baselineIndex); neutral = records(neutralIndex);
    baselineMetrics = [baseline.Fb_modified, baseline.equivalent_stiffness_x, baseline.equivalent_stiffness_y, ...
        baseline.min_oil_film_thickness, baseline.average_oil_film_thickness, baseline.PV_value, baseline.cage_slip_ratio];
    neutralMetrics = [neutral.Fb_modified, neutral.equivalent_stiffness_x, neutral.equivalent_stiffness_y, ...
        neutral.min_oil_film_thickness, neutral.average_oil_film_thickness, neutral.PV_value, neutral.cage_slip_ratio];
    relativeDifference = max(abs(neutralMetrics - baselineMetrics) ./ max(abs(baselineMetrics), 1));
    records(neutralIndex).neutral_relative_difference = relativeDifference;
    if ~isfinite(relativeDifference) || relativeDifference > 1e-12
        records(neutralIndex) = invalid_record(records(neutralIndex), 'neutral_interface_mismatch', ...
            'neutral interface does not reproduce the baseline');
    else
        records(neutralIndex).comment = append_comment(records(neutralIndex).comment, 'neutral interface matches baseline');
    end
end
end

function change = relative_contact_change(oldQ, oldLoadi, newQ, newLoadi, ballCount, Qfloor)
oldFull = contact_vector(oldQ, oldLoadi, ballCount);
newFull = contact_vector(newQ, newLoadi, ballCount);
change = max(abs(newFull - oldFull)) / max(max(abs(oldFull)), Qfloor);
end

function values = contact_vector(Q2, loadi, ballCount)
values = zeros(ballCount, 1);
values(loadi) = Q2;
end

function record = fill_contact_change_metrics(record, original, modified, Qfloor)
ids = unique([original.loadi; modified.loadi]);
deltaOriginal = map_contact(original, ids, 'delta2', NaN);
deltaModified = map_contact(modified, ids, 'delta2', NaN);
QOriginal = map_contact(original, ids, 'Q2', NaN);
QModified = map_contact(modified, ids, 'Q2', NaN);
record.max_gap_change_m = max(abs((-deltaModified) - (-deltaOriginal)), [], 'omitnan');
record.max_delta_change_m = max(abs(deltaModified - deltaOriginal), [], 'omitnan');
record.max_Q_change_N = max(abs(QModified - QOriginal), [], 'omitnan');
record.Fb_change_norm = abs(record.Fb_modified - record.Fb_original) / max(abs(record.Fb_original), Qfloor);
end

function write_contact_diagnostics(summaryDir, contactPair, record, eta_d)
ids = unique([contactPair.original.loadi; contactPair.modified.loadi]);
originalQ = map_contact(contactPair.original, ids, 'Q2', NaN);
modifiedQ = map_contact(contactPair.modified, ids, 'Q2', NaN);
deltaOriginal = map_contact(contactPair.original, ids, 'delta2', NaN);
deltaModified = map_contact(contactPair.modified, ids, 'delta2', NaN);
shift = map_contact(contactPair.modified, ids, 'inner_gap_shift', 0);
wNorm = NaN(size(ids)); ud_um = zeros(size(ids));
if numel(contactPair.ud_info.w_norm) == numel(contactPair.ud_loadi)
    udState = struct('loadi', contactPair.ud_loadi, 'w_norm', contactPair.ud_info.w_norm, 'ud_um', contactPair.ud_info.ud_um);
    wNorm = map_contact(udState, ids, 'w_norm', NaN); ud_um = map_contact(udState, ids, 'ud_um', 0);
end
ud_m = zeros(size(shift));
if eta_d ~= 0
    ud_m = shift / eta_d;
end
gapOriginal = -deltaOriginal;
gapModified = -deltaModified;
gapChange = gapModified - gapOriginal;
deltaChange = deltaModified - deltaOriginal;
QChange = modifiedQ - originalQ;
contactTable = table(ids, originalQ, modifiedQ, wNorm, ud_um, ud_m, gapOriginal, gapModified, gapChange, ...
    deltaOriginal, deltaModified, deltaChange, QChange, ...
    repmat(record.Fb_original, numel(ids), 1), ...
    repmat(record.Fb_modified, numel(ids), 1), 'VariableNames', ...
    {'rolling_element', 'Q_original', 'Q_modified', 'w_norm', 'ud_um', 'ud_m', 'gap_original', 'gap_modified', ...
    'gap_change_m', 'delta_original', 'delta_modified', 'delta_change_m', 'Q_change_N', ...
    'Fb_original', 'Fb_modified'});
writetable(contactTable, fullfile(summaryDir, 'contact_diagnostics.csv'), 'Encoding', 'UTF-8');
save(fullfile(summaryDir, 'contact_diagnostics.mat'), 'contactTable');
end

function write_dynamic_statistics(summaryDir, contactPair)
if ~isstruct(contactPair) || ~isfield(contactPair, 'dynamic_stats')
    return;
end
stats = contactPair.dynamic_stats;
theta = stats.theta; %#ok<NASGU>
Q_samples = stats.Q_samples; %#ok<NASGU>
w_mean = stats.w_mean; %#ok<NASGU>
sigma_Q = stats.sigma_Q; %#ok<NASGU>
Q_sample_count = stats.Q_sample_count; %#ok<NASGU>
Q_out_of_domain_count = stats.Q_out_of_domain_count; %#ok<NASGU>
Q_min = stats.Q_min; %#ok<NASGU>
Q_max = stats.Q_max; %#ok<NASGU>
w_norm_min = stats.w_norm_min; %#ok<NASGU>
w_norm_max = stats.w_norm_max; %#ok<NASGU>
N_theta = stats.N_theta; %#ok<NASGU>
cage_speed_rad_s = stats.cage_speed_rad_s; %#ok<NASGU>
save(fullfile(summaryDir, 'Q_dynamic_samples.mat'), 'theta', 'Q_samples', 'w_mean', 'sigma_Q', ...
    'Q_sample_count', 'Q_out_of_domain_count', 'Q_min', 'Q_max', 'w_norm_min', 'w_norm_max', ...
    'N_theta', 'cage_speed_rad_s');
if isfield(stats, 'formula_curve')
    formula_curve = stats.formula_curve; %#ok<NASGU>
    save(fullfile(summaryDir, 'ud_formula_curve.mat'), 'formula_curve');
    curveTable = table(formula_curve.Q_N, formula_curve.w_norm, formula_curve.ud_um, formula_curve.ud_m, ...
        'VariableNames', {'Q_N', 'w_norm', 'ud_um', 'ud_m'});
    writetable(curveTable, fullfile(summaryDir, 'ud_formula_curve.csv'), 'Encoding', 'UTF-8');
end
end

function values = map_contact(contact, ids, fieldName, fillValue)
values = fillValue * ones(numel(ids), 1);
[matched, index] = ismember(ids, contact.loadi);
if any(matched)
    values(matched) = contact.(fieldName)(index(matched));
end
end

function caseDirs = prepare_case_dirs(resultsRoot, caseName)
caseDirs.root = fullfile(resultsRoot, caseName);
if exist(caseDirs.root, 'dir') == 7
    rmdir(caseDirs.root, 's');
end
caseDirs.mat = fullfile(caseDirs.root, 'mat');
caseDirs.txt = fullfile(caseDirs.root, 'txt');
caseDirs.fig = fullfile(caseDirs.root, 'fig');
caseDirs.summary = fullfile(caseDirs.root, 'summary');
ensure_dir(caseDirs.mat); ensure_dir(caseDirs.txt); ensure_dir(caseDirs.fig); ensure_dir(caseDirs.summary);
end

function copy_project_workspace(projectRoot, workspaceRoot)
if exist(workspaceRoot, 'dir') == 7
    rmdir(workspaceRoot, 's');
end
mkdir(workspaceRoot);
copyfile(fullfile(projectRoot, '*.m'), workspaceRoot);
copyfile(fullfile(projectRoot, '球轴承程序'), fullfile(workspaceRoot, '球轴承程序'));
copyfile(fullfile(projectRoot, '滚子轴承程序'), fullfile(workspaceRoot, '滚子轴承程序'));
end

function add_workspace_paths(workspaceRoot)
addpath(workspaceRoot, '-begin');
addpath(fullfile(workspaceRoot, '球轴承程序'), '-begin');
addpath(fullfile(workspaceRoot, '滚子轴承程序'), '-begin');
end

function remove_workspace_paths(workspaceRoot)
rmpath(fullfile(workspaceRoot, '滚子轴承程序'));
rmpath(fullfile(workspaceRoot, '球轴承程序'));
rmpath(workspaceRoot);
end

function clear_workspace_functions()
clear('qiujieend', 'qiujieall', 'ffSPEED1', 'ffSPEED', 'ffLOAD', 'JffLOAD', ...
    'ff2', 'ff3', 'Jff2', 'make_micro_interface_config', 'load_micro_interface_config', ...
    'debris_ud_model', 'calculate_dynamic_debris_statistics', 'defaultBearingInput', 'bearingOutputPath');
end

function archive_case_outputs(workspaceRoot, bearingType, caseDirs)
if strcmp(bearingType, 'ball')
    bearingDir = fullfile(workspaceRoot, '球轴承程序');
    baseDir = fullfile(workspaceRoot, 'results_bearing_base', 'ball');
else
    bearingDir = fullfile(workspaceRoot, '滚子轴承程序');
    baseDir = fullfile(workspaceRoot, 'results_bearing_base', 'roller');
end
archive_files(bearingDir, caseDirs.mat, [bearingType '_'], {'.mat'});
archive_files(bearingDir, caseDirs.txt, [bearingType '_'], {'.txt'});
archive_files(bearingDir, caseDirs.fig, [bearingType '_'], {'.jpg', '.jpeg', '.png', '.fig'});
if exist(baseDir, 'dir') == 7
    archive_files(baseDir, caseDirs.mat, [bearingType '_base_'], {'.mat'});
    archive_files(baseDir, caseDirs.txt, [bearingType '_base_'], {'.txt'});
    archive_files(baseDir, caseDirs.fig, [bearingType '_base_'], {'.jpg', '.jpeg', '.png', '.fig'});
end
end

function archive_files(srcRoot, destRoot, prefix, extensions)
files = dir(fullfile(srcRoot, '**', '*'));
for i = 1:numel(files)
    if files(i).isdir
        continue;
    end
    [~, ~, ext] = fileparts(files(i).name);
    if ~any(strcmpi(ext, extensions))
        continue;
    end
    relFolder = regexprep(strrep(files(i).folder, srcRoot, ''), ['^' regexptranslate('escape', filesep)], '');
    relFolder = strrep(relFolder, filesep, '_');
    if isempty(relFolder)
        name = [prefix files(i).name];
    else
        name = [prefix relFolder '_' files(i).name];
    end
    copyfile(fullfile(files(i).folder, files(i).name), fullfile(destRoot, name));
end
end

function write_case_note(caseDirs, caseInfo)
fid = fopen(fullfile(caseDirs.summary, 'case_note.txt'), 'w', 'n', 'UTF-8');
fprintf(fid, 'group_name=%s\n', caseInfo.group_name);
fprintf(fid, 'case_name=%s\n', caseInfo.case_name);
fprintf(fid, 'inner_ring_speed_rpm=%.16g\n', caseInfo.speed_rpm);
fprintf(fid, 'axial_load_N=%.16g\n', caseInfo.axial_load_N);
fprintf(fid, 'radial_load_N=%.16g\n', caseInfo.radial_load_N);
fprintf(fid, 'temperature_C=%.16g\n', caseInfo.temperature_C);
fprintf(fid, 'debris.enabled=%d\n', caseInfo.enabled);
fprintf(fid, 'debris.E_delta=%.16g\n', caseInfo.E_delta);
fprintf(fid, 'debris.eta_d=%.16g\n', caseInfo.eta_d);
fprintf(fid, 'Model: Q2 -> ud -> frozen inner-race gap -> delta2 -> Q2.\n');
fclose(fid);
end

function write_plot_csv(resultsRoot, records)
plotSummary = struct2table(records(strcmp({records.bearing_type}, 'ball')));
desktopRoot = fullfile(getenv('HOME'), 'Desktop');
writetable(plotSummary, fullfile(desktopRoot, 'bearing_debris_radial_load_9900rpm_summary.csv'), 'Encoding', 'UTF-8');

plotContacts = table();
for i = 1:numel(records)
    record = records(i);
    if ~strcmp(record.bearing_type, 'ball')
        continue;
    end
    pathName = fullfile(resultsRoot, record.group_name, record.case_name, 'summary', 'contact_diagnostics.csv');
    if exist(pathName, 'file') ~= 2
        continue;
    end
    contactTable = readtable(pathName);
    contactTable.group_name = repmat(string(record.group_name), height(contactTable), 1);
    contactTable.case_name = repmat(string(record.case_name), height(contactTable), 1);
    contactTable.speed_rpm = repmat(record.speed_rpm, height(contactTable), 1);
    contactTable.axial_load_N = repmat(record.axial_load, height(contactTable), 1);
    contactTable.radial_load_N = repmat(record.radial_load, height(contactTable), 1);
    if isempty(plotContacts)
        plotContacts = contactTable;
    else
        plotContacts = [plotContacts; contactTable]; %#ok<AGROW>
    end
end
writetable(plotContacts, fullfile(desktopRoot, 'bearing_debris_radial_load_9900rpm_contacts.csv'), 'Encoding', 'UTF-8');
end

function write_run_note(resultsRoot)
fid = fopen(fullfile(resultsRoot, 'run_note.txt'), 'w', 'n', 'UTF-8');
fprintf(fid, 'Equation (2-60) is evaluated only for ball-inner-race contacts.\n');
fprintf(fid, 'The fixed point is outside the unchanged Newton solver.\n');
fprintf(fid, 'The study varies radial load only at 9900 rpm.\n');
fprintf(fid, 'No time-domain rotor response is claimed by this steady-state runner.\n');
fclose(fid);
end

function write_error_note(summaryDir, ME)
fid = fopen(fullfile(summaryDir, 'case_error.txt'), 'w', 'n', 'UTF-8');
fprintf(fid, '%s\n', getReport(ME, 'extended', 'hyperlinks', 'off'));
fclose(fid);
end

function [warningFlag, comment] = classify_warning(runLog)
[lastMsg, lastId] = lastwarn;
warningFlag = 'none';
comment = 'completed';
if contains(runLog, 'singular', 'IgnoreCase', true) || contains(lastMsg, 'singular', 'IgnoreCase', true)
    warningFlag = 'singular_matrix';
    comment = 'completed with singular matrix warning';
elseif contains(runLog, 'Warning', 'IgnoreCase', true) || strlength(string(lastMsg)) > 0 || strlength(string(lastId)) > 0
    warningFlag = 'warning';
    if strlength(string(lastMsg)) > 0
        comment = char(lastMsg);
    end
end
end

function warningFlag = merge_warning(warningFlag, newFlag)
if isempty(newFlag) || strcmp(newFlag, 'none')
    return;
elseif isempty(warningFlag) || strcmp(warningFlag, 'none')
    warningFlag = newFlag;
elseif ~contains(warningFlag, newFlag)
    warningFlag = [warningFlag ';' newFlag];
end
end

function comment = append_comment(comment, extra)
if isempty(comment)
    comment = extra;
else
    comment = [comment '; ' extra];
end
end

function value = value_or_nan(values, index)
if numel(values) >= index
    value = values(index);
else
    value = NaN;
end
end

function record = error_record(bearingType, caseInfo, ME)
record = empty_record();
record.bearing_type = bearingType;
record.group_name = caseInfo.group_name;
record.case_name = caseInfo.case_name;
record.model_scope = 'paper_ud_ball_inner_race_only';
record.E_delta = caseInfo.E_delta;
record.eta_d = caseInfo.eta_d;
record.warning_flag = 'error';
record.comment = sprintf('%s at %s:%d', ME.message, error_file(ME), error_line(ME));
end

function info = empty_ud_info()
info = struct('warning_flag', '', 'max_ud_m', NaN, 'mean_ud_m', NaN, ...
    'w_norm_min', NaN, 'w_norm_max', NaN, 'w_norm', zeros(0, 1), 'ud_um', zeros(0, 1));
end

function record = empty_record()
record = struct('bearing_type', '', 'group_name', '', 'case_name', '', 'model_scope', '', 'speed_rpm', NaN, ...
    'axial_load', NaN, 'radial_load', NaN, 'temperature', NaN, 'lubricant', '', 'E_delta', NaN, 'eta_d', NaN, ...
    'w_mean', NaN, 'sigma_w', NaN, 'w_mean_old', NaN, 'sigma_w_old', NaN, ...
    'w_mean_N', NaN, 'sigma_Q_N', NaN, 'Q_sample_count', NaN, 'Q_out_of_domain_count', NaN, ...
    'Q_min_N', NaN, 'Q_max_N', NaN, ...
    'w_norm_min', NaN, 'w_norm_max', NaN, 'formula_ud_min_um', NaN, 'formula_ud_max_um', NaN, ...
    'max_ud_old_m', NaN, 'max_ud_new_m', NaN, ...
    'max_ud_m', NaN, 'mean_ud_m', NaN, ...
    'max_gap_change_m', NaN, 'max_delta_change_m', NaN, 'max_Q_change_N', NaN, 'Fb_change_norm', NaN, ...
    'neutral_relative_difference', NaN, ...
    'min_oil_film_thickness', NaN, 'average_oil_film_thickness', NaN, 'lambda_ratio', NaN, ...
    'PV_value', NaN, 'equivalent_stiffness_x', NaN, 'equivalent_stiffness_y', NaN, ...
    'cage_slip_ratio', NaN, 'Fb_original', NaN, 'Fb_modified', NaN, 'outer_iterations', 0, ...
    'outer_converged', false, 'valid_result', false, 'warning_flag', '', 'comment', '');
end

function fileName = error_file(ME)
if isempty(ME.stack), fileName = 'unknown'; else, fileName = ME.stack(1).file; end
end

function lineNo = error_line(ME)
if isempty(ME.stack), lineNo = 0; else, lineNo = ME.stack(1).line; end
end

function ensure_dir(pathName)
if exist(pathName, 'dir') ~= 7, mkdir(pathName); end
end

function datafromvb = default_ball_input()
datafromvb = zeros(75, 1);
datafromvb(1)=15; datafromvb(2)=0.02223; datafromvb(3)=0.1253; datafromvb(4)=0.5232; datafromvb(5)=0.5232; datafromvb(6)=40; datafromvb(7)=0.0115; datafromvb(35)=5;
datafromvb(61)=7800; datafromvb(68)=7800; datafromvb(51)=7800; datafromvb(50)=7800; datafromvb(9)=2.06e+11; datafromvb(10)=2.06e+11; datafromvb(11)=2.06e+11;
datafromvb(65)=2.06e+11; datafromvb(12)=0.3; datafromvb(13)=0.3; datafromvb(14)=0.3; datafromvb(66)=0.3; datafromvb(69)=0; datafromvb(15)=10000; datafromvb(16)=20000; datafromvb(17)=0;
datafromvb(18)=3000; datafromvb(19)=0; datafromvb(20)=0; datafromvb(21)=0.275; datafromvb(22)=0.225; datafromvb(23)=1e-07; datafromvb(24)=1e-07; datafromvb(25)=1e-07; datafromvb(26)=970;
datafromvb(27)=20; datafromvb(28)=0.0966; datafromvb(29)=0.0318; datafromvb(30)=1.28e-008; datafromvb(31)=3.2e-002; datafromvb(32)=1; datafromvb(33)=0.4e-03; datafromvb(34)=3.8;
datafromvb(74)=2; datafromvb(36)=370e-03; datafromvb(37)=220e-03; datafromvb(38)=120.0e-03; datafromvb(39)=225e-03; datafromvb(40)=1.96e11; datafromvb(41)=2.18e11; datafromvb(42)=0.3; datafromvb(43)=0.3;
datafromvb(44)=0.5e-03; datafromvb(45)=-0.5e-03; datafromvb(54)=11.6e-06; datafromvb(55)=11.8e-06; datafromvb(56)=11.8e-06; datafromvb(57)=11.8e-06; datafromvb(58)=11.8e-06; datafromvb(46)=180;
datafromvb(47)=170; datafromvb(48)=190; datafromvb(49)=27; datafromvb(59)=195; datafromvb(60)=160; datafromvb(50)=7870; datafromvb(51)=7870; datafromvb(52)=7860; datafromvb(53)=8360;
datafromvb(70)=0; datafromvb(71)=0.2; datafromvb(72)=0.2; datafromvb(73)=0.2;
end
