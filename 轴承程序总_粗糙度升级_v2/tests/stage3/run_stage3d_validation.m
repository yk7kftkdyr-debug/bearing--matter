function result = run_stage3d_validation(projectRoot,stage3cExternalRoot,mode)
%RUN_STAGE3D_VALIDATION Minimal Stage 3 integration gate.
% Re-runs the approved Stage 3B/3C entries into temporary result files,
% compares only scientific/acceptance state with the protected formal
% results, and writes the single Stage 3D decision report.

if nargin < 1 || isempty(projectRoot)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
if nargin < 2 || isempty(stage3cExternalRoot)
    stage3cExternalRoot = fullfile(tempdir,'stage3c-final-run');
end
if nargin < 3 || isempty(mode)
    mode = 'validate';
end
addpath(projectRoot);
addpath(fullfile(projectRoot,'roughness'));
addpath(fullfile(projectRoot,'tests','stage3'));

reportRoot = fullfile(projectRoot,'reports','stage3');
assert(isfolder(reportRoot),'run_stage3d_validation:Reports', ...
    'Stage 3 report directory is missing.');
paths = result_paths(reportRoot);
if ~strcmp(mode,'resume_after_stage3b')
    delete_temporary(paths);
end

result = initial_result(projectRoot,stage3cExternalRoot);
result.git_head = git_head(projectRoot);
result.stage3b_entry_sha256 = file_sha256(fullfile(projectRoot,'tests','stage3','test_stage3b_ball_feedback.m'));
result.stage3c_entry_sha256 = file_sha256(fullfile(projectRoot,'tests','stage3','test_stage3c_roller_feedback.m'));
result.io_precheck_passed = io_precheck(paths);
if strcmp(mode,'preflight')
    result.status = ternary(result.io_precheck_passed,'STAGE3D_PREFLIGHT_PASS','STAGE3D_HOLD');
    result.decision = ternary(result.io_precheck_passed,'PREFLIGHT_ONLY','HOLD_STAGE3D');
    write_report(paths.report,result);
    return
end
assert(strcmp(mode,'validate') || strcmp(mode,'resume_after_stage3b'), ...
    'run_stage3d_validation:Mode','Unsupported Stage 3D mode.');
assert(result.io_precheck_passed,'run_stage3d_validation:Preflight', ...
    'Stage 3D temporary-result I/O precheck failed.');
try
    [result.stage2c_seed_passed,result.stage2c_seed_reason] = validate_seed(projectRoot);
    [formalB,result.stage3b_formal_passed,result.stage3b_formal_reason] = ...
        load_formal(paths.stage3bFinal,paths.stage3bDecision,'GO_TO_STAGE3C_ROLLER', ...
        {'off_passed','level1_passed','smooth_passed','rough_passed', ...
         'direction_passed','dissipation_passed','normal_state_unchanged'});
    [formalC,result.stage3c_formal_passed,result.stage3c_formal_reason] = ...
        load_formal(paths.stage3cFinal,paths.stage3cDecision,'GO_TO_STAGE3D_VALIDATION', ...
        {'all_passed','normal_snapshot_validation','friction_direction_validation', ...
         'friction_dissipation_validation','gamma_mu_one_completed', ...
         'ffLOAD_called_in_level2','level1_qiujieall_call_count', ...
         'level2_qiujieall_call_count','speed_recompute_validation'});
    [stage3a,result.stage3a_passed,result.stage3a_reason] = validate_stage3a(paths.stage3aFinal,paths.stage3aDecision);

    if result.stage3b_formal_passed && strcmp(mode,'validate')
        currentB = test_stage3b_ball_feedback(projectRoot,paths.stage3bTemporary);
        [temporaryB,temporaryBValid,temporaryBReason] = load_temporary( ...
            paths.stage3bTemporary,paths.stage3bTemporaryDecision,'GO_TO_STAGE3C_ROLLER');
        result.stage3b_recheck_passed = temporaryBValid && currentB.off_passed && ...
            currentB.level1_passed && currentB.smooth_passed && currentB.rough_passed && ...
            currentB.direction_passed && currentB.dissipation_passed && ...
            currentB.normal_state_unchanged;
        result.stage3b_recheck_reason = temporaryBReason;
        [result.stage3b_science_consistent,result.stage3b_difference,result.stage3b_max_abs_error,result.stage3b_max_rel_error] = ...
            compare_scientific(formalB,temporaryB,1e-10);
        result.stage3b_evidence_recorded = true;
        result.status = 'STAGE3D_IN_PROGRESS';
        result.decision = 'PENDING_STAGE3C';
        write_report(paths.report,result);
        delete_stage3b_temporary(paths);
        assert(result.stage3b_recheck_passed && result.stage3b_science_consistent, ...
            'run_stage3d_validation:Stage3B','Stage 3B temporary recheck or scientific comparison failed.');
    end

    if result.stage3b_formal_passed && strcmp(mode,'resume_after_stage3b')
        [temporaryB,temporaryBValid,temporaryBReason] = load_temporary( ...
            paths.stage3bTemporary,paths.stage3bTemporaryDecision,'GO_TO_STAGE3C_ROLLER');
        result.stage3b_recheck_passed = temporaryBValid && temporaryB.off_passed && ...
            temporaryB.level1_passed && temporaryB.smooth_passed && temporaryB.rough_passed && ...
            temporaryB.direction_passed && temporaryB.dissipation_passed && ...
            temporaryB.normal_state_unchanged;
        result.stage3b_recheck_reason = temporaryBReason;
        [result.stage3b_science_consistent,result.stage3b_difference,result.stage3b_max_abs_error,result.stage3b_max_rel_error] = ...
            compare_scientific(formalB,temporaryB,1e-10);
        result.stage3b_evidence_recorded = true;
        result.status = 'STAGE3D_IN_PROGRESS';
        result.decision = 'PENDING_STAGE3C';
        write_report(paths.report,result);
        assert(result.stage3b_recheck_passed && result.stage3b_science_consistent, ...
            'run_stage3d_validation:Stage3B','Stage 3B temporary scientific comparison failed.');
        delete_stage3b_temporary(paths);
    end

    if result.stage3c_formal_passed
        currentC = test_stage3c_roller_feedback(projectRoot,stage3cExternalRoot, ...
            paths.stage3cTemporary);
        [temporaryC,temporaryCValid,temporaryCReason] = load_temporary( ...
            paths.stage3cTemporary,paths.stage3cTemporaryDecision,'GO_TO_STAGE3D_VALIDATION');
        result.stage3c_recheck_passed = temporaryCValid && currentC.all_passed;
        result.stage3c_recheck_reason = temporaryCReason;
        [result.stage3c_science_consistent,result.stage3c_difference,result.stage3c_max_abs_error,result.stage3c_max_rel_error] = ...
            compare_scientific(formalC,temporaryC,1e-10);
        result.stage3c_evidence_recorded = true;
        write_report(paths.report,result);
        delete_stage3c_temporary(paths);
        assert(result.stage3c_recheck_passed && result.stage3c_science_consistent, ...
            'run_stage3d_validation:Stage3C','Stage 3C temporary recheck or scientific comparison failed.');
    end

    result.off_regression_passed = result.stage3b_recheck_passed && ...
        result.stage3c_recheck_passed && formalB.off_passed && ...
        formalC.off_validation.passed;
    result.normal_frozen_passed = result.stage3b_recheck_passed && ...
        result.stage3c_recheck_passed && formalB.normal_state_unchanged && ...
        formalC.normal_snapshot_validation && ...
        formalC.Q1_difference == 0 && formalC.Q2_difference == 0 && ...
        formalC.oilh1_difference == 0 && formalC.oilh2_difference == 0 && ...
        formalC.slice_load_difference == 0 && ...
        ~formalC.ffLOAD_called_in_level2;
    result.traction_physics_passed = result.stage3b_recheck_passed && ...
        result.stage3c_recheck_passed && formalB.direction_passed && ...
        formalB.dissipation_passed && formalC.friction_direction_validation && ...
        formalC.friction_dissipation_validation;
    result.speed_isolation_passed = result.stage3c_recheck_passed && ...
        formalC.level1_qiujieall_call_count == 1 && ...
        formalC.level2_qiujieall_call_count == 0 && ...
        formalC.speed_recompute_validation.passed && ...
        ~formalC.ffLOAD_called_in_level2;
    result.finite_real_passed = finite_physical(stage3a) && finite_physical(formalB) && ...
        finite_physical(formalC) && finite_physical(temporaryB) && finite_physical(temporaryC);
    result.no_stage4_pollution = isempty(stage4_files(projectRoot));
    result.deterministic_passed = result.stage3b_science_consistent && ...
        result.stage3c_science_consistent;
catch exception
    result.failure_reason = getReport(exception,'basic','hyperlinks','off');
end

result.all_passed = result.stage2c_seed_passed && result.stage3a_passed && ...
    result.stage3b_formal_passed && result.stage3c_formal_passed && ...
    result.stage3b_recheck_passed && result.stage3c_recheck_passed && ...
    result.stage3b_science_consistent && result.stage3c_science_consistent && ...
    result.off_regression_passed && result.normal_frozen_passed && ...
    result.traction_physics_passed && result.speed_isolation_passed && ...
    result.finite_real_passed && result.no_stage4_pollution;
if result.all_passed
    result.status = 'STAGE3D_PASS';
    result.decision = 'GO_TO_STAGE4_LOCAL_CONTACT';
else
    result.status = 'STAGE3D_HOLD';
    result.decision = 'HOLD_STAGE3D';
end
write_report(paths.report,result);
end

function paths = result_paths(reportRoot)
paths = struct( ...
    'stage3aFinal',fullfile(reportRoot,'stage3a_result.mat'), ...
    'stage3aDecision',fullfile(reportRoot,'stage3a_decision.txt'), ...
    'stage3bFinal',fullfile(reportRoot,'stage3b_ball_result.mat'), ...
    'stage3bDecision',fullfile(reportRoot,'stage3b_decision.txt'), ...
    'stage3cFinal',fullfile(reportRoot,'stage3c_roller_result.mat'), ...
    'stage3cDecision',fullfile(reportRoot,'stage3c_decision.txt'), ...
    'stage3bTemporary',fullfile(reportRoot,'stage3b_ball_result.stage3d_tmp.mat'), ...
    'stage3bTemporaryDecision',fullfile(reportRoot,'stage3b_decision.stage3d_tmp.txt'), ...
    'stage3cTemporary',fullfile(reportRoot,'stage3c_roller_result.stage3d_tmp.mat'), ...
    'stage3cTemporaryDecision',fullfile(reportRoot,'stage3c_decision.stage3d_tmp.txt'), ...
    'report',fullfile(reportRoot,'stage3d_total_validation.txt'));
end

function result = initial_result(projectRoot,externalRoot)
result = struct('project_root',projectRoot,'stage3c_external_root',externalRoot, ...
    'stage2c_seed_passed',false,'stage2c_seed_reason','','stage3a_passed',false, ...
    'stage3a_reason','','stage3b_formal_passed',false,'stage3b_formal_reason','', ...
    'stage3c_formal_passed',false,'stage3c_formal_reason','', ...
    'stage3b_recheck_passed',false,'stage3b_recheck_reason','not run', ...
    'stage3c_recheck_passed',false,'stage3c_recheck_reason','not run', ...
    'stage3b_science_consistent',false,'stage3b_difference','not compared', ...
    'stage3c_science_consistent',false,'stage3c_difference','not compared', ...
    'off_regression_passed',false,'normal_frozen_passed',false, ...
    'traction_physics_passed',false,'speed_isolation_passed',false, ...
    'finite_real_passed',false,'no_stage4_pollution',false, ...
    'deterministic_passed',false,'failure_reason','');
result.io_precheck_passed = false;
result.git_head = '';
result.stage3b_entry_sha256 = '';
result.stage3c_entry_sha256 = '';
result.stage3b_max_abs_error = NaN;
result.stage3b_max_rel_error = NaN;
result.stage3c_max_abs_error = NaN;
result.stage3c_max_rel_error = NaN;
result.stage3b_evidence_recorded = false;
result.stage3c_evidence_recorded = false;
end

function [passed,reason] = validate_seed(projectRoot)
seed = fullfile(tempdir,'stage3-recovery-seed','roller_rough_feedback_validation.mat');
if ~isfile(seed)
    passed = false; reason = 'Stage 2C gamma=1 seed is missing.'; return
end
data = load(seed);
passed = finite_physical(data) && isfile(fullfile(projectRoot,'reports','stage2','stage2c_result.mat'));
if passed
    reason = '';
else
    reason = 'Stage 2C gamma=1 seed is unreadable or contains nonphysical values.';
end
end

function [result,passed,reason] = validate_stage3a(resultFile,decisionFile)
[result,passed,reason] = load_formal(resultFile,decisionFile,'GO_TO_STAGE3B_BALL', ...
    {'smooth_passed','ball_passed','roller_passed','dissipation_passed', ...
     'normal_state_unchanged','all_passed'});
end

function [result,passed,reason] = load_formal(resultFile,decisionFile,decision,required)
result = struct(); passed = false; reason = '';
if ~isfile(resultFile) || ~isfile(decisionFile)
    reason = sprintf('Missing formal result or decision: %s',resultFile); return
end
loaded = load(resultFile,'result');
if ~isfield(loaded,'result') || ~isstruct(loaded.result)
    reason = sprintf('Unreadable formal result: %s',resultFile); return
end
result = loaded.result;
if ~strcmp(strtrim(fileread(decisionFile)),decision)
    reason = sprintf('Formal decision mismatch: %s',decisionFile); return
end
if ~all(isfield(result,required)) || ~finite_physical(result)
    reason = sprintf('Required field or physical-value check failed: %s',resultFile); return
end
passFields = required(endsWith(required,'passed'));
passValues = false(size(passFields));
for index = 1:numel(passFields)
    passValues(index) = logical(result.(passFields{index}));
end
passed = all(passValues);
if ~passed, reason = 'Formal result contains a failed acceptance flag.'; end
end

function [result,passed,reason] = load_temporary(resultFile,decisionFile,decision)
result = struct(); passed = false; reason = '';
if ~isfile(resultFile) || ~isfile(decisionFile)
    reason = sprintf('Temporary review output was not created: %s',resultFile); return
end
loaded = load(resultFile,'result');
if ~isfield(loaded,'result') || ~isstruct(loaded.result)
    reason = sprintf('Temporary review result is unreadable: %s',resultFile); return
end
result = loaded.result;
if ~strcmp(strtrim(fileread(decisionFile)),decision)
    reason = sprintf('Temporary decision mismatch: %s',decisionFile); return
end
passed = true;
end

function [equal,firstDifference,maxAbsolute,maxRelative] = compare_scientific(left,right,tolerance)
[equal,firstDifference,maxAbsolute,maxRelative] = compare_value(left,right,'result',tolerance);
end

function [equal,firstDifference,maxAbsolute,maxRelative] = compare_value(left,right,pathName,tolerance)
if ignore_field(pathName)
    equal = true; firstDifference = ''; maxAbsolute = 0; maxRelative = 0; return
end
if isnumeric(left) || islogical(left)
    if ~(isnumeric(right) || islogical(right)) || ~isequal(size(left),size(right))
        equal = false; firstDifference = [pathName ' type or shape']; maxAbsolute = Inf; maxRelative = Inf; return
    end
    if ~isreal(left) || ~isreal(right) || any(isnan(left(:))) || any(isnan(right(:)))
        equal = false; firstDifference = [pathName ' nonreal or NaN']; maxAbsolute = Inf; maxRelative = Inf; return
    end
    sameInfinity = isequal(isinf(left),isinf(right)) && all(left(isinf(left)) == right(isinf(right)));
    finite = isfinite(left) & isfinite(right);
    if any(isinf(left(:)) | isinf(right(:))) && ~sameInfinity
        equal = false; firstDifference = [pathName ' Inf pattern']; maxAbsolute = Inf; maxRelative = Inf; return
    end
    if any(finite(:))
        differences = abs(double(left(finite))-double(right(finite)));
        maxAbsolute = max(differences);
        errorValue = max(differences ./ ...
            max(abs(double(left(finite))),1));
    else
        maxAbsolute = 0; errorValue = 0;
    end
    maxRelative = errorValue;
    equal = errorValue <= tolerance;
    if equal, firstDifference = ''; else, firstDifference = sprintf('%s relative error %.17g',pathName,errorValue); end
    return
end
if ischar(left) || isstring(left)
    equal = isequal(left,right);
    maxAbsolute = 0; maxRelative = 0;
    if equal, firstDifference = ''; else, firstDifference = [pathName ' text']; maxAbsolute = Inf; maxRelative = Inf; end
    return
end
if isstruct(left)
    if ~isstruct(right) || ~isequal(size(left),size(right)) || ~isequal(sort(fieldnames(left)),sort(fieldnames(right)))
        equal = false; firstDifference = [pathName ' structure']; maxAbsolute = Inf; maxRelative = Inf; return
    end
    names = sort(fieldnames(left));
    maxAbsolute = 0; maxRelative = 0;
    for item = 1:numel(left)
        for index = 1:numel(names)
            [equal,firstDifference,childAbsolute,childRelative] = compare_value(left(item).(names{index}), ...
                right(item).(names{index}),[pathName '.' names{index}],tolerance);
            maxAbsolute = max(maxAbsolute,childAbsolute);
            maxRelative = max(maxRelative,childRelative);
            if ~equal, return; end
        end
    end
    equal = true; firstDifference = ''; return
end
if iscell(left)
    if ~iscell(right) || ~isequal(size(left),size(right))
        equal = false; firstDifference = [pathName ' cell']; maxAbsolute = Inf; maxRelative = Inf; return
    end
    maxAbsolute = 0; maxRelative = 0;
    for index = 1:numel(left)
        [equal,firstDifference,childAbsolute,childRelative] = compare_value(left{index},right{index}, ...
            sprintf('%s{%d}',pathName,index),tolerance);
        maxAbsolute = max(maxAbsolute,childAbsolute);
        maxRelative = max(maxRelative,childRelative);
        if ~equal, return; end
    end
    equal = true;
    firstDifference = '';
    return
end
equal = isequaln(left,right);
maxAbsolute = 0; maxRelative = 0;
if equal, firstDifference = ''; else, firstDifference = [pathName ' value']; maxAbsolute = Inf; maxRelative = Inf; end
end

function ignore = ignore_field(pathName)
ignore = ~isempty(regexp(pathName, ...
    '\.(raw_result_file|output_dir|input_file|file_a|file_b|project_root|stage3c_external_root|message|failure_reason|runtime|result333|legacy_metric|gamma_final|gamma_mu|gamma_mu_final|speed_residual)$','once'));
end

function passed = finite_physical(value)
[passed,~] = finite_value(value,'result');
end

function [passed,reason] = finite_value(value,pathName)
if diagnostic_field(pathName)
    passed = true; reason = ''; return
end
if isnumeric(value) || islogical(value)
    if ~isreal(value) || any(isnan(value(:)))
        passed = false; reason = [pathName ' has NaN or complex value']; return
    end
    if any(isinf(value(:))) && isempty(regexp(pathName,'\.lambda$','once'))
        passed = false; reason = [pathName ' has non-lambda Inf']; return
    end
    passed = true; reason = ''; return
end
if isstruct(value)
    names = fieldnames(value);
    for item = 1:numel(value)
        for index = 1:numel(names)
            [passed,reason] = finite_value(value(item).(names{index}),[pathName '.' names{index}]);
            if ~passed, return; end
        end
    end
    passed = true; reason = ''; return
end
if iscell(value)
    for index = 1:numel(value)
        [passed,reason] = finite_value(value{index},sprintf('%s{%d}',pathName,index));
        if ~passed, return; end
    end
end
passed = true; reason = '';
end

function diagnostic = diagnostic_field(pathName)
diagnostic = ~isempty(regexp(pathName, ...
    '\.(result333|legacy_metric|gamma_final|gamma_mu|gamma_mu_final|speed_residual)$','once'));
end

function files = stage4_files(projectRoot)
entries = dir(fullfile(projectRoot,'**','*stage4*'));
files = {entries(~[entries.isdir]).name};
end

function passed = io_precheck(paths)
targets = {paths.stage3bTemporary,paths.stage3cTemporary};
passed = all(cellfun(@(item)endsWith(item,'.mat') && ...
    isempty(strfind(item,'.mat.mat')),targets)); %#ok<STREMP>
passed = passed && ~strcmp(paths.stage3bTemporary,paths.stage3bFinal) && ...
    ~strcmp(paths.stage3cTemporary,paths.stage3cFinal) && ...
    ~strcmp(paths.stage3cTemporaryDecision,paths.stage3cDecision);
if ~passed, return; end
probe = struct('fixed_scalar',1);
for index = 1:numel(targets)
    target = targets{index};
    [folder,base,extension] = fileparts(target);
    atomic = fullfile(folder,[base '.io_precheck' extension]);
    if isfile(atomic), delete(atomic); end
    save(atomic,'probe','-v7');
    loaded = load(atomic,'probe');
    passed = passed && isfield(loaded,'probe') && isequal(loaded.probe,probe);
    delete(atomic);
    if ~passed, return; end
end
end

function value = git_head(projectRoot)
[status,value] = system(sprintf('git -C "%s" rev-parse HEAD',projectRoot));
assert(status == 0,'run_stage3d_validation:Git','Cannot read current Git HEAD.');
value = strtrim(value);
end

function value = file_sha256(pathName)
[status,value] = system(sprintf('shasum -a 256 "%s"',pathName));
assert(status == 0,'run_stage3d_validation:Fingerprint','Cannot fingerprint Stage 3 entry.');
value = char(extractBefore(strtrim(value),' '));
end

function value = ternary(condition,trueValue,falseValue)
if condition, value = trueValue; else, value = falseValue; end
end

function write_report(pathName,result)
temporary = [pathName '.write_tmp'];
fileId = fopen(temporary,'w');
assert(fileId >= 0,'run_stage3d_validation:Report','Cannot write Stage 3D report.');
cleanup = onCleanup(@()fclose(fileId)); %#ok<NASGU>
fprintf(fileId,'%s\n%s\n',result.status,result.decision);
fprintf(fileId,'git_head=%s\nstage3b_entry_sha256=%s\nstage3c_entry_sha256=%s\n', ...
    result.git_head,result.stage3b_entry_sha256,result.stage3c_entry_sha256);
fprintf(fileId,'io_precheck=%d\nstage3b_evidence_recorded=%d\nstage3c_evidence_recorded=%d\n', ...
    result.io_precheck_passed,result.stage3b_evidence_recorded,result.stage3c_evidence_recorded);
fprintf(fileId,'stage2c_seed=%d\nstage3a=%d\nstage3b_recheck=%d\nstage3c_recheck=%d\n', ...
    result.stage2c_seed_passed,result.stage3a_passed,result.stage3b_recheck_passed,result.stage3c_recheck_passed);
fprintf(fileId,'stage3b_consistent=%d\nstage3c_consistent=%d\noff_regression=%d\n', ...
    result.stage3b_science_consistent,result.stage3c_science_consistent,result.off_regression_passed);
fprintf(fileId,'normal_frozen=%d\ntraction_physics=%d\nspeed_isolation=%d\n', ...
    result.normal_frozen_passed,result.traction_physics_passed,result.speed_isolation_passed);
fprintf(fileId,'finite_real=%d\ndeterministic=%d\nno_stage4_pollution=%d\n', ...
    result.finite_real_passed,result.deterministic_passed,result.no_stage4_pollution);
fprintf(fileId,'stage3b_max_absolute_error=%.17g\nstage3b_max_relative_error=%.17g\n', ...
    result.stage3b_max_abs_error,result.stage3b_max_rel_error);
fprintf(fileId,'stage3c_max_absolute_error=%.17g\nstage3c_max_relative_error=%.17g\n', ...
    result.stage3c_max_abs_error,result.stage3c_max_rel_error);
fprintf(fileId,'stage3b_difference=%s\nstage3c_difference=%s\n', ...
    result.stage3b_difference,result.stage3c_difference);
if ~isempty(result.failure_reason), fprintf(fileId,'failure_reason=%s\n',result.failure_reason); end
clear cleanup
movefile(temporary,pathName,'f');
end

function delete_temporary(paths)
for pathName = {paths.stage3bTemporary,paths.stage3bTemporaryDecision, ...
        paths.stage3cTemporary,paths.stage3cTemporaryDecision, ...
        atomic_mat_path(paths.stage3bTemporary),atomic_mat_path(paths.stage3cTemporary)}
    if isfile(pathName{1}), delete(pathName{1}); end
end
end

function delete_stage3b_temporary(paths)
for pathName = {paths.stage3bTemporary,paths.stage3bTemporaryDecision,atomic_mat_path(paths.stage3bTemporary)}
    if isfile(pathName{1}), delete(pathName{1}); end
end
end

function delete_stage3c_temporary(paths)
for pathName = {paths.stage3cTemporary,paths.stage3cTemporaryDecision,atomic_mat_path(paths.stage3cTemporary)}
    if isfile(pathName{1}), delete(pathName{1}); end
end
end

function pathName = atomic_mat_path(final)
[folder,base,extension] = fileparts(final);
pathName = fullfile(folder,[base '.write_tmp' extension]);
end
