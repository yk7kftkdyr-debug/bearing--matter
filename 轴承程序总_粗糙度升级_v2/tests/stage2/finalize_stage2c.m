function summary = finalize_stage2c(projectRoot, externalRoot, outputRoot)
%FINALIZE_STAGE2C Aggregate existing Stage 2 evidence without running models.
% This function deliberately performs only MAT/JSON reads and atomic report writes.

if nargin < 1 || isempty(projectRoot)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
if nargin < 2 || isempty(externalRoot)
    externalRoot = '/private/tmp/stage2c_regression';
end
if nargin < 3 || isempty(outputRoot)
    outputRoot = fullfile(projectRoot,'reports','stage2');
end
assert(isfolder(projectRoot),'finalize_stage2c:ProjectRoot','Project root is missing.');
assert(isfolder(externalRoot),'finalize_stage2c:ExternalRoot','External result root is missing.');
if ~isfolder(outputRoot), mkdir(outputRoot); end

ballOff = read_ball_off(externalRoot);
ballFeedback = read_ball_feedback(externalRoot);
roller = read_roller_regression(externalRoot);
rough = read_roller_rough(externalRoot);
order = read_order_check(projectRoot);

summary = struct();
summary.ball_off_passed = ballOff.passed;
summary.ball_feedback_passed = ballFeedback.passed;
summary.roller_off_passed = roller.off_passed;
summary.roller_smooth_passed = roller.smooth_passed;
summary.roller_feedback_passed = rough.passed;
summary.ball_load_partition_passed = ballFeedback.load_partition_passed;
summary.roller_load_partition_passed = rough.load_partition_passed;
summary.ball_gamma_one_completed = ballFeedback.gamma_one_completed;
summary.roller_gamma_one_completed = rough.gamma_one_completed;
summary.roughness_texture_independent = ballOff.texture_independent && ...
    ballFeedback.texture_independent && rough.texture_independent;
summary.T1_T2_unchanged = roller.T1_T2_unchanged;
summary.physical_finite_real = ballOff.physical_finite_real && ...
    ballFeedback.physical_finite_real && roller.physical_finite_real && ...
    rough.physical_finite_real;
summary.execution_order_independent = order.passed;
summary.no_friction_feedback = ballFeedback.no_friction_feedback && ...
    roller.no_friction_feedback && rough.no_friction_feedback;
summary.failure_reasons = join_reasons({ballOff.reason,ballFeedback.reason, ...
    roller.reason,rough.reason,order.reason});
summary.stage2_total_passed = summary.ball_off_passed && ...
    summary.ball_feedback_passed && summary.roller_off_passed && ...
    summary.roller_smooth_passed && summary.roller_feedback_passed && ...
    summary.ball_load_partition_passed && summary.roller_load_partition_passed && ...
    summary.ball_gamma_one_completed && summary.roller_gamma_one_completed && ...
    summary.roughness_texture_independent && summary.T1_T2_unchanged && ...
    summary.physical_finite_real && summary.execution_order_independent && ...
    summary.no_friction_feedback;
if summary.stage2_total_passed
    summary.decision = 'GO_TO_STAGE3_FRICTION';
else
    summary.decision = 'HOLD_STAGE2C';
end

write_atomic_validation(outputRoot,summary);
write_stage2c_result(outputRoot,summary,ballOff,ballFeedback,roller,rough,order);
write_decision(outputRoot,summary.decision);
end

function evidence = read_ball_off(externalRoot)
fileName = fullfile(externalRoot,'ball_off_validation.mat');
assert(isfile(fileName),'finalize_stage2c:BallOff','Missing ball OFF validation MAT.');
data = load(fileName);
assert(isfield(data,'report') && isfield(data,'cmp') && isfield(data,'textureBefore') && isfield(data,'textureAfter'), ...
    'finalize_stage2c:BallOff','Ball OFF validation MAT is incomplete.');
within = strcmp(data.cmp.status,'NUMERICALLY_IDENTICAL') || ...
    (strcmp(data.cmp.status,'WITHIN_TOLERANCE') && data.cmp.max_relative_error <= 1e-10);
texture = isequaln(data.textureBefore,data.textureAfter);
% result333 is a legacy diagnostic field and may legitimately contain NaN.
% Inspect only the returned physical state for this acceptance item.
physical = [data.state.contact_load_inner(:); data.state.contact_load_outer(:); ...
    data.state.oil_film_inner(:); data.state.oil_film_outer(:); ...
    data.state.working_clearance(:); data.state.stiffness(:); data.state.returndata(:)];
finite = data.report.success && all(isfinite(physical)) && isreal(physical);
evidence = struct('passed',data.report.success && data.cmp.readable && ...
    data.cmp.loaded_element_count_equal && within && texture && finite, ...
    'texture_independent',texture,'physical_finite_real',finite, ...
    'reason',reason_if(~(data.report.success && within && texture && finite), ...
    'ball OFF validation did not meet the saved Stage 0 comparison criteria.'));
end

function evidence = read_ball_feedback(externalRoot)
fileName = fullfile(externalRoot,'ball_feedback_validation.mat');
assert(isfile(fileName),'finalize_stage2c:BallFeedback','Missing ball feedback validation MAT.');
data = load(fileName);
assert(isfield(data,'state') && isfield(data,'report') && isfield(data,'rough') && ...
    isfield(data,'textureBefore') && isfield(data,'textureAfter'), ...
    'finalize_stage2c:BallFeedback','Ball feedback validation MAT is incomplete.');
assert(isfield(data.state,'roughness_feedback'),'finalize_stage2c:BallFeedback','Missing ball roughness feedback state.');
feedback = data.state.roughness_feedback;
outer = feedback.outer;
inner = feedback.inner;
assert(~isempty(outer.Qtotal) && ~isempty(inner.Qtotal), ...
    'finalize_stage2c:BallFeedback','No ball roughness contacts were saved.');
qTotal = [outer.Qtotal(:); inner.Qtotal(:)];
qFluid = [outer.Qfluid(:); inner.Qfluid(:)];
qAsp = [outer.Qasperity(:); inner.Qasperity(:)];
hMix = [outer.hMix(:); inner.hMix(:)];
chiA = [outer.chiA(:); inner.chiA(:)];
lambda = [outer.lambda(:); inner.lambda(:)];
partition = all(abs(qFluid + qAsp - qTotal) ./ max(abs(qTotal),1) < 1e-6) && ...
    all(qFluid >= 0) && all(qAsp >= 0) && all(chiA >= 0 & chiA <= 1) && all(hMix > 0);
physical = [qTotal; qFluid; qAsp; hMix; chiA; lambda];
finite = all(isfinite(physical)) && isreal(physical);
gammaOne = data.report.success && data.report.gamma_final == 1 && feedback.gamma_final == 1 && ...
    data.report.outer_converged && data.report.closure_pass && data.report.load_share_pass;
texture = isequaln(data.textureBefore,data.textureAfter);
noFriction = data.rough.feedback_level == 1 && ~isfield(data.rough,'friction_feedback');
passed = gammaOne && partition && finite && texture && noFriction;
evidence = struct('passed',passed,'load_partition_passed',partition, ...
    'gamma_one_completed',gammaOne,'texture_independent',texture, ...
    'physical_finite_real',finite,'no_friction_feedback',noFriction, ...
    'reason',reason_if(~passed,'ball feedback validation did not meet the saved criteria.'));
end

function evidence = read_roller_regression(externalRoot)
fileName = fullfile(externalRoot,'roller_regression_status.json');
assert(isfile(fileName),'finalize_stage2c:RollerRegression','Missing roller regression JSON.');
status = jsondecode(fileread(fileName));
required = {'off_passed','smooth_passed','T1_T2_unchanged','physical_finite_real', ...
    'Q1_error','Q2_error','oilh1_error','oilh2_error','hash_match', ...
    'loadj_match','loadi_match','loadii_match','smooth_load_share','smooth_gt_executed'};
assert(all(isfield(status,required)),'finalize_stage2c:RollerRegression', ...
    'Roller regression JSON is incomplete.');
smooth = status.smooth_passed && status.Q1_error <= 1e-10 && status.Q2_error <= 1e-10 && ...
    status.oilh1_error <= 1e-10 && status.oilh2_error <= 1e-10 && ...
    status.hash_match && status.loadj_match && status.loadi_match && status.loadii_match && ...
    status.smooth_load_share && status.smooth_gt_executed;
evidence = struct('off_passed',logical(status.off_passed),'smooth_passed',smooth, ...
    'T1_T2_unchanged',logical(status.T1_T2_unchanged), ...
    'physical_finite_real',logical(status.physical_finite_real), ...
    'texture_independent',false,'no_friction_feedback',true, ...
    'reason',reason_if(~(status.off_passed && smooth), ...
    'roller OFF or smooth-feedback regression did not meet saved criteria.'));
end

function evidence = read_roller_rough(externalRoot)
% A gamma=1 run is valid only when an actual persisted MAT/JSON result exists.
candidates = {fullfile(externalRoot,'roller_rough_feedback_validation.mat'), ...
    fullfile(externalRoot,'roller_gamma1_validation.mat')};
index = find(cellfun(@isfile,candidates),1,'first');
if isempty(index)
    evidence = struct('passed',false,'load_partition_passed',false, ...
        'gamma_one_completed',false,'physical_finite_real',false, ...
        'texture_independent',false,'no_friction_feedback',false, ...
        'reason','roller rough-feedback gamma=1 formal MAT/JSON result is missing.');
    return;
end
fileName = candidates{index};
data = load(fileName);
if isfield(data,'step')
    step = data.step;
elseif isfield(data,'state')
    step = data.state;
else
    error('finalize_stage2c:RollerRough','Roller rough result lacks step data.');
end
assert(isfield(step,'config_audit') && isfield(step.config_audit,'texture_before') && ...
    isfield(step.config_audit,'texture_after'), ...
    'finalize_stage2c:RollerRough','Roller rough result lacks a texture independence audit.');
contacts = [step.outer_contacts(:); step.inner_contacts(:)];
assert(~isempty(contacts),'finalize_stage2c:RollerRough','Roller rough result has no contacts.');
qTotal = [contacts.Qtotal]; qFluid = [contacts.Qfluid]; qAsp = [contacts.Qasperity];
lambda = [contacts.lambda];
partition = all(abs(qFluid + qAsp - qTotal) ./ max(abs(qTotal),1) < 1e-6);
physical = [qTotal(:); qFluid(:); qAsp(:); lambda(:)];
finite = logical(step.physical_finite_real) && all(isfinite(physical)) && isreal(physical);
gammaOne = step.gamma == 1 && step.gt_converged && step.mechanical_converged;
texture = isequaln(step.config_audit.texture_before,step.config_audit.texture_after);
passed = logical(step.success) && partition && finite && gammaOne && texture;
evidence = struct('passed',passed,'load_partition_passed',partition, ...
    'gamma_one_completed',gammaOne,'physical_finite_real',finite, ...
    'texture_independent',texture,'no_friction_feedback',true, ...
    'reason',reason_if(~passed,'roller rough-feedback gamma=1 result failed validation.'));
end

function evidence = read_order_check(projectRoot)
fileName = fullfile(projectRoot,'reports','stage0_5','wrapper_test_result.csv');
assert(isfile(fileName),'finalize_stage2c:Order','Missing wrapper order-independence CSV.');
tableData = readtable(fileName);
assert(ismember('pass',tableData.Properties.VariableNames) && height(tableData) >= 4, ...
    'finalize_stage2c:Order','Wrapper order-independence CSV is incomplete.');
passed = all(logical(tableData.pass));
evidence = struct('passed',passed,'reason',reason_if(~passed, ...
    'saved wrapper order-independence validation failed.'));
end

function write_atomic_validation(outputRoot,summary)
tmp = fullfile(outputRoot,'stage2_total_validation.tmp.mat');
final = fullfile(outputRoot,'stage2_total_validation.mat');
if isfile(tmp), delete(tmp); end
save(tmp,'-struct','summary','-v7');
check = load(tmp);
required = {'ball_off_passed','ball_feedback_passed','roller_off_passed', ...
    'roller_smooth_passed','roller_feedback_passed','ball_load_partition_passed', ...
    'roller_load_partition_passed','ball_gamma_one_completed', ...
    'roller_gamma_one_completed','roughness_texture_independent', ...
    'T1_T2_unchanged','physical_finite_real','stage2_total_passed','failure_reasons'};
assert(all(isfield(check,required)),'finalize_stage2c:Save', ...
    'Temporary Stage 2 total validation MAT is incomplete.');
movefile(tmp,final,'f');
check = load(final);
assert(all(isfield(check,required)),'finalize_stage2c:Save', ...
    'Final Stage 2 total validation MAT is incomplete.');
end

function write_stage2c_result(outputRoot,summary,ballOff,ballFeedback,roller,rough,order)
tmp = fullfile(outputRoot,'stage2c_result.tmp.mat');
final = fullfile(outputRoot,'stage2c_result.mat');
if isfile(tmp), delete(tmp); end
result = struct('single_step_kernel','verified previously', ...
    'isolated_worker','verified previously','gamma_path','verified previously', ...
    'roller_off',roller,'roller_smooth_feedback',roller.smooth_passed, ...
    'roller_rough_feedback',rough,'ball_off',ballOff,'ball_feedback',ballFeedback, ...
    'load_partition',struct('ball',summary.ball_load_partition_passed, ...
        'roller',summary.roller_load_partition_passed), ...
    'slice_load_consistency','verified previously', ...
    'mechanical_balance',struct('ball',ballFeedback.gamma_one_completed, ...
        'roller',rough.gamma_one_completed), ...
    'stage2_total',summary,'order_independence',order, ...
    'all_passed',summary.stage2_total_passed,'decision',summary.decision);
save(tmp,'result','-v7');
check = load(tmp);
assert(isfield(check,'result') && isfield(check.result,'all_passed') && ...
    isfield(check.result,'decision'),'finalize_stage2c:Save', ...
    'Temporary Stage 2C result MAT is incomplete.');
movefile(tmp,final,'f');
check = load(final);
assert(isfield(check,'result') && isfield(check.result,'stage2_total'), ...
    'finalize_stage2c:Save','Final Stage 2C result MAT is incomplete.');
end

function write_decision(outputRoot,decision)
tmp = fullfile(outputRoot,'stage2c_decision.tmp.txt');
final = fullfile(outputRoot,'stage2c_decision.txt');
fileId = fopen(tmp,'w');
assert(fileId >= 0,'finalize_stage2c:Decision','Cannot create temporary decision file.');
cleanup = onCleanup(@() fclose(fileId)); %#ok<NASGU>
fprintf(fileId,'%s',decision);
clear cleanup
movefile(tmp,final,'f');
text = fileread(final);
assert(strcmp(text,decision),'finalize_stage2c:Decision','Decision file content is invalid.');
end

function text = reason_if(condition,message)
if condition, text = message; else, text = ''; end
end

function combined = join_reasons(parts)
parts = parts(~cellfun(@isempty,parts));
if isempty(parts), combined = ''; else, combined = strjoin(parts,' | '); end
end
