function [state, report] = legacy_run_ball(caseInput)
%LEGACY_RUN_BALL Compatibility wrapper for legacy, shadow, and feedback modes.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root,'roughness'));
roughConfig = roughness_config(caseInput);
if roughConfig.enabled && roughConfig.feedback_level == 1
    [state,report] = run_feedback(caseInput,roughConfig);
else
    [state,report] = run_legacy(caseInput,roughConfig);
end
end

function [state, report] = run_legacy(c,roughConfig)
required = {'case_id','input_file','output_dir','rng_seed'};
for index = 1:numel(required)
    assert(isfield(c,required{index}),'legacy_run_ball:Input', ...
        'Missing %s',required{index});
end
assert(isfile(c.input_file),'legacy_run_ball:Input','input_file does not exist');
root = fileparts(mfilename('fullpath'));
if isfolder(c.output_dir), rmdir(c.output_dir,'s'); end
copyfile(root,c.output_dir);
raw = fullfile(c.output_dir,'raw_result.mat');
old = pwd;
cleanup = onCleanup(@() cd(old));
addpath(fullfile(root,'tests','baseline_legacy'));
rng(c.rng_seed,'twister');
startTime = tic;
result = execute_legacy_ball(c.output_dir,c.input_file);
save(raw,'result');
state = legacy_state(result.generated,result,c.case_id,raw);
contactFile = fullfile(c.output_dir,'球轴承程序','q1q2a1a2.mat');
if isfile(contactFile)
    state = enrich_legacy_contact_state(state,load(contactFile));
end
if roughConfig.enabled
    assert(roughConfig.feedback_level == 0 && strcmp(roughConfig.mode,'diagnostic'), ...
        'legacy_run_ball:RoughnessMode', ...
        'Only diagnostic shadow mode is permitted when feedback_level is zero.');
    assert(isfile(contactFile),'legacy_run_ball:RoughnessData', ...
        'Missing saved ball contact results.');
    state.roughness = ball_roughness_shadow(state,load(contactFile),roughConfig);
end

report = legacy_report(result,state,c.case_id,toc(startTime));
report.feedback_level = 0;
report.gamma_final = NaN;
report.outer_converged = false;
report.closure_pass = false;
report.load_share_pass = false;
end

function result = execute_legacy_ball(projectRoot,inputFile)
% Keep the legacy mechanics intact while avoiding startup_roughness_v2,
% whose clear-functions reset prevents this wrapper from returning state.
source = load(inputFile);
assert(isfield(source,'result') && isfield(source.result,'input'), ...
    'legacy_run_ball:Input','input_file must contain result.input.');
input = source.result.input;
clear global;
lastwarn('');
startTime = tic;
workDir = fullfile(projectRoot,'球轴承程序');
old = pwd;
cleanup = onCleanup(@() cd(old));
cd(workDir);
try
    stdout = evalc('[loadj, returndata] = qiujieend(input, make_micro_interface_config());');
    success = true;
catch exception
    stdout = getReport(exception,'extended','hyperlinks','off');
    loadj = NaN;
    returndata = [];
    success = false;
end
[warningText,warningId] = lastwarn;
result = struct('success',success,'loadj',loadj,'returndata',returndata, ...
    'bearing_type','ball','load_factor',1,'speed_factor',1,'input',input, ...
    'stdout',stdout,'warning',warningText,'warning_id',warningId, ...
    'runtime_s',toc(startTime),'generated',collect_legacy_outputs(workDir), ...
    'result333',parse_result333(stdout));
end

function outputs = collect_legacy_outputs(workDir)
names = {'Q1','Q2','oilh1','oilh2','Ph1','Ph2','kk','deltaw','roller_slip_speed_check'};
outputs = struct();
for index = 1:numel(names)
    fileName = fullfile(workDir,[names{index} '.mat']);
    if isfile(fileName)
        outputs.(names{index}) = load(fileName);
    end
end
end

function info = parse_result333(stdout)
tokens = regexp(stdout,'result333\\s*=\\s*([+-]?\\d*\\.?\\d+(?:[eE][+-]?\\d+)?)','tokens');
values = cellfun(@(value) str2double(value{1}),tokens);
info = struct('classification','LEGACY_VERBOSE_CONVERGENCE_OUTPUT', ...
    'count',numel(values),'initial',NaN,'final',NaN,'maximum',NaN, ...
    'nonfinite',any(~isfinite(values)));
if ~isempty(values)
    info.initial = values(1);
    info.final = values(end);
    info.maximum = max(values);
end
end

function [state, report] = run_feedback(c,roughConfig)
input = feedback_input(c);
runnerCase = struct('input',input,'output_dir',c.output_dir);
startTime = tic;
[feedback,runnerReport] = run_ball_staggered_feedback(runnerCase,roughConfig);
if ~runnerReport.success
    state = empty_feedback_state(c,feedback);
    report = feedback_report(false,false,false,runnerReport,feedback,c,toc(startTime));
    return;
end

state = feedback_state(feedback,c.case_id);
physical = physical_feedback_state(state);
finite = all(isfinite(physical)) && isreal(physical);
loadSharePass = feedback.metrics.load_share_pass;
closurePass = feedback.metrics.closure_pass;
outerConverged = feedback.metrics.outer_converged;
success = finite && loadSharePass && closurePass && outerConverged;
report = feedback_report(success,closurePass,loadSharePass,runnerReport,feedback,c,toc(startTime));
if ~success
    report.status = 'ROUGHNESS_FEEDBACK_INVALID_PHYSICS';
    if finite && loadSharePass && closurePass
        report.status = 'ROUGHNESS_FEEDBACK_NONCONVERGED';
    end
end
end

function input = feedback_input(c)
if isfield(c,'input')
    input = c.input;
    return;
end
assert(isfield(c,'input_file') && isfile(c.input_file), ...
    'legacy_run_ball:Input','Feedback mode requires input or a readable input_file.');
source = load(c.input_file);
assert(isfield(source,'result') && isfield(source.result,'input'), ...
    'legacy_run_ball:Input','input_file must contain result.input for feedback mode.');
input = source.result.input;
end

function config = roughness_config(c)
config = struct('enabled',false,'mode','off','feedback_level',0);
if isfield(c,'roughness')
    config = merge_config(config,c.roughness);
end
if ~config.enabled
    config.feedback_level = 0;
end
assert(ismember(config.feedback_level,[0 1]), ...
    'legacy_run_ball:FeedbackLevel','feedback_level must be 0 or 1.');
end

function base = merge_config(base,overrides)
names = fieldnames(overrides);
for index = 1:numel(names)
    name = names{index};
    if isfield(base,name) && isstruct(base.(name)) && isstruct(overrides.(name))
        base.(name) = merge_config(base.(name),overrides.(name));
    else
        base.(name) = overrides.(name);
    end
end
end

function state = legacy_state(generated,result,caseId,rawFile)
state = struct('bearing_type','ball','case_id',caseId, ...
    'contact_load_inner',pick(generated,'Q1'), ...
    'contact_load_outer',pick(generated,'Q2'), ...
    'oil_film_inner',pick(generated,'oilh1'), ...
    'oil_film_outer',pick(generated,'oilh2'), ...
    'contact_angle',[],'working_clearance',pick(generated,'deltaw'), ...
    'loaded_element_count',result.loadj,'stiffness',pick(generated,'kk'), ...
    'legacy_metric',result.result333.final,'returndata',result.returndata, ...
    'raw_result_file',rawFile);
end

function state = enrich_legacy_contact_state(state,contact)
% Q1/oilh1 and Q2/oilh2 retain the historical wrapper field convention.
if isfield(contact,'Q1'), state.contact_load_inner = contact.Q1; end
if isfield(contact,'Q2'), state.contact_load_outer = contact.Q2; end
if isfield(contact,'oilh1'), state.oil_film_inner = contact.oilh1; end
if isfield(contact,'oilh2'), state.oil_film_outer = contact.oilh2; end
if isfield(contact,'a1') && isfield(contact,'a2')
    state.contact_angle = struct('outer',contact.a1,'inner',contact.a2);
end
end

function state = feedback_state(feedback,caseId)
legacy = feedback.legacy;
state = struct('bearing_type','ball','case_id',caseId, ...
    'contact_load_inner',legacy.Q1,'contact_load_outer',legacy.Q2, ...
    'oil_film_inner',legacy.oilh1,'oil_film_outer',legacy.oilh2, ...
    'contact_angle',struct('outer',legacy.a1,'inner',legacy.a2), ...
    'working_clearance',legacy.deltaw, ...
    'loaded_element_count',legacy.loadj,'stiffness',legacy.kk, ...
    'legacy_metric',NaN,'returndata',feedback.returndata, ...
    'raw_result_file','');
state.roughness_feedback = struct('gamma_final',feedback.metrics.gamma_final, ...
    'outer_iterations',feedback.metrics.outer_iterations, ...
    'errQ',feedback.metrics.errQ,'errH',feedback.metrics.errH, ...
    'closure_error',feedback.metrics.closure_error, ...
    'active_set_stable',feedback.metrics.active_set_stable, ...
    'outer',side_state(feedback.states.outer), ...
    'inner',side_state(feedback.states.inner));
end

function state = empty_feedback_state(c,feedback)
state = struct('bearing_type','ball','case_id',c.case_id, ...
    'contact_load_inner',[],'contact_load_outer',[],'oil_film_inner',[], ...
    'oil_film_outer',[],'contact_angle',[],'working_clearance',[], ...
    'loaded_element_count',NaN,'stiffness',[],'legacy_metric',NaN, ...
    'returndata',feedback.returndata,'raw_result_file','');
state.roughness_feedback = struct('gamma_final',feedback.metrics.gamma_final, ...
    'outer_iterations',feedback.metrics.outer_iterations, ...
    'errQ',feedback.metrics.errQ,'errH',feedback.metrics.errH, ...
    'closure_error',feedback.metrics.closure_error, ...
    'active_set_stable',feedback.metrics.active_set_stable, ...
    'outer',side_state(feedback.states.outer), ...
    'inner',side_state(feedback.states.inner));
end

function side = side_state(states)
fields = {'Qtotal','Qfluid','Qasperity','hMix','lambda','chiA','loadBalanceError'};
side = struct();
for index = 1:numel(fields)
    name = fields{index};
    if isempty(states)
        side.(name) = [];
    else
        side.(name) = reshape([states.(name)],[],1);
    end
end
end

function report = legacy_report(result,state,caseId,runtime)
report = struct('success',result.success,'status',legacy_status(result), ...
    'case_id',caseId,'runtime',runtime, ...
    'warning_count',double(~isempty(result.warning)), ...
    'nan_count',count_numeric(state,@isnan), ...
    'inf_count',count_numeric(state,@isinf), ...
    'complex_count',count_numeric(state,@(x) ~isreal(x)), ...
    'message',result.warning);
end

function report = feedback_report(success,closurePass,loadSharePass,runnerReport,feedback,c,runtime)
if success
    status = 'ROUGHNESS_FEEDBACK_SUCCESS';
else
    status = 'ROUGHNESS_FEEDBACK_NONCONVERGED';
end
report = struct('success',success,'status',status,'case_id',c.case_id, ...
    'runtime',runtime,'warning_count',0,'nan_count',0,'inf_count',0, ...
    'complex_count',0,'message',runnerReport.message, ...
    'feedback_level',1,'gamma_final',feedback.metrics.gamma_final, ...
    'outer_converged',feedback.metrics.outer_converged, ...
    'closure_pass',closurePass,'load_share_pass',loadSharePass);
end

function value = pick(generated,name)
value = [];
if isfield(generated,name)
    container = generated.(name);
    if isfield(container,name)
        value = container.(name);
    else
        fields = fieldnames(container);
        if ~isempty(fields), value = container.(fields{1}); end
    end
end
end

function value = physical_feedback_state(state)
value = [state.contact_load_inner(:);state.contact_load_outer(:); ...
    state.oil_film_inner(:);state.oil_film_outer(:); ...
    state.working_clearance(:);state.stiffness(:);state.returndata(:)];
end

function number = count_numeric(value,fn)
number = 0;
fields = fieldnames(value);
for index = 1:numel(fields)
    item = value.(fields{index});
    if isnumeric(item)
        number = number + sum(fn(item(:)));
    end
end
end

function value = legacy_status(result)
if result.success
    value = 'LEGACY_SUCCESS';
else
    value = 'LEGACY_FAILED';
end
end
