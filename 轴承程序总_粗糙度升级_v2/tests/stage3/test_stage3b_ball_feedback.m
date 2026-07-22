function result = test_stage3b_ball_feedback(projectRoot,resultFile)
%TEST_STAGE3B_BALL_FEEDBACK Acceptance of frozen mixed-traction feedback.
% Runs only the four specified ball-bearing cases.  No roller solver is used.

if nargin < 1 || isempty(projectRoot)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
if nargin < 2
    resultFile = '';
end
addpath(projectRoot);
addpath(fullfile(projectRoot,'roughness'));
addpath(fullfile(projectRoot,'tests','baseline_legacy'));
decision = strtrim(fileread(fullfile(projectRoot,'reports','stage3','stage3a_decision.txt')));
assert(strcmp(decision,'GO_TO_STAGE3B_BALL'), ...
    'test_stage3b_ball_feedback:Gate','Stage 3A decision does not permit Stage 3B.');

inputFile = fullfile(projectRoot,'reports','baseline','ball_repeat1.mat');
assert(isfile(inputFile),'test_stage3b_ball_feedback:Input','Missing saved Stage 0 ball baseline.');
workRoot = tempname;
mkdir(workRoot);
cleanup = onCleanup(@()remove_tree(workRoot));

try
off = run_case('off',inputFile,workRoot,rough_config(false,0,0));
level1 = run_case('level1_smooth',inputFile,workRoot,rough_config(true,1,0));
level2Smooth = run_case('level2_smooth',inputFile,workRoot,rough_config(true,2,0));
level2Rough = run_case('level2_rough',inputFile,workRoot,rough_config(true,2,12e-9));

offComparison = compare_stage0_results(inputFile,off.state.raw_result_file, ...
    struct('relative_tolerance',1e-10));
offPassed = off.report.success && offComparison.readable && ...
    offComparison.loaded_element_count_equal && ...
    (strcmp(offComparison.status,'NUMERICALLY_IDENTICAL') || ...
    (strcmp(offComparison.status,'WITHIN_TOLERANCE') && ...
    offComparison.max_relative_error <= 1e-10)) && ...
    strcmp(physics_hash(inputFile),physics_hash(off.state.raw_result_file)) && ...
    finite_state(off.state);
level1Passed = level1.report.success && level1.report.feedback_level == 1 && ...
    level1.state.roughness_feedback.gamma_final == 1;
smooth = level2Smooth.state.roughness_feedback.traction;
smoothPassed = level2Smooth.report.success && level2Smooth.report.feedback_level == 2 && ...
    smooth.gamma_mu_final == 1 && smooth.limit_pass && ...
    max_relative(physical_values(level2Smooth.state),physical_values(level1.state)) <= 1e-10;
rough = level2Rough.state.roughness_feedback.traction;
normal = level2Rough.state.roughness_feedback;
partition = all(abs([normal.outer.Qfluid]+[normal.outer.Qasperity]-[normal.outer.Qtotal]) ...
    ./ max(abs([normal.outer.Qtotal]),1) < 1e-6) && ...
    all(abs([normal.inner.Qfluid]+[normal.inner.Qasperity]-[normal.inner.Qtotal]) ...
    ./ max(abs([normal.inner.Qtotal]),1) < 1e-6);
roughPassed = level2Rough.report.success && rough.gamma_mu_final == 1 && ...
    isstruct(rough.outer) && isstruct(rough.inner) && ...
    ~isempty(rough.outer) && ~isempty(rough.inner) && partition && finite_state(level2Rough.state);
if roughPassed
    roughContacts = [rough.outer(:); rough.inner(:)];
    roughPassed = all([roughContacts.finite]);
    directionPassed = all([roughContacts.traction_direction].*[roughContacts.slip_velocity] < 0);
    dissipationPassed = all([roughContacts.friction_power] <= 0);
else
    directionPassed = false;
    dissipationPassed = false;
end
normalUnchanged = rough.normal_state_unchanged && level2Smooth.state.roughness_feedback.traction.normal_state_unchanged;

result = struct('off_passed',offPassed,'level1_passed',level1Passed, ...
    'smooth_passed',smoothPassed,'rough_passed',roughPassed, ...
    'direction_passed',directionPassed,'dissipation_passed',dissipationPassed, ...
    'normal_state_unchanged',normalUnchanged,'off_comparison',offComparison, ...
    'off',off,'level1',level1, ...
    'level2_smooth',level2Smooth,'level2_rough',level2Rough);
allPassed = offPassed && level1Passed && smoothPassed && roughPassed && ...
    directionPassed && dissipationPassed && normalUnchanged;
write_results(projectRoot,result,allPassed,resultFile);
assert(allPassed,'test_stage3b_ball_feedback:Acceptance', ...
    'Stage 3B ball traction-feedback acceptance failed.');
catch exception
    if ~exist('result','var')
        result = struct('off_passed',false,'level1_passed',false, ...
            'smooth_passed',false,'rough_passed',false,'direction_passed',false, ...
            'dissipation_passed',false,'normal_state_unchanged',false, ...
            'failure_reason',getReport(exception,'basic','hyperlinks','off'));
        write_results(projectRoot,result,false,resultFile);
    end
    rethrow(exception);
end
end

function item = run_case(caseId,inputFile,workRoot,roughness)
caseInput = struct('case_id',caseId,'input_file',inputFile, ...
    'output_dir',fullfile(workRoot,caseId),'rng_seed',0,'roughness',roughness);
[item.state,item.report] = legacy_run_ball(caseInput);
end

function config = rough_config(enabled,level,rq)
pair = struct('etaAsperity',1e10,'betaAsperity',1e-6, ...
    'C_GT',1e-4,'muBoundary',0.15); % Stage 1 TEST_ONLY values, reused unchanged.
config = struct('enabled',enabled,'mode','diagnostic','feedback_level',level, ...
    'Rq_inner',rq,'Rq_outer',rq,'Rq_element',rq, ...
    'inner_pair',pair,'outer_pair',pair,'gamma_sequence',[0 0.5 1], ...
    'gamma_mu_sequence',[0 0.5 1],'max_outer_iterations',8,'omega',0.5, ...
    'errQ_tol',1e-3,'errH_tol',1e-4,'closure_tol',1e-6);
end

function value = physical_values(state)
value = [state.contact_load_inner(:);state.contact_load_outer(:); ...
    state.oil_film_inner(:);state.oil_film_outer(:); ...
    flatten(state.contact_angle);state.working_clearance(:); ...
    state.stiffness(:);state.returndata(:)];
end

function value = flatten(item)
if isnumeric(item)
    value = item(:);
elseif isstruct(item)
    value = [];
    names = sort(fieldnames(item));
    for index = 1:numel(names)
        value = [value;flatten(item.(names{index}))]; %#ok<AGROW>
    end
else
    value = [];
end
end

function value = max_relative(a,b)
assert(isequal(size(a),size(b)), ...
    'test_stage3b_ball_feedback:Shape','Physical result shape changed unexpectedly.');
value = max(abs(a-b)./max(abs(b),1));
end

function digest = physics_hash(fileName)
data = load(fileName);
assert(isfield(data,'result') && isfield(data.result,'generated'), ...
    'test_stage3b_ball_feedback:Hash','Saved legacy result is incomplete.');
generated = data.result.generated;
value = [double(data.result.loadj); generated_value(generated,'Q1','Q1'); ...
    generated_value(generated,'Q2','Q2'); generated_value(generated,'oilh1','oilh1'); ...
    generated_value(generated,'oilh2','oilh2'); generated_value(generated,'Ph1','Ph1'); ...
    generated_value(generated,'Ph2','Ph2'); generated_value(generated,'kk','kk'); ...
    generated_value(generated,'deltaw','deltaw'); double(data.result.returndata(:))];
% Use the complete canonical physical byte stream rather than a Java digest
% so this regression remains runnable with MATLAB -nojvm.
digest = lower(reshape(dec2hex(typecast(value(:),'uint8'),2).',1,[]));
end

function value = generated_value(generated,container,name)
value = [];
if isfield(generated,container) && isfield(generated.(container),name)
    value = double(generated.(container).(name)(:));
end
end

function valid = finite_state(state)
value = physical_values(state);
valid = ~isempty(value) && all(isfinite(value)) && isreal(value);
end

function write_results(projectRoot,result,allPassed,resultFile)
reportRoot = fullfile(projectRoot,'reports','stage3');
if ~isfolder(reportRoot), mkdir(reportRoot); end
if nargin < 4 || isempty(resultFile)
    final = fullfile(reportRoot,'stage3b_ball_result.mat');
    decisionFinal = fullfile(reportRoot,'stage3b_decision.txt');
else
    final = resultFile;
    decisionFinal = decision_path(resultFile,'stage3b_ball_result','stage3b_decision');
end
tmp = atomic_mat_path(final);
save(tmp,'result','-v7');
check = load(tmp);
assert(isfield(check,'result') && isfield(check.result,'rough_passed'), ...
    'test_stage3b_ball_feedback:Save','Temporary result was not written correctly.');
movefile(tmp,final,'f');
if allPassed
    decision = 'GO_TO_STAGE3C_ROLLER';
else
    decision = 'HOLD_STAGE3B';
end
decisionTemp = [decisionFinal '.write_tmp'];
fileId = fopen(decisionTemp,'w');
assert(fileId >= 0,'test_stage3b_ball_feedback:Save','Cannot write Stage 3B decision.');
cleanup = onCleanup(@()fclose(fileId));
fprintf(fileId,'%s',decision);
clear cleanup
movefile(decisionTemp,decisionFinal,'f');
end

function pathName = decision_path(resultFile,resultToken,decisionToken)
[folder,base,~] = fileparts(resultFile);
base = strrep(base,resultToken,decisionToken);
pathName = fullfile(folder,[base '.txt']);
end

function pathName = atomic_mat_path(final)
[folder,base,extension] = fileparts(final);
pathName = fullfile(folder,[base '.write_tmp' extension]);
end

function remove_tree(pathName)
if isfolder(pathName), rmdir(pathName,'s'); end
end
