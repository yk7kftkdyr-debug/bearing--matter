function result = test_stage2c_roller_feedback(projectRoot)
%TEST_STAGE2C_ROLLER_FEEDBACK One-process/one-solve contract for Stage 2C.
if nargin < 1 || isempty(projectRoot)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
addpath(projectRoot);
addpath(fullfile(projectRoot,'roughness'));
assert(exist('run_roller_staggered_feedback','file') == 2, ...
    'test_stage2c_roller_feedback:MissingRunner', ...
    'Stage 2C requires the roller staggered feedback runner.');

root = tempname;
cleanup = onCleanup(@()remove_temp(root)); %#ok<NASGU>
mkdir(root);
runDir = fullfile(root,'run');
copyfile(fullfile(projectRoot,'滚子轴承程序'),runDir);
micro = make_micro_interface_config(struct('roughness',roughness_config(12e-9,12e-9,12e-9)));
previous = struct('delta_h_outer',zeros(30,1),'delta_h_inner',zeros(30,1), ...
    'roller_id',[],'restart_state',struct());
step = run_roller_staggered_feedback(runDir,micro,previous,0);
assert(step.success && step.qiujieall_call_count == 1, ...
    'test_stage2c_roller_feedback:CallCount','The single-step kernel must run qiujieall exactly once.');
assert(step.mechanical_converged && step.gt_converged && step.physical_finite_real, ...
    'test_stage2c_roller_feedback:Physical','The single-step kernel returned invalid physics.');
assert(numel(step.roller_id) == step.loadj && numel(unique(step.roller_id)) == step.loadj, ...
    'test_stage2c_roller_feedback:RollerIds','Loaded roller IDs are incomplete.');
assert(numel(step.outer_contacts) == step.loadj && numel(step.inner_contacts) == step.loadj, ...
    'test_stage2c_roller_feedback:Contacts','GT must be evaluated once per loaded roller and side.');
assert(size(step.restart_state.slice_load_outer,1) == 150 && ...
    size(step.restart_state.slice_load_inner,1) == 150, ...
    'test_stage2c_roller_feedback:Slices','The returned mechanical state must retain the 150-slice load fields.');
assert(all(step.delta_h_outer == 0) && all(step.delta_h_inner == 0), ...
    'test_stage2c_roller_feedback:GammaZero','Gamma zero must not alter frozen film corrections.');
result = step;
end

function r = roughness_config(Rqi,Rqo,Rqe)
pair = struct('etaAsperity',1e10,'betaAsperity',1e-6,'C_GT',1e-4,'muBoundary',0.15); % TEST_ONLY; Stage 1 values.
r = struct('enabled',true,'mode','diagnostic','feedback_level',1, ...
    'Rq_inner',Rqi,'Rq_outer',Rqo,'Rq_element',Rqe, ...
    'inner_pair',pair,'outer_pair',pair,'omega',0.5);
end

function remove_temp(pathName)
if isfolder(pathName), rmdir(pathName,'s'); end
end
