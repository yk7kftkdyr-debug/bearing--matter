function stage2c_isolated_worker(control_json_path)
%STAGE2C_ISOLATED_WORKER One MATLAB process, one Stage 2C mechanical solve.
% Python owns process lifetime and only copies complete MAT state files.
control = jsondecode(fileread(control_json_path));
required = {'repository_root','run_directory','input_state_path','output_state_path', ...
    'status_json_path','case_name','gamma','staggered_iteration','roughness'};
assert(all(isfield(control,required)), ...
    'stage2c_isolated_worker:Control','control.json is missing a required field.');
try
    addpath(control.repository_root);
    addpath(fullfile(control.repository_root,'roughness'));
    addpath(control.run_directory);
    previous = initial_state(control.run_directory);
    if isfile(control.input_state_path)
        inputData = load(control.input_state_path,'state');
        assert(isfield(inputData,'state') && isstruct(inputData.state), ...
            'stage2c_isolated_worker:InputState','input_state.mat must contain struct state.');
        previous = inputData.state;
    end
    baseline_config = make_micro_interface_config();
    texture_before = baseline_config.texture;
    micro_config = make_micro_interface_config(struct('roughness',control.roughness));
    state = run_roller_staggered_feedback(control.run_directory,micro_config,previous,control.gamma);
    assert(state.qiujieall_call_count == 1, ...
        'stage2c_isolated_worker:CallCount','A worker must call qiujieall exactly once.');
    % Audit only: prove roughness override did not mutate the full texture struct.
    state.config_audit = struct('texture_before',texture_before, ...
        'texture_after',micro_config.texture,'roughness',micro_config.roughness);
    save_state(control.output_state_path,state);
    write_status(control.status_json_path,success_status(state,control));
catch exception
    state = struct('success',false,'failure_reason',getReport(exception,'basic','hyperlinks','off'));
    save_state(control.output_state_path,state);
    write_status(control.status_json_path,failure_status(control,exception));
end
end

function previous = initial_state(runDirectory)
data = defaultBearingInput();
previous = struct('delta_h_outer',zeros(data(1),1),'delta_h_inner',zeros(data(1),1), ...
    'roller_id',[],'Q1',[],'Q2',[],'restart_state',struct());
assert(isfolder(runDirectory),'stage2c_isolated_worker:RunDirectory','run_directory does not exist.');
end

function status = success_status(state,control)
status = struct('success',logical(state.success),'gamma',control.gamma, ...
    'staggered_iteration',control.staggered_iteration, ...
    'qiujieall_call_count',state.qiujieall_call_count, ...
    'mechanical_converged',logical(state.mechanical_converged), ...
    'gt_converged',logical(state.gt_converged),'errQ',state.errQ, ...
    'errH',state.errH,'closure_error',state.closure_error, ...
    'active_set_stable',logical(state.active_set_stable), ...
    'physical_finite_real',logical(state.physical_finite_real), ...
    'loadj',state.loadj,'roller_id',reshape(state.roller_id,1,[]), ...
    'outer_contacts',state.outer_contacts,'inner_contacts',state.inner_contacts, ...
    'failure_reason','');
end

function status = failure_status(control,exception)
status = struct('success',false,'gamma',control.gamma, ...
    'staggered_iteration',control.staggered_iteration, ...
    'qiujieall_call_count',0,'mechanical_converged',false, ...
    'gt_converged',false,'errQ',NaN,'errH',NaN,'closure_error',NaN, ...
    'active_set_stable',false,'physical_finite_real',false,'loadj',0, ...
    'roller_id',[],'outer_contacts',[],'inner_contacts',[], ...
    'failure_reason',getReport(exception,'basic','hyperlinks','off'));
end

function save_state(pathName,state)
folder = fileparts(pathName);
if ~isempty(folder) && ~isfolder(folder), mkdir(folder); end
partial = [pathName '.partial'];
save(partial,'state','-v7');
movefile(partial,pathName,'f');
end

function write_status(pathName,status)
folder = fileparts(pathName);
if ~isempty(folder) && ~isfolder(folder), mkdir(folder); end
partial = [pathName '.partial'];
fid = fopen(partial,'w');
assert(fid >= 0,'stage2c_isolated_worker:Status','Cannot write status JSON.');
cleanup = onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(status));
clear cleanup
movefile(partial,pathName,'f');
end
