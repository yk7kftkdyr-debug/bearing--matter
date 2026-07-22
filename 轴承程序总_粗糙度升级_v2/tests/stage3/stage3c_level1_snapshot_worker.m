function stage3c_level1_snapshot_worker(control_json_path)
%STAGE3C_LEVEL1_SNAPSHOT_WORKER One isolated normal solve and frozen snapshot.
control = jsondecode(fileread(control_json_path));
required = {'repository_root','run_directory','input_state_path','output_state_path', ...
    'status_json_path','case_name','gamma','staggered_iteration','roughness'};
assert(all(isfield(control,required)), ...
    'stage3c_level1_snapshot_worker:Control','control.json is incomplete.');
try
    addpath(control.repository_root);
    addpath(fullfile(control.repository_root,'roughness'));
    addpath(control.run_directory,'-begin');
    oldDirectory = pwd;
    cleanup = onCleanup(@()cd(oldDirectory)); %#ok<NASGU>
    cd(control.run_directory);

    previous = initial_state();
    if isfile(control.input_state_path)
        input = load(control.input_state_path,'state');
        assert(isfield(input,'state') && isstruct(input.state), ...
            'stage3c_level1_snapshot_worker:Input','input state is invalid.');
        previous = input.state;
    end
    micro_config = make_micro_interface_config(struct('roughness',control.roughness));
    state = run_roller_staggered_feedback(control.run_directory,micro_config,previous,control.gamma);
    assert(state.qiujieall_call_count == 1, ...
        'stage3c_level1_snapshot_worker:CallCount','Level 1 must make one mechanical call.');
    normal_snapshot = build_snapshot(state,control.run_directory);
    worker = struct('success',state.success,'case_name',control.case_name, ...
        'gamma',control.gamma,'staggered_iteration',control.staggered_iteration, ...
        'qiujieall_call_count',1,'ffLOAD_called_in_level2',false, ...
        'mechanical_converged',state.mechanical_converged, ...
        'gt_converged',state.gt_converged,'physical_finite_real',state.physical_finite_real, ...
        'normal_physics_hash',normal_snapshot.normal_physics_hash);
    save_output(control.output_state_path,state,normal_snapshot,worker);
    write_status(control.status_json_path,success_status(worker,state));
catch exception
    save_failure(control.output_state_path,exception);
    write_status(control.status_json_path,failure_status(control,exception));
end
end

function state = initial_state()
data = defaultBearingInput();
state = struct('delta_h_outer',zeros(data(1),1),'delta_h_inner',zeros(data(1),1), ...
    'roller_id',[],'Q1',[],'Q2',[],'restart_state',struct());
end

function snapshot = build_snapshot(state,runDirectory)
required = {'Ph1','Ph2','Ph1ii','Ph2ii','Q1ii','Q2ii','wwmin2', ...
    'deltaU1','deltaU2','result222'};
raw = struct();
for k = 1:numel(required)
    name = required{k};
    data = load(fullfile(runDirectory,[name '.mat']),name);
    assert(isfield(data,name),'stage3c_level1_snapshot_worker:Output', ...
        'Missing %s in the legacy output.',name);
    raw.(name) = data.(name);
end
[found,location] = ismember(state.roller_id(:),state.legacy.loadii(:));
assert(all(found) && numel(unique(state.roller_id(:))) == numel(state.roller_id) && ...
    numel(unique(state.legacy.loadii(:))) == numel(state.legacy.loadii), ...
    'stage3c_level1_snapshot_worker:RollerMap','Invalid frozen roller mapping.');
assert(isequal(location(:),state.outer_to_inner_map(:)), ...
    'stage3c_level1_snapshot_worker:RollerMap','The retained Stage 2C map is inconsistent.');
dataWork = defaultBearingInput();
dataWork(7) = state.legacy.working_clearance;
snapshot = struct('roller_id',state.roller_id(:),'loadi',state.legacy.loadi(:), ...
    'loadii',state.legacy.loadii(:),'outer_to_inner_map',location(:), ...
    'Q1',state.Q1(:),'Q2',state.Q2(:),'oilh1',state.oilh1(:),'oilh2',state.oilh2(:), ...
    'T1',state.T1(:),'T2',state.T2(:),'loadj',state.loadj, ...
    'outer_slice_loads',state.legacy.slice_load_outer, ...
    'inner_slice_loads',state.legacy.slice_load_inner, ...
    'working_clearance',state.legacy.working_clearance, ...
    'normal_physics_hash',state.legacy.physics_hash,'data_work',dataWork, ...
    'outer_contacts',state.outer_contacts,'inner_contacts',state.inner_contacts, ...
    'raw',raw,'tangential_residual_legacy',raw.result222);
end

function save_output(pathName,state,normal_snapshot,worker)
folder = fileparts(pathName);
if ~isempty(folder) && ~isfolder(folder), mkdir(folder); end
partial = [pathName '.partial'];
save(partial,'state','normal_snapshot','worker','-v7');
movefile(partial,pathName,'f');
end
function save_failure(pathName,exception)
folder = fileparts(pathName);
if ~isempty(folder) && ~isfolder(folder), mkdir(folder); end
failure = struct('success',false,'failure_reason',getReport(exception,'basic','hyperlinks','off'));
save(pathName,'failure','-v7');
end

function status = success_status(worker,state)
status = struct('success',logical(worker.success),'gamma',worker.gamma, ...
    'staggered_iteration',worker.staggered_iteration, ...
    'qiujieall_call_count',worker.qiujieall_call_count, ...
    'mechanical_converged',logical(state.mechanical_converged), ...
    'gt_converged',logical(state.gt_converged),'errQ',state.errQ,'errH',state.errH, ...
    'closure_error',state.closure_error,'active_set_stable',logical(state.active_set_stable), ...
    'physical_finite_real',logical(state.physical_finite_real), ...
    'loadj',state.loadj,'roller_id',reshape(state.roller_id,1,[]),'failure_reason','');
end

function status = failure_status(control,exception)
status = struct('success',false,'gamma',control.gamma, ...
    'staggered_iteration',control.staggered_iteration,'qiujieall_call_count',0, ...
    'mechanical_converged',false,'gt_converged',false,'errQ',NaN,'errH',NaN, ...
    'closure_error',NaN,'active_set_stable',false,'physical_finite_real',false, ...
    'loadj',0,'roller_id',[],'failure_reason',getReport(exception,'basic','hyperlinks','off'));
end

function write_status(pathName,status)
folder = fileparts(pathName);
if ~isempty(folder) && ~isfolder(folder), mkdir(folder); end
partial = [pathName '.partial'];
fid = fopen(partial,'w');
assert(fid >= 0,'stage3c_level1_snapshot_worker:Status','Cannot write status JSON.');
cleanup = onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(status));
clear cleanup
movefile(partial,pathName,'f');
end
