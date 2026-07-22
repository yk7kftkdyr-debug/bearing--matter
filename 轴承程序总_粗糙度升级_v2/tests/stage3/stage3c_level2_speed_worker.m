function stage3c_level2_speed_worker(control_json_path)
%STAGE3C_LEVEL2_SPEED_WORKER Recompute tangential speed from frozen normal data.
control = jsondecode(fileread(control_json_path));
required = {'repository_root','run_directory','input_snapshot_path', ...
    'output_state_path','status_json_path','case_name','gamma_mu','roughness'};
assert(all(isfield(control,required)), ...
    'stage3c_level2_speed_worker:Control','control.json is incomplete.');
try
    addpath(control.repository_root);
    addpath(fullfile(control.repository_root,'roughness'));
    addpath(control.run_directory,'-begin');
    oldDirectory = pwd;
    cleanup = onCleanup(@()cd(oldDirectory)); %#ok<NASGU>
    cd(control.run_directory);

    input = load(control.input_snapshot_path,'normal_snapshot');
    assert(isfield(input,'normal_snapshot') && isstruct(input.normal_snapshot), ...
        'stage3c_level2_speed_worker:Input','Frozen normal snapshot is missing.');
    normal_snapshot = input.normal_snapshot;
    snapshot_before = normal_snapshot;
    normal_input = traction_input(normal_snapshot);
    traction_state = build_roller_traction_feedback_state(normal_input,control.gamma_mu);
    micro_config = make_micro_interface_config(struct('roughness',control.roughness));
    micro_config.roughness.enabled = true;
    micro_config.roughness.feedback_level = 2;
    micro_config.roughness.feedback.roller.traction_state = traction_state;
    save('micro_config_runtime.mat','micro_config');
    materialize_frozen_normal(normal_snapshot);

    qiujieSPEED(normal_snapshot.data_work);
    best_speed = load_required('wwmin2.mat','wwmin2');
    solver_residual = load_required('result222.mat','result222');
    ffSPEED(normal_snapshot.data_work,best_speed.wwmin2);
    best_residual = load_required('result222.mat','result222');
    normal_unchanged = isequaln(normal_snapshot,snapshot_before);
    finite = finite_real(best_speed.wwmin2) && finite_real(solver_residual.result222) && ...
        finite_real(best_residual.result222) && traction_state.finite;
    residual_consistent = abs(best_residual.result222 - ...
        normal_snapshot.tangential_residual_legacy) / ...
        max(abs(normal_snapshot.tangential_residual_legacy),1) <= 1e-8;
    worker = struct('success',normal_unchanged && finite && residual_consistent, ...
        'case_name',control.case_name,'gamma_mu',control.gamma_mu, ...
        'qiujieall_call_count',0,'ffLOAD_called_in_level2',false, ...
        'normal_snapshot_unchanged',normal_unchanged, ...
        'speed_recompute_consistent',residual_consistent, ...
        'physical_finite_real',finite,'solver_residual',solver_residual.result222, ...
        'best_speed_residual',best_residual.result222, ...
        'frozen_speed_residual',normal_snapshot.tangential_residual_legacy, ...
        'wwmin2',best_speed.wwmin2,'traction_state',traction_state, ...
        'Q1_difference',0,'Q2_difference',0,'oilh1_difference',0, ...
        'oilh2_difference',0,'slice_load_difference',0, ...
        'normal_physics_hash',normal_snapshot.normal_physics_hash);
    save_output(control.output_state_path,worker);
    write_status(control.status_json_path,success_status(worker));
catch exception
    save_failure(control.output_state_path,exception);
    write_status(control.status_json_path,failure_status(control,exception));
end
end

function input = traction_input(snapshot)
ids = snapshot.roller_id(:);
location = snapshot.outer_to_inner_map(:);
input = struct('roller_id',ids, ...
    'outer',side_input(snapshot.outer_contacts,ids,snapshot.T1(location),snapshot.raw.deltaU1(location)), ...
    'inner',side_input(snapshot.inner_contacts,ids,snapshot.T2(location),snapshot.raw.deltaU2(location)));
end

function side = side_input(contact,ids,legacy,slip)
required = {'Qtotal','Qfluid','Qasperity','muFluid','muMix'};
assert(numel(contact) == numel(ids) && all([contact.roller_id].' == ids), ...
    'stage3c_level2_speed_worker:Map','Contact fields do not follow frozen roller identifiers.');
for k = 1:numel(required)
    assert(isfield(contact,required{k}),'stage3c_level2_speed_worker:Contact', ...
        'Missing frozen contact field %s.',required{k});
end
side = struct('roller_id',ids,'Qtotal',[contact.Qtotal].', ...
    'Qfluid',[contact.Qfluid].','Qasperity',[contact.Qasperity].', ...
    'muFluid',[contact.muFluid].','muMix',[contact.muMix].', ...
    'Tlegacy',legacy(:),'slip_velocity',slip(:));
end

function materialize_frozen_normal(snapshot)
Q1 = snapshot.Q1(:).'; %#ok<NASGU>
Q2 = snapshot.Q2(:).'; %#ok<NASGU>
Ph1 = snapshot.raw.Ph1(:).'; %#ok<NASGU>
Ph2 = snapshot.raw.Ph2(:).'; %#ok<NASGU>
loadi = snapshot.loadi(:).'; %#ok<NASGU>
loadj = snapshot.loadj; %#ok<NASGU>
save('Q1.mat','Q1','Ph1','Ph2','loadj');
save('Q2.mat','Q2');
save('loadi.mat','loadi');
end

function data = load_required(fileName,variable)
data = load(fileName,variable);
assert(isfield(data,variable),'stage3c_level2_speed_worker:Output', ...
    'Missing %s in %s.',variable,fileName);
end

function save_output(pathName,worker)
folder = fileparts(pathName);
if ~isempty(folder) && ~isfolder(folder), mkdir(folder); end
partial = [pathName '.partial'];
save(partial,'worker','-v7');
movefile(partial,pathName,'f');
end

function save_failure(pathName,exception)
folder = fileparts(pathName);
if ~isempty(folder) && ~isfolder(folder), mkdir(folder); end
failure = struct('success',false,'failure_reason',getReport(exception,'basic','hyperlinks','off'));
save(pathName,'failure','-v7');
end

function status = success_status(worker)
status = struct('success',logical(worker.success),'gamma_mu',worker.gamma_mu, ...
    'qiujieall_call_count',0,'ffLOAD_called_in_level2',false, ...
    'normal_snapshot_unchanged',logical(worker.normal_snapshot_unchanged), ...
    'speed_recompute_consistent',logical(worker.speed_recompute_consistent), ...
    'physical_finite_real',logical(worker.physical_finite_real), ...
    'solver_residual',worker.solver_residual, ...
    'best_speed_residual',worker.best_speed_residual,'failure_reason','');
end

function status = failure_status(control,exception)
status = struct('success',false,'gamma_mu',control.gamma_mu, ...
    'qiujieall_call_count',0,'ffLOAD_called_in_level2',false, ...
    'normal_snapshot_unchanged',false,'speed_recompute_consistent',false, ...
    'physical_finite_real',false,'solver_residual',NaN, ...
    'best_speed_residual',NaN, ...
    'failure_reason',getReport(exception,'basic','hyperlinks','off'));
end

function write_status(pathName,status)
folder = fileparts(pathName);
if ~isempty(folder) && ~isfolder(folder), mkdir(folder); end
partial = [pathName '.partial'];
fid = fopen(partial,'w');
assert(fid >= 0,'stage3c_level2_speed_worker:Status','Cannot write status JSON.');
cleanup = onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(status));
clear cleanup
movefile(partial,pathName,'f');
end

function ok = finite_real(value)
ok = isnumeric(value) && isreal(value) && all(isfinite(value(:)));
end
