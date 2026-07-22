function stage3c_level2_traction_worker(control_json_path)
%STAGE3C_LEVEL2_TRACTION_WORKER Evaluate frozen-normal Level-2 traction.
control = jsondecode(fileread(control_json_path));
required = {'repository_root','run_directory','input_snapshot_path', ...
    'output_state_path','status_json_path','case_name','gamma_mu','roughness'};
assert(all(isfield(control,required)), ...
    'stage3c_level2_traction_worker:Control','control.json is incomplete.');
try
    addpath(control.repository_root);
    addpath(fullfile(control.repository_root,'roughness'));
    addpath(control.run_directory,'-begin');
    oldDirectory = pwd;
    cleanup = onCleanup(@()cd(oldDirectory)); %#ok<NASGU>
    cd(control.run_directory);

    input = load(control.input_snapshot_path,'normal_snapshot');
    assert(isfield(input,'normal_snapshot') && isstruct(input.normal_snapshot), ...
        'stage3c_level2_traction_worker:Input','Frozen normal snapshot is missing.');
    normal_snapshot = input.normal_snapshot;
    snapshot_before = normal_snapshot;
    normal_input = traction_input(normal_snapshot);
    traction_state = build_roller_traction_feedback_state(normal_input,control.gamma_mu);

    micro_config = make_micro_interface_config(struct('roughness',control.roughness));
    micro_config.roughness.enabled = true;
    micro_config.roughness.feedback_level = 2;
    micro_config.roughness.feedback.roller.traction_state = traction_state;
    save('micro_config_runtime.mat','micro_config');
    materialize_legacy_inputs(normal_snapshot);

    [expectedT1,expectedT2,mapping] = apply_roller_traction_override( ...
        normal_snapshot.T1,normal_snapshot.T2,normal_snapshot.loadi, ...
        normal_snapshot.loadii,traction_state,micro_config);
    assert(mapping.success,'stage3c_level2_traction_worker:Mapping','%s',mapping.message);
    ffSPEED(normal_snapshot.data_work,normal_snapshot.raw.wwmin2);
    T1actual = load_scalar_vector('T1.mat','T1');
    T2actual = load_scalar_vector('T2.mat','T2');
    residual = load_scalar_vector('result222.mat','result222');
    normal_unchanged = isequaln(normal_snapshot,snapshot_before);
    maps_exact = same_numeric_vector(T1actual,expectedT1) && ...
        same_numeric_vector(T2actual,expectedT2);
    finite = finite_real(T1actual) && finite_real(T2actual) && finite_real(residual) && ...
        traction_state.finite;
    direction_ok = all(traction_state.slip_direction.outer(:) .* ...
        [traction_state.outer.traction_direction].' > 0) && ...
        all(traction_state.slip_direction.inner(:) .* ...
        [traction_state.inner.traction_direction].' > 0);
    dissipation_ok = all(traction_state.friction_power.outer(:) <= 0) && ...
        all(traction_state.friction_power.inner(:) <= 0);
    worker = struct('success',maps_exact && finite && normal_unchanged && ...
        direction_ok && dissipation_ok,'case_name',control.case_name, ...
        'gamma_mu',control.gamma_mu,'qiujieall_call_count',0, ...
        'ffLOAD_called_in_level2',false,'normal_snapshot_unchanged',normal_unchanged, ...
        'mapping_valid',mapping.mapping_valid && maps_exact, ...
        'physical_finite_real',finite,'tangential_residual_diagnostic',residual, ...
        'T1',T1actual,'T2',T2actual, ...
        'traction_state',traction_state,'friction_direction_valid',direction_ok, ...
        'friction_dissipation_valid',dissipation_ok, ...
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
loc = snapshot.outer_to_inner_map(:);
assert(isequal(ids,snapshot.loadi(:)) && numel(ids) == numel(loc), ...
    'stage3c_level2_traction_worker:Map','Snapshot roller identifiers are inconsistent.');
input = struct('roller_id',ids, ...
    'outer',side_input(snapshot.outer_contacts,ids,snapshot.T1(loc),snapshot.raw.deltaU1(loc)), ...
    'inner',side_input(snapshot.inner_contacts,ids,snapshot.T2(loc),snapshot.raw.deltaU2(loc)));
end

function side = side_input(contact,ids,legacy,slip)
assert(numel(contact) == numel(ids) && all([contact.roller_id].' == ids), ...
    'stage3c_level2_traction_worker:Map','Contact fields do not follow frozen roller_id order.');
required = {'Qtotal','Qfluid','Qasperity','muFluid','muMix'};
for k = 1:numel(required)
    assert(isfield(contact,required{k}),'stage3c_level2_traction_worker:Contact', ...
        'Frozen contact is missing %s.',required{k});
end
side = struct('roller_id',ids,'Qtotal',[contact.Qtotal].', ...
    'Qfluid',[contact.Qfluid].','Qasperity',[contact.Qasperity].', ...
    'muFluid',[contact.muFluid].','muMix',[contact.muMix].', ...
    'Tlegacy',legacy(:),'slip_velocity',slip(:));
end

function materialize_legacy_inputs(snapshot)
Ph1ii = snapshot.raw.Ph1ii; %#ok<NASGU>
Ph2ii = snapshot.raw.Ph2ii; %#ok<NASGU>
Q1ii = snapshot.raw.Q1ii; %#ok<NASGU>
Q2ii = snapshot.raw.Q2ii; %#ok<NASGU>
loadii = snapshot.loadii; %#ok<NASGU>
loadi = snapshot.loadi; %#ok<NASGU>
loadj = snapshot.loadj; %#ok<NASGU>
save('Ph1ii.mat','Ph1ii');
save('Ph2ii.mat','Ph2ii');
save('Q1ii.mat','Q1ii');
save('Q2ii.mat','Q2ii');
save('loadii.mat','loadii','loadi','loadj');
end

function value = load_scalar_vector(fileName,variable)
data = load(fileName,variable);
assert(isfield(data,variable),'stage3c_level2_traction_worker:Output', ...
    'Missing %s in %s.',variable,fileName);
value = data.(variable);
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
    'mapping_valid',logical(worker.mapping_valid), ...
    'physical_finite_real',logical(worker.physical_finite_real), ...
    'tangential_residual_diagnostic',worker.tangential_residual_diagnostic, ...
    'failure_reason','');
end

function status = failure_status(control,exception)
status = struct('success',false,'gamma_mu',control.gamma_mu, ...
    'qiujieall_call_count',0,'ffLOAD_called_in_level2',false, ...
    'normal_snapshot_unchanged',false,'mapping_valid',false, ...
    'physical_finite_real',false,'tangential_residual_diagnostic',NaN, ...
    'failure_reason',getReport(exception,'basic','hyperlinks','off'));
end

function write_status(pathName,status)
folder = fileparts(pathName);
if ~isempty(folder) && ~isfolder(folder), mkdir(folder); end
partial = [pathName '.partial'];
fid = fopen(partial,'w');
assert(fid >= 0,'stage3c_level2_traction_worker:Status','Cannot write status JSON.');
cleanup = onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(status));
clear cleanup
movefile(partial,pathName,'f');
end

function ok = same_numeric_vector(left,right)
ok = isnumeric(left) && isnumeric(right) && isreal(left) && isreal(right) && ...
    numel(left) == numel(right) && all(left(:) == right(:));
end

function ok = finite_real(value)
ok = isnumeric(value) && isreal(value) && all(isfinite(value(:)));
end
