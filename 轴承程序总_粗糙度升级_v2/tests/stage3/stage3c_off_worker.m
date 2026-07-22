function stage3c_off_worker(control_json_path)
%STAGE3C_OFF_WORKER One isolated Level-0 legacy roller execution.
control = jsondecode(fileread(control_json_path));
required = {'repository_root','run_directory','output_state_path','status_json_path','case_name'};
assert(all(isfield(control,required)), ...
    'stage3c_off_worker:Control','control.json is incomplete.');
try
    addpath(control.repository_root);
    addpath(fullfile(control.repository_root,'roughness'));
    addpath(control.run_directory,'-begin');
    oldDirectory = pwd;
    cleanup = onCleanup(@()cd(oldDirectory)); %#ok<NASGU>
    cd(control.run_directory);
    roughness = struct('enabled',false,'mode','off','feedback_level',0);
    micro_config = make_micro_interface_config(struct('roughness',roughness));
    legacy = legacy_run_roller(control.run_directory,micro_config,struct());
    worker = struct('success',legacy.converged,'case_name',control.case_name, ...
        'qiujieall_call_count',legacy.qiujieall_call_count,'Q1',legacy.Q1, ...
        'Q2',legacy.Q2,'oilh1',legacy.oilh1,'oilh2',legacy.oilh2, ...
        'T1',legacy.T1,'T2',legacy.T2,'loadj',legacy.loadj, ...
        'loadi',legacy.loadi,'loadii',legacy.loadii, ...
        'working_clearance',legacy.working_clearance,'kk',legacy.kk, ...
        'returndata',legacy.returndata,'physics_hash',legacy.physics_hash, ...
        'slice_load_outer',legacy.slice_load_outer, ...
        'slice_load_inner',legacy.slice_load_inner);
    save_output(control.output_state_path,worker);
    write_status(control.status_json_path,struct('success',logical(worker.success), ...
        'qiujieall_call_count',worker.qiujieall_call_count,'loadj',worker.loadj, ...
        'roller_id',reshape(worker.loadi,1,[]),'failure_reason',''));
catch exception
    failure = struct('success',false,'failure_reason',getReport(exception,'basic','hyperlinks','off'));
    save(control.output_state_path,'failure','-v7');
    write_status(control.status_json_path,struct('success',false, ...
        'qiujieall_call_count',0,'loadj',0,'roller_id',[], ...
        'failure_reason',failure.failure_reason));
end
end

function save_output(pathName,worker)
folder = fileparts(pathName);
if ~isempty(folder) && ~isfolder(folder), mkdir(folder); end
partial = [pathName '.partial'];
save(partial,'worker','-v7');
movefile(partial,pathName,'f');
end
function write_status(pathName,status)
folder = fileparts(pathName);
if ~isempty(folder) && ~isfolder(folder), mkdir(folder); end
partial = [pathName '.partial'];
fid = fopen(partial,'w');
assert(fid >= 0,'stage3c_off_worker:Status','Cannot write status JSON.');
cleanup = onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(status));
clear cleanup
movefile(partial,pathName,'f');
end
