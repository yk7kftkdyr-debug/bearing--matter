function varargout = legacy_run_roller(varargin)
%LEGACY_RUN_ROLLER One-call adapter for the legacy cylindrical-roller solver.
% New Stage 2C interface: result = legacy_run_roller(run_dir,micro_config,restart_state).
% The historical one-input wrapper is retained only for Stage 0/1 callers.
if nargin == 3
    varargout{1} = one_legacy_solve(varargin{1},varargin{2},varargin{3});
    return;
end
assert(nargin == 1 && nargout == 2, ...
    'legacy_run_roller:Interface','Use either legacy_run_roller(caseInput) or legacy_run_roller(run_dir,micro_config,restart_state).');
[varargout{1},varargout{2}] = runLegacyRoller(varargin{1});
end

function result = one_legacy_solve(runDir,microConfig,restartState)
assert(ischar(runDir) || isstring(runDir),'legacy_run_roller:RunDir','run_dir must be text.');
runDir = char(runDir);
assert(isfolder(runDir),'legacy_run_roller:RunDir','run_dir does not exist: %s',runDir);
assert(isstruct(microConfig) && isfield(microConfig,'roughness'), ...
    'legacy_run_roller:Config','micro_config.roughness is required.');
assert(isstruct(restartState),'legacy_run_roller:Restart','restart_state must be a struct.');

oldDir = pwd;
cleanup = onCleanup(@()cd(oldDir)); %#ok<NASGU>
cd(runDir);
micro_config = microConfig;
save('micro_config_runtime.mat','micro_config');
% The legacy solver has no supported restart-file input.  Keep the restart
% state in the returned adapter result; do not write an unused pseudo-restart.
callCount = 1;
[loadj,returndata] = qiujieall(defaultBearingInput(),microConfig);
assert(callCount == 1,'legacy_run_roller:CallCount','qiujieall must be called exactly once.');

q1 = load_required('Q1.mat',{'Q1','E1','E2'});
q2 = load_required('Q2.mat',{'Q2'});
h1 = load_required('oilh1.mat',{'oilh1'});
h2 = load_required('oilh2.mat',{'oilh2'});
ids = load_required('loadi.mat',{'loadi'});
idsAll = load_required('loadii.mat',{'loadii'});
kk = load_required('kk.mat',{'kk'});
slip = load_required('roller_slip_speed_check.mat',{'Wc'});
traction = tractionAlignedRaw(runDir);
terms = struct();
if microConfig.roughness.enabled && microConfig.roughness.feedback_level == 1
    terms = load_required('roller_feedback_terms.mat', ...
        {'loadi','Q1','Q2','oilh1_legacy','oilh2_legacy','oilh1_used','oilh2_used', ...
         'widthcontact11','widthcontact22','lenroller'});
end
width1 = load_required('widthcontact1.mat',{'widthcontact1'});
width2 = load_required('widthcontact2.mat',{'widthcontact2'});
length1 = load_required('lengthcontact1.mat',{'lengthcontact1'});
length2 = load_required('lengthcontact2.mat',{'lengthcontact2'});
sliceLoad1 = load_required('QQ1.mat',{'QQ1'});
sliceLoad2 = load_required('QQ2.mat',{'QQ2'});
[deltaw,~] = calcWorkingClearance(defaultBearingInput());

result = struct('converged',true,'Q1',q1.Q1,'Q2',q2.Q2, ...
    'oilh1',h1.oilh1,'oilh2',h2.oilh2,'T1',traction.T1,'T2',traction.T2, ...
    'loadj',loadj,'loadi',ids.loadi(:),'loadii',idsAll.loadii(:), ...
    'widthcontact1',width1.widthcontact1,'widthcontact2',width2.widthcontact2, ...
    'lengthcontact1',length1.lengthcontact1,'lengthcontact2',length2.lengthcontact2, ...
    'slice_load_outer',sliceLoad1.QQ1,'slice_load_inner',sliceLoad2.QQ2, ...
    'working_clearance',deltaw,'kk',kk.kk,'returndata',returndata, ...
    'slip_indicator',slip.Wc,'E1',q1.E1,'E2',q1.E2,'traction',traction, ...
    'frozen_terms',terms,'restart_state',restartState,'qiujieall_call_count',callCount);
result.physics_hash = physics_hash(result);
end

function d = load_required(file,fields)
assert(isfile(file),'legacy_run_roller:MissingOutput','Missing legacy output %s.',file);
d = load(file);
assert(all(isfield(d,fields)),'legacy_run_roller:MissingOutput','Missing fields in %s.',file);
end

function [state, report] = runLegacyRoller(c)
required={'case_id','input_file','output_dir','rng_seed'};
for k=1:numel(required)
    assert(isfield(c,required{k}),'legacy_run_roller:Input','Missing %s',required{k});
end
assert(isfile(c.input_file),'legacy_run_roller:Input','input_file does not exist');

root=fileparts(mfilename('fullpath'));
rough = roughness_config(c);
if rough.enabled && rough.feedback_level == 1
    error('legacy_run_roller:FeedbackInterface', ...
        'Stage 2C feedback must use legacy_run_roller(run_dir,micro_config,restart_state).');
end
if isfolder(c.output_dir), rmdir(c.output_dir,'s'); end
copyfile(root,c.output_dir);
raw=fullfile(c.output_dir,'raw_result.mat');
old=pwd; cleanup=onCleanup(@() cd(old)); %#ok<NASGU>
addpath(fullfile(root,'tests','baseline_legacy'));
rng(c.rng_seed,'twister');
t=tic;
r=run_stage0_legacy_case(c.output_dir,'roller',1,1,raw);
g=r.generated;
% minresult111 is an iteration-stagnation sentinel, not part of kk or returndata.
state=struct('bearing_type','roller','case_id',c.case_id, ...
    'contact_load_inner',pick(g,'Q2'),'contact_load_outer',pick(g,'Q1'), ...
    'oil_film_inner',pick(g,'oilh2'),'oil_film_outer',pick(g,'oilh1'), ...
    'pressure_indicator_inner',pick(g,'Ph1'),'pressure_indicator_outer',pick(g,'Ph2'), ...
    'working_clearance',pick(g,'deltaw'),'loaded_element_count',r.loadj, ...
    'stiffness',pick(g,'kk'),'slip_indicator',pick(g,'roller_slip_speed_check'), ...
    'legacy_metric',r.result333.final,'returndata',r.returndata, ...
    'diagnostics',struct('minresult111',pick(g,'kk','minresult111'),'result333',r.result333), ...
    'raw_result_file',raw);
state.converged=r.success; state.Q1=pick(g,'Q1'); state.Q2=pick(g,'Q2');
state.oilh1=pick(g,'oilh1'); state.oilh2=pick(g,'oilh2'); state.loadj=r.loadj;
state.outer_loaded_roller_ids=[]; state.inner_loaded_roller_ids=[]; state.kk=pick(g,'kk');
state.physics_hash=physics_hash(state); state.mechanical_residual=[]; state.original_balance_status=r.success;
if rough.enabled
    assert(rough.feedback_level == 0 && strcmp(rough.mode,'diagnostic'), ...
        'legacy_run_roller:RoughnessMode','Only diagnostic shadow mode is permitted for feedback_level zero.');
    state.roughness=roller_roughness_shadow(state,tractionAlignedRaw(fullfile(c.output_dir,'滚子轴承程序')),rough);
end

physical=rmfield(state,{'bearing_type','case_id','legacy_metric','diagnostics','raw_result_file'});
report=struct('success',r.success,'status',status(r),'case_id',c.case_id, ...
    'runtime',toc(t),'warning_count',double(~isempty(r.warning)), ...
    'nan_count',count(physical,@isnan),'inf_count',count(physical,@isinf), ...
    'complex_count',count(physical,@(x) ~isreal(x)), ...
    'diagnostic_nonfinite_count',count(state.diagnostics,@(x) ~isfinite(x)), ...
    'message',r.warning);
end

function rough = roughness_config(c)
rough = struct('enabled',false,'mode','off','feedback_level',0);
if isfield(c,'roughness') && ~isempty(c.roughness)
    rough = c.roughness;
end
assert(isfield(rough,'enabled') && isfield(rough,'feedback_level'), ...
    'legacy_run_roller:RoughnessConfig','roughness must provide enabled and feedback_level.');
assert(ismember(rough.feedback_level,[0 1]), ...
    'legacy_run_roller:RoughnessConfig','feedback_level must be 0 or 1.');
end

function digest = physics_hash(state)
fields={'Q1','Q2','oilh1','oilh2','working_clearance','kk','returndata'};
v=[];
for k=1:numel(fields)
    if isfield(state,fields{k}) && isnumeric(state.(fields{k})), v=[v;double(state.(fields{k})(:))]; end %#ok<AGROW>
end
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(typecast(v(:),'uint8'));
digest=lower(reshape(dec2hex(typecast(md.digest,'uint8'),2).',1,[]));
end
function q=tractionAlignedRaw(wd)
files={'Q1','Q2','oilh1','oilh2','lengthcontact1','lengthcontact2','widthcontact1','widthcontact2','T1','T2','Q1ii','Q2ii','loadii','loadi'};
for k=1:numel(files), f=fullfile(wd,[files{k} '.mat']); assert(isfile(f),'legacy_run_roller:MissingTraction','Missing %s',f); d=load(f); if k==1,q=d;else,q.(files{k})=d.(files{k});end,end
[found,loc]=ismember(q.loadi(:),q.loadii(:)); assert(all(found)&&numel(unique(q.loadii))==numel(q.loadii),'legacy_run_roller:RollerMap','Invalid loadi/loadii mapping.');
qo=q.Q1ii(loc); qi=q.Q2ii(loc); scale=max([abs(qo(:));abs(qi(:));1]); tol=1e-12*scale; assert(all(qo>tol)&&all(qi>tol),'legacy_run_roller:TractionLoad','Nonpositive traction load.');
q.miuO=q.T1(loc)./qo; q.miuI=q.T2(loc)./qi; assert(all(isfinite(q.miuO)&q.miuO>=0)&all(isfinite(q.miuI)&q.miuI>=0),'legacy_run_roller:Traction','Invalid legacy T/Q traction.');
end

function x=pick(g,n,varargin)
x=[];
if isfield(g,n)
    s=g.(n); target=n; if ~isempty(varargin), target=varargin{1}; end
    if isfield(s,target), x=s.(target); else, f=fieldnames(s); if ~isempty(f), x=s.(f{1}); end, end
end
end
function n=count(s,fn)
n=0; f=fieldnames(s);
for i=1:numel(f)
    if isnumeric(s.(f{i})), n=n+sum(fn(s.(f{i})(:))); end
end
end
function s=status(r)
if ~r.success, s='LEGACY_FAILED';
elseif ~isempty(r.warning), s='LEGACY_SUCCESS_WITH_WARNING';
else, s='LEGACY_SUCCESS'; end
end
