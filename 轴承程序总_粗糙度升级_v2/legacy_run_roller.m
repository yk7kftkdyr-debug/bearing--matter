function [state, report] = legacy_run_roller(caseInput)
%LEGACY_RUN_ROLLER Isolated compatibility call to the unmodified roller solver.
[state, report] = runLegacyRoller(caseInput);
end

function [state, report] = runLegacyRoller(c)
required={'case_id','input_file','output_dir','rng_seed'};
for k=1:numel(required)
    assert(isfield(c,required{k}),'legacy_run_roller:Input','Missing %s',required{k});
end
assert(isfile(c.input_file),'legacy_run_roller:Input','input_file does not exist');

root=fileparts(mfilename('fullpath'));
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
    'contact_load_inner',pick(g,'Q1'),'contact_load_outer',pick(g,'Q2'), ...
    'oil_film_inner',pick(g,'oilh1'),'oil_film_outer',pick(g,'oilh2'), ...
    'pressure_indicator_inner',pick(g,'Ph1'),'pressure_indicator_outer',pick(g,'Ph2'), ...
    'working_clearance',pick(g,'deltaw'),'loaded_element_count',r.loadj, ...
    'stiffness',pick(g,'kk'),'slip_indicator',pick(g,'roller_slip_speed_check'), ...
    'legacy_metric',r.result333.final,'returndata',r.returndata, ...
    'diagnostics',struct('minresult111',pick(g,'kk','minresult111'),'result333',r.result333), ...
    'raw_result_file',raw);
if isfield(c,'roughness') && c.roughness.enabled
    assert(strcmp(c.roughness.mode,'diagnostic'),'legacy_run_roller:RoughnessMode','Only diagnostic roughness mode is permitted.');
    state.roughness=roller_roughness_shadow(state,tractionAlignedRaw(fullfile(c.output_dir,'滚子轴承程序')),c.roughness);
end
physical=rmfield(state,{'bearing_type','case_id','legacy_metric','diagnostics','raw_result_file'});
report=struct('success',r.success,'status',status(r),'case_id',c.case_id, ...
    'runtime',toc(t),'warning_count',double(~isempty(r.warning)), ...
    'nan_count',count(physical,@isnan),'inf_count',count(physical,@isinf), ...
    'complex_count',count(physical,@(x) ~isreal(x)), ...
    'diagnostic_nonfinite_count',count(state.diagnostics,@(x) ~isfinite(x)), ...
    'message',r.warning);
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
