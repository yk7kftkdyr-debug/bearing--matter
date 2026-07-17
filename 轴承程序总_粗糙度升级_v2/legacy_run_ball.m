function [state, report] = legacy_run_ball(caseInput)
%LEGACY_RUN_BALL Isolated compatibility call to the unmodified ball solver.
[state, report] = runLegacy(caseInput, 'ball');
end

function [state, report] = runLegacy(c, kind)
required={'case_id','input_file','output_dir','rng_seed'}; for k=1:numel(required), assert(isfield(c,required{k}),'legacy_run_ball:Input','Missing %s',required{k}); end
assert(isfile(c.input_file),'legacy_run_ball:Input','input_file does not exist');
root=fileparts(mfilename('fullpath')); if isfolder(c.output_dir), rmdir(c.output_dir,'s'); end; copyfile(root,c.output_dir); raw=fullfile(c.output_dir,'raw_result.mat');
old=pwd; cleanup=onCleanup(@() cd(old)); addpath(fullfile(root,'tests','baseline_legacy')); rng(c.rng_seed,'twister'); t=tic;
r=run_stage0_legacy_case(c.output_dir,kind,1,1,raw); g=r.generated; state=struct('bearing_type',kind,'case_id',c.case_id,'contact_load_inner',pick(g,'Q1'),'contact_load_outer',pick(g,'Q2'),'oil_film_inner',pick(g,'oilh1'),'oil_film_outer',pick(g,'oilh2'),'contact_angle',[],'working_clearance',pick(g,'deltaw'),'loaded_element_count',r.loadj,'stiffness',pick(g,'kk'),'legacy_metric',r.result333.final,'returndata',r.returndata,'raw_result_file',raw);
if strcmp(kind,'ball') && isfield(c,'roughness') && c.roughness.enabled
    assert(strcmp(c.roughness.mode,'diagnostic'),'legacy_run_ball:RoughnessMode','Only diagnostic roughness mode is permitted.');
    contactFile=fullfile(c.output_dir,'球轴承程序','q1q2a1a2.mat');
    assert(isfile(contactFile),'legacy_run_ball:RoughnessData','Missing saved ball contact results.');
    state.roughness=ball_roughness_shadow(state,load(contactFile),c.roughness);
end
report=struct('success',r.success,'status',status(r),'case_id',c.case_id,'runtime',toc(t),'warning_count',double(~isempty(r.warning)),'nan_count',count(state,@isnan),'inf_count',count(state,@isinf),'complex_count',count(state,@(x)~isreal(x)),'message',r.warning);
end
function x=pick(g,n),x=[];if isfield(g,n),s=g.(n);if isfield(s,n),x=s.(n);else,f=fieldnames(s);if ~isempty(f),x=s.(f{1});end,end,end,end
function n=count(s,fn),n=0;f=fieldnames(s);for i=1:numel(f),if isnumeric(s.(f{i})),n=n+sum(fn(s.(f{i})(:)));end,end,end
function s=status(r),if ~r.success,s='LEGACY_FAILED';elseif ~isempty(r.warning),s='LEGACY_SUCCESS_WITH_WARNING';else,s='LEGACY_SUCCESS';end,end
