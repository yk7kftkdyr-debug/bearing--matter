function scales = extract_stage0_scales(reportRoot, outputRoot)
%EXTRACT_STAGE0_SCALES Minimal Stage-1 scales from saved legacy outputs.
files=dir(fullfile(reportRoot,'*.mat')); rows={};
for kind={'ball','roller'}
  values=struct('contact_load',[],'oil_film',[],'stiffness',[],'pressure_indicator',[],'slip_indicator',[],'legacy_metric',[],'external_load',[],'moment',[],'rotation',[],'working_clearance',[],'displacement',[]);
  for k=1:numel(files)
    d=load(fullfile(files(k).folder,files(k).name)); if ~isfield(d,'result')||~strcmp(d.result.bearing_type,kind{1}),continue,end
    g=d.result.generated; values.contact_load=[values.contact_load; get(g,{'Q1','Q2'})]; values.oil_film=[values.oil_film;get(g,{'oilh1','oilh2'})]; values.stiffness=[values.stiffness;get(g,{'kk'})]; values.pressure_indicator=[values.pressure_indicator;get(g,{'Ph1','Ph2'})]; values.working_clearance=[values.working_clearance;get(g,{'deltaw'})];
    values.external_load=[values.external_load; d.result.input(:)]; values.legacy_metric=[values.legacy_metric; d.result.result333.final];
  end
  names=fieldnames(values); for j=1:numel(names), x=abs(values.(names{j})); x=x(isfinite(x)); if isempty(x), n=0;med=[];p95=[];rec=[]; else,n=numel(x);med=median(x);p95=prctile(x,95);rec=p95;if isempty(rec)||rec==0,rec=med;end;if isempty(rec)||rec==0,rec=1e-12;end;end; rows(end+1,:)={kind{1},names{j},n,med,p95,rec,'legacy SI'}; end
end
scales=cell2table(rows,'VariableNames',{'bearing_type','quantity','count','median','P95','recommended_scale','unit'}); writetable(scales,fullfile(outputRoot,'stage0_numerical_scales.csv')); save(fullfile(outputRoot,'stage0_numerical_scales.mat'),'scales');
end
function x=get(s,names),x=[];for i=1:numel(names),if isfield(s,names{i}),q=s.(names{i});f=fieldnames(q);for j=1:numel(f),if isnumeric(q.(f{j})),x=[x;q.(f{j})(:)];end,end,end,end,end
