function [state,report] = solve_point_contact_from_gap(deltaGeom,pointGeom,lubricant,roughPair,prevState,opts)
%SOLVE_POINT_CONTACT_FROM_GAP Stage 4A isolated mixed-lubrication point contact.
if nargin < 5 || isempty(prevState), prevState=struct(); end
if nargin < 6, opts=struct(); end
validateattributes(deltaGeom,{'numeric'},{'real','finite','scalar'});
assert(isstruct(pointGeom) && isfield(pointGeom,'K'),'solve_point_contact_from_gap:Input','pointGeom.K is required.');
validateattributes(pointGeom.K,{'numeric'},{'real','finite','scalar','positive'});
if deltaGeom <= 0, [state,report]=no_contact(); return; end
m=1.5; qdry=pointGeom.K*deltaGeom^m;
qmin=max(qdry*1e-8,realmin); qmax=max(qdry*1e3,1);
n=getopt(opts,'scan_points',81); tol=getopt(opts,'root_tol',1e-10); maxit=getopt(opts,'max_iter',80);
q=logspace(log10(qmin),log10(qmax),n); g=nan(size(q)); states=cell(size(q));
for k=1:n
    [states{k},ok]=contact_state(q(k),lubricant,roughPair);
    if ok, g(k)=(q(k)/pointGeom.K)^(1/m)+states{k}.h-deltaGeom; end
end
ix=find(g(1:end-1).*g(2:end)<0);
if isempty(ix), [state,report]=hold_state(); return; end
roots=zeros(size(ix)); rootStates=cell(size(ix)); iterations=0;
for j=1:numel(ix)
    lo=q(ix(j)); hi=q(ix(j)+1); flo=g(ix(j));
    for it=1:maxit
        mid=sqrt(lo*hi); [s,ok]=contact_state(mid,lubricant,roughPair);
        if ~ok, break; end
        fm=(mid/pointGeom.K)^(1/m)+s.h-deltaGeom;
        if abs(fm)<=tol || abs(log(hi/lo))<=tol, break; end
        if flo*fm<=0, hi=mid; else, lo=mid; flo=fm; end
    end
    roots(j)=mid; rootStates{j}=s; iterations=max(iterations,it);
end
if isfield(prevState,'Q') && isfinite(prevState.Q) && prevState.Q>0
    [~,chosen]=min(abs(roots-prevState.Q)); branch='nearest_previous';
else
    [~,chosen]=max(roots); branch='high_load';
end
state=rootStates{chosen}; state.Q=roots(chosen); state.status='CONVERGED_STRICT';
report=struct('status','CONVERGED_STRICT','converged',true,'iterations',iterations, ...
    'residual_norm',abs((state.Q/pointGeom.K)^(1/m)+state.h-deltaGeom), ...
    'load_balance_error',abs(state.Qfluid+state.Qasperity-state.Q)/state.Q, ...
    'selected_branch',branch);
end

function [s,ok]=contact_state(q,lubricant,pair)
try
    assert(isstruct(lubricant) && isfield(lubricant,'filmInput'),'solve_point_contact_from_gap:Input','lubricant.filmInput is required.');
    in=struct('contactType','point','Qtotal',q,'filmInput',lubricant.filmInput, ...
        'roughnessInput',pair);
    if isfield(pair,'muFluid'), in.muFluid=pair.muFluid; else, in.muFluid=0; end
    if isfield(pair,'muBoundary'), in.muBoundary=pair.muBoundary; else, in.muBoundary=0; end
    [x,r]=roughness_solve_load_share(in); ok=r.success;
    s=struct('Q',q,'Qfluid',x.Qfluid,'Qasperity',x.Qasperity,'h',x.hMix, ...
        'lambda',x.lambda,'chiA',x.chiA,'status',ternary(ok,'CONVERGED_STRICT','OUT_OF_MODEL_DOMAIN'));
    ok=ok && isreal(s.h) && isfinite(s.h) && all(isfinite([s.Q s.Qfluid s.Qasperity s.chiA]));
catch, s=struct('Q',NaN,'Qfluid',NaN,'Qasperity',NaN,'h',NaN,'lambda',NaN,'chiA',NaN,'status','OUT_OF_MODEL_DOMAIN'); ok=false; end
end
function v=getopt(s,n,d), if isfield(s,n),v=s.(n);else,v=d;end,end
function v=ternary(c,a,b), if c,v=a;else,v=b;end,end
function [s,r]=no_contact(), s=struct('Q',0,'Qfluid',0,'Qasperity',0,'h',NaN,'lambda',NaN,'chiA',0,'status','NO_CONTACT'); r=struct('status','NO_CONTACT','converged',true,'iterations',0,'residual_norm',0,'load_balance_error',0,'selected_branch','none'); end
function [s,r]=hold_state(), s=struct('Q',NaN,'Qfluid',NaN,'Qasperity',NaN,'h',NaN,'lambda',NaN,'chiA',NaN,'status','OUT_OF_MODEL_DOMAIN'); r=struct('status','OUT_OF_MODEL_DOMAIN','converged',false,'iterations',0,'residual_norm',Inf,'load_balance_error',Inf,'selected_branch','none'); end
