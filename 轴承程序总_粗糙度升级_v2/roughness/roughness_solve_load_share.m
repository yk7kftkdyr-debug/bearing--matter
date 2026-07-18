function [state,report] = roughness_solve_load_share(input)
%ROUGHNESS_SOLVE_LOAD_SHARE Local self-consistent fluid--asperity sharing.
required={'contactType','Qtotal','filmInput','roughnessInput','muFluid','muBoundary'};
for k=1:numel(required)
    assert(isfield(input,required{k}),'roughness_solve_load_share:Input','Missing %s.',required{k});
end
assert(ischar(input.contactType) || isstring(input.contactType),'roughness_solve_load_share:Input','contactType must be text.');
assert(any(strcmp(char(input.contactType),{'point','line'})),'roughness_solve_load_share:Input','contactType must be point or line.');
validateattributes(input.Qtotal,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(input.muFluid,{'numeric'},{'real','finite','scalar'});
validateattributes(input.muBoundary,{'numeric'},{'real','finite','scalar'});

sigma=roughness_composite_rq(input.roughnessInput.RqSurface1,input.roughnessInput.RqSurface2);
if sigma==0
    h=filmThickness(input.filmInput,input.Qtotal);
    state=makeState(input.Qtotal,input.Qtotal,0,h,Inf,0,input.muFluid,input.muFluid,0,0);
    report=struct('success',true,'status','OK','message','Smooth limit.');
    return;
end

xLower=1e-8; xUpper=1; maxIter=80; tol=1e-8;
[rLower,~]=residual(xLower,input); [rUpper,~]=residual(xUpper,input);
if ~isfinite(rLower) || ~isfinite(rUpper) || rLower*rUpper>0
    state=emptyState(input);
    report=struct('success',false,'status','OUT_OF_MODEL_DOMAIN','message','No valid load-share residual bracket.');
    return;
end
last=[];
for it=1:maxIter
    x=0.5*(xLower+xUpper);
    [r,last]=residual(x,input);
    if ~isfinite(r) || ~isreal(r)
        state=emptyState(input);
        report=struct('success',false,'status','OUT_OF_MODEL_DOMAIN','message','Nonfinite local load-share residual.');
        return;
    end
    if abs(r)<=tol || (xUpper-xLower)<=tol
        break;
    end
    if rLower*r<=0
        xUpper=x; rUpper=r;
    else
        xLower=x; rLower=r;
    end
end
qfluid=x*input.Qtotal; qasp=last.QasperityRaw; chi=qasp/input.Qtotal;
balance=abs(qfluid+qasp-input.Qtotal)/max(input.Qtotal,1);
if ~(qfluid>=0 && qasp>=0 && chi>=0 && chi<=1 && balance<1e-6)
    state=emptyState(input);
    report=struct('success',false,'status','OUT_OF_MODEL_DOMAIN','message','No physical local load-share solution.');
    return;
end
mu=(1-chi)*input.muFluid-chi*input.muBoundary;
state=makeState(input.Qtotal,qfluid,qasp,last.h, last.lambda,chi,input.muFluid,mu,balance,it);
report=struct('success',true,'status','OK','message','Bisection converged.');
end

function [r,d]=residual(x,input)
q=x*input.Qtotal; h=filmThickness(input.filmInput,q);
ri=input.roughnessInput; ri.Qtotal=input.Qtotal; ri.hSmooth=h; ri.muFluid=input.muFluid; ri.muBoundary=input.muBoundary;
[d,dr]=roughness_contact_diagnostic(ri);
assert(dr.success,'roughness_solve_load_share:Diagnostic','Stage 1 roughness diagnostic failed.');
% Conservation Qtotal=Qfluid+Qasperity gives x+Qasperity/Qtotal-1=0.
r=x+d.QasperityRaw/input.Qtotal-1;
d.h=h;
end

function h=filmThickness(spec,q)
if isa(spec,'function_handle')
    h=spec(q);
elseif isstruct(spec) && isfield(spec,'hAtQtotal') && isfield(spec,'loadExponent') && isfield(spec,'Qtotal')
    h=spec.hAtQtotal*(q/spec.Qtotal)^spec.loadExponent;
elseif isstruct(spec) && isfield(spec,'hAtQtotal') && isfield(spec,'loadExponent')
    error('roughness_solve_load_share:FilmInput','filmInput.Qtotal is required for struct filmInput.');
else
    error('roughness_solve_load_share:FilmInput','filmInput must be a function handle or load-law struct.');
end
validateattributes(h,{'numeric'},{'real','finite','scalar','nonnegative'});
end

function s=makeState(qt,qf,qa,h,lambda,chi,muf,mum,balance,it)
s=struct('Qtotal',qt,'Qfluid',qf,'Qasperity',qa,'hMix',h,'lambda',lambda,'chiA',chi,...
    'muFluid',muf,'muMix',mum,'loadBalanceError',balance,'iterations',it);
end
function s=emptyState(input)
s=makeState(input.Qtotal,NaN,NaN,NaN,NaN,NaN,input.muFluid,NaN,NaN,0);
end
