function [state,report] = roughness_contact_diagnostic(input)
%ROUGHNESS_CONTACT_DIAGNOSTIC Standalone GT statistical-contact diagnostic.
required={'Qtotal','hSmooth','RqSurface1','RqSurface2','Ered','nominalArea','etaAsperity','betaAsperity','C_GT','muFluid','muBoundary'};
for k=1:numel(required), assert(isfield(input,required{k}),'roughness_contact_diagnostic:Input','Missing %s',required{k}); end
positive={'Qtotal','Ered','nominalArea','etaAsperity','betaAsperity','C_GT'};
for k=1:numel(positive), validateattributes(input.(positive{k}),{'numeric'},{'real','finite','scalar','positive'}); end
validateattributes(input.hSmooth,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(input.RqSurface1,{'numeric'},{'real','finite','scalar','nonnegative'}); validateattributes(input.RqSurface2,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(input.muFluid,{'numeric'},{'real','finite','scalar'}); validateattributes(input.muBoundary,{'numeric'},{'real','finite','scalar'});
sigma=roughness_composite_rq(input.RqSurface1,input.RqSurface2); tiny=1e-15;
if sigma<tiny
    qraw=0; qasp=0; lambda=Inf; limited=false; kasp=0;
else
    lambda=input.hSmooth/sigma; qraw=rawLoad(input.hSmooth,input,sigma); qasp=min(max(qraw,0),input.Qtotal); limited=qraw>input.Qtotal;
    dh=max(input.hSmooth*1e-6,sigma*1e-8); qp=clampedLoad(input.hSmooth+dh,input,sigma); qm=clampedLoad(max(0,input.hSmooth-dh),input,sigma); kasp=-(qp-qm)/((input.hSmooth+dh)-max(0,input.hSmooth-dh));
end
qfluid=input.Qtotal-qasp; chi=qasp/input.Qtotal; mu=(1-chi)*input.muFluid-chi*input.muBoundary;
state=struct('sigmaComposite',sigma,'lambda',lambda,'Qtotal',input.Qtotal,'Qfluid',qfluid,'Qasperity',qasp,'QasperityRaw',qraw,'chiA',chi,'muMix',mu,'kAsperity',kasp,'asperity_limit_applied',limited);
finite=[sigma input.Qtotal qfluid qasp qraw chi mu kasp]; assert(all(isfinite(finite)&isreal(finite)),'roughness_contact_diagnostic:InvalidOutput','Nonfinite physical diagnostic output.');
report=struct('success',true,'status','OK','message','Standalone diagnostic only; no legacy solver feedback.');
end
function q=rawLoad(h,x,sigma)
q=x.C_GT*x.Ered*x.nominalArea*(x.etaAsperity*x.betaAsperity*sigma)^2*sqrt(sigma/x.betaAsperity)*roughness_gt_integral(h/sigma,2.5);
end
function q=clampedLoad(h,x,sigma), q=min(max(rawLoad(h,x,sigma),0),x.Qtotal); end
