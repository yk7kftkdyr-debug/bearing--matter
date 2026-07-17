function roughState = roller_roughness_shadow(rollerState,rawData,roughConfig)
%ROLLER_ROUGHNESS_SHADOW One GT diagnostic per roller and contact side.
if ~roughConfig.enabled || ~strcmp(roughConfig.mode,'diagnostic'),roughState=struct();return;end
roughState=struct('inner',side(rawData.Q1,rawData.oilh1,rawData.E1,rawData.lengthcontact1,rawData.widthcontact1,rawData.miuI,roughConfig.Rq_inner,roughConfig.Rq_element,roughConfig.inner_pair), ...
 'outer',side(rawData.Q2,rawData.oilh2,rawData.E2,rawData.lengthcontact2,rawData.widthcontact2,rawData.miuO,roughConfig.Rq_outer,roughConfig.Rq_element,roughConfig.outer_pair));
source='legacy_T_over_Q'; if isfield(rawData,'lubricationtype')&&rawData.lubricationtype~=0,source='legacy_effective_T_over_Q';end
roughState.inner.muFluidSource=source; roughState.outer.muFluidSource=source;
end
function s=side(Q,h,E,L,w,mu,Rr,Re,p)
req={'etaAsperity','betaAsperity','C_GT','muBoundary'};for k=1:numel(req),assert(isfield(p,req{k})&&~isempty(p.(req{k})),'roller_roughness_shadow:Config','Missing pair.%s',req{k});end
keys={'sigmaComposite','lambda','Qfluid','Qasperity','chiA','muMix','kAsperity'};for k=1:numel(keys),s.(keys{k})=zeros(size(Q));end
for i=1:numel(Q),in=struct('Qtotal',Q(i),'hSmooth',h(i),'RqSurface1',Rr,'RqSurface2',Re,'Ered',E,'nominalArea',L(i)*w(i),'etaAsperity',p.etaAsperity,'betaAsperity',p.betaAsperity,'C_GT',p.C_GT,'muFluid',mu(i),'muBoundary',p.muBoundary);[x,r]=roughness_contact_diagnostic(in);assert(r.success);for k=1:numel(keys),s.(keys{k})(i)=x.(keys{k});end,end
s.muFluid=mu;
end
