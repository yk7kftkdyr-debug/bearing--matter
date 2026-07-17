function roughState = ball_roughness_shadow(ballState,rawData,roughConfig)
%BALL_ROUGHNESS_SHADOW Diagnostic-only GT evaluation; does not feed the solver.
if ~roughConfig.enabled || ~strcmp(roughConfig.mode,'diagnostic'), roughState=struct(); return; end
need={'Rq_inner','Rq_outer','Rq_element','inner_pair','outer_pair'}; for k=1:numel(need), assert(isfield(roughConfig,need{k})&&~isempty(roughConfig.(need{k})),'ball_roughness_shadow:Config','Missing roughness.%s',need{k});end
% ffLOAD defines Q1/E1/a1/b1/miuI for inner ring and Q2/E2/a2/b2/miuO for outer.
roughState=struct('inner',evaluate(rawData.Q1,rawData.oilh1,rawData.E1,rawData.aa1,rawData.b1,rawData.miuI,roughConfig.Rq_inner,roughConfig.Rq_element,roughConfig.inner_pair), ...
 'outer',evaluate(rawData.Q2,rawData.oilh2,rawData.E2,rawData.aa2,rawData.b2,rawData.miuO,roughConfig.Rq_outer,roughConfig.Rq_element,roughConfig.outer_pair));
end
function s=evaluate(Q,h,E,a,b,mu,RqRing,RqBall,p)
required={'etaAsperity','betaAsperity','C_GT','muBoundary'};for k=1:numel(required),assert(isfield(p,required{k})&&~isempty(p.(required{k})),'ball_roughness_shadow:Config','Missing pair.%s',required{k});end
n=numel(Q); keys={'sigmaComposite','lambda','Qfluid','Qasperity','chiA','muMix','kAsperity'};for k=1:numel(keys),s.(keys{k})=zeros(size(Q));end
for i=1:n
 in=struct('Qtotal',Q(i),'hSmooth',h(i),'RqSurface1',RqRing,'RqSurface2',RqBall,'Ered',E,'nominalArea',pi*a(i)*b(i),'etaAsperity',p.etaAsperity,'betaAsperity',p.betaAsperity,'C_GT',p.C_GT,'muFluid',mu(i),'muBoundary',p.muBoundary);
 [x,r]=roughness_contact_diagnostic(in); assert(r.success); for k=1:numel(keys),s.(keys{k})(i)=x.(keys{k});end
end
end
