function test_ball_roughness_shadow()
%TEST_BALL_ROUGHNESS_SHADOW Diagnostic-only ball contact roughness checks.
p=fileparts(fileparts(fileparts(mfilename('fullpath')))); addpath(fullfile(p,'roughness'));
raw=struct('Q1',[10;20],'Q2',[12;22],'oilh1',[1e-8;2e-8],'oilh2',[1.5e-8;2.5e-8], ...
 'E1',1e11,'E2',1.1e11,'aa1',[2e-5;2e-5],'b1',[1e-5;1e-5],'aa2',[2e-5;2e-5],'b2',[1e-5;1e-5],'miuI',[0.05;0.05],'miuO',[0.06;0.06]);
c=config(); c.Rq_inner=0;c.Rq_outer=0;c.Rq_element=0;s=ball_roughness_shadow(struct(),raw,c);
assert(all(s.inner.chiA==0)&all(s.outer.chiA==0)&all(isinf(s.inner.lambda))&all(s.inner.Qasperity==0));
low=config(); low.Rq_inner=5e-9;low.Rq_outer=5e-9;low.Rq_element=5e-9; a=ball_roughness_shadow(struct(),raw,low);
high=low;high.Rq_inner=20e-9;high.Rq_outer=20e-9;high.Rq_element=20e-9; b=ball_roughness_shadow(struct(),raw,high);
assert(mean(b.inner.lambda)<=mean(a.inner.lambda)&&mean(b.inner.chiA)>=mean(a.inner.chiA));
assert(all(isfinite([a.inner.Qfluid;a.inner.Qasperity;a.inner.chiA;a.inner.muMix;a.inner.kAsperity;a.outer.Qfluid;a.outer.Qasperity])));
end
function c=config(),c=struct('enabled',true,'mode','diagnostic','Rq_inner',[],'Rq_outer',[],'Rq_element',[],'inner_pair',struct('etaAsperity',1e10,'betaAsperity',1e-6,'C_GT',1e-4,'muBoundary',0.15),'outer_pair',struct('etaAsperity',1e10,'betaAsperity',1e-6,'C_GT',1e-4,'muBoundary',0.15));end
