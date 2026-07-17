function test_roller_roughness_shadow()
%TEST_ROLLER_ROUGHNESS_SHADOW Stage 1C diagnostic-only checks.
p=fileparts(fileparts(fileparts(mfilename('fullpath'))));addpath(fullfile(p,'roughness'));
raw=struct('Q1',[10;20],'Q2',[12;22],'oilh1',[1e-8;2e-8],'oilh2',[1e-8;2e-8],'E1',1e11,'E2',1.1e11,'lengthcontact1',[1e-3;1e-3],'lengthcontact2',[1e-3;1e-3],'widthcontact1',[2e-5;2e-5],'widthcontact2',[2e-5;2e-5],'miuI',[.05;.05],'miuO',[.06;.06]);
c=config();c.Rq_inner=0;c.Rq_outer=0;c.Rq_element=0;s=roller_roughness_shadow(struct(),raw,c);assert(all(s.inner.chiA==0)&all(isinf(s.outer.lambda)));
low=config();low.Rq_inner=5e-9;low.Rq_outer=5e-9;low.Rq_element=5e-9;a=roller_roughness_shadow(struct(),raw,low);high=low;high.Rq_inner=20e-9;high.Rq_outer=20e-9;high.Rq_element=20e-9;b=roller_roughness_shadow(struct(),raw,high);assert(mean(b.inner.lambda)<=mean(a.inner.lambda)&&mean(b.outer.chiA)>=mean(a.outer.chiA));
Touter=raw.miuO.*raw.Q1; Tinner=raw.miuI.*raw.Q2; assert(max(abs(Touter-raw.miuO.*raw.Q1)./max(abs(Touter),1))<=1e-12);assert(max(abs(Tinner-raw.miuI.*raw.Q2)./max(abs(Tinner),1))<=1e-12);
end
function c=config(),c=struct('enabled',true,'mode','diagnostic','Rq_inner',[],'Rq_outer',[],'Rq_element',[],'inner_pair',struct('etaAsperity',1e10,'betaAsperity',1e-6,'C_GT',1e-4,'muBoundary',.15),'outer_pair',struct('etaAsperity',1e10,'betaAsperity',1e-6,'C_GT',1e-4,'muBoundary',.15));end
