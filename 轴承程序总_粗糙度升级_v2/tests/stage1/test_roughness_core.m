function test_roughness_core()
%TEST_ROUGHNESS_CORE Stage 1A independent statistical roughness checks.
p=fileparts(fileparts(fileparts(mfilename('fullpath')))); addpath(fullfile(p,'roughness'));
base=inputCase();
smooth=base; smooth.RqSurface1=0; smooth.RqSurface2=0;
[s,r]=roughness_contact_diagnostic(smooth);
assert(r.success && isinf(s.lambda) && s.chiA==0 && s.Qasperity==0 && s.Qfluid==s.Qtotal);
assert(roughness_composite_rq(3e-9,4e-9)==5e-9);
rough=base; rough.RqSurface1=5e-9; rough.RqSurface2=5e-9;
[a,ra]=roughness_contact_diagnostic(rough); assert(ra.success);
rough.RqSurface1=10e-9; rough.RqSurface2=10e-9;
[b,rb]=roughness_contact_diagnostic(rough); assert(rb.success && b.lambda<a.lambda && b.chiA>=a.chiA);
rough=base; rough.hSmooth=2*base.hSmooth;
[c,rc]=roughness_contact_diagnostic(rough); assert(rc.success && c.chiA<=a.chiA);
for s={a,b,c}, x=s{1}; assert(abs(x.Qfluid+x.Qasperity-x.Qtotal)/x.Qtotal<1e-10); assert(all(isfinite([x.sigmaComposite x.Qtotal x.Qfluid x.Qasperity x.QasperityRaw x.chiA x.muMix x.kAsperity])) && isreal(x.kAsperity)); end
assert(roughness_gt_integral(2,2)>roughness_gt_integral(3,2));
end
function x=inputCase()
x=struct('Qtotal',100,'hSmooth',1e-8,'RqSurface1',5e-9,'RqSurface2',5e-9,'Ered',1e11,'nominalArea',1e-6,'etaAsperity',1e10,'betaAsperity',1e-6,'C_GT',1e-4,'muFluid',0.05,'muBoundary',0.15); % TEST_ONLY parameters
end
