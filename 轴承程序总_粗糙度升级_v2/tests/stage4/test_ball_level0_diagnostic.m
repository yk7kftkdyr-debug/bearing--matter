function test_ball_level0_diagnostic(projectRoot)
if nargin<1, projectRoot=fileparts(fileparts(fileparts(mfilename('fullpath')))); end
addpath(fullfile(projectRoot,'roughness')); addpath(fullfile(projectRoot,'球轴承程序'));
p=struct('RqSurface1',0,'RqSurface2',0,'Ered',2e11,'nominalArea',1e-6,'etaAsperity',1e10,'betaAsperity',1e-6,'C_GT',1,'muFluid',.05,'muBoundary',.12);
legacy=struct('Q',1000,'delta',1e-4,'h',0,'roughPair',p); cfg=struct('roughness',struct('feedback_level',0));
d0=ball_roughness_diagnostic(legacy,cfg); assert(abs(d0.Qfluid-legacy.Q)/legacy.Q<1e-6 && d0.Qasperity==0 && isinf(d0.lambda));
p.RqSurface1=1e-9; legacy.roughPair=p; d1=ball_roughness_diagnostic(legacy,cfg); assert(d1.lambda<d0.lambda && d1.chiA>=d0.chiA && d1.Qasperity>=d0.Qasperity);
assert(all(isfinite([d1.h d1.Qfluid d1.Qasperity d1.chiA d1.muMix d1.kEq])) && isreal(d1.lambda));
end
