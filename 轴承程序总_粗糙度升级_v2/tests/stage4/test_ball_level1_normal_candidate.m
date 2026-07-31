function test_ball_level1_normal_candidate(projectRoot)
if nargin<1, projectRoot=fileparts(fileparts(fileparts(mfilename('fullpath')))); end
addpath(fullfile(projectRoot,'roughness')); addpath(fullfile(projectRoot,'球轴承程序'));
s=fixture(); original=s;
[smooth,r0]=ball_roughness_normal_candidate(s);
assert(r0.converged && strcmp(r0.status,'STAGE4B_LEVEL1_CANDIDATE_PASS'));
assert(max(abs(smooth.inner.Q-s.legacyQInner)./s.legacyQInner)<1e-6);
assert(max(abs(smooth.outer.Q-s.legacyQOuter)./s.legacyQOuter)<1e-6);
assert(all(smooth.inner.Qasperity==0) && all(smooth.outer.Qasperity==0));
assert(all(isinf(smooth.inner.lambda)) && all(isinf(smooth.outer.lambda)));
assert(isequaln(s,original));

inner=s; inner.innerRoughPair.RqSurface1=1e-9;
[ci,ri]=ball_roughness_normal_candidate(inner); assert(ri.converged);
assert(isequaln(ci.outer,smooth.outer));
assert(all(ci.inner.lambda<smooth.inner.lambda) && all(ci.inner.Qasperity>=smooth.inner.Qasperity) && all(ci.inner.chiA>=smooth.inner.chiA));

outer=s; outer.outerRoughPair.RqSurface1=1e-9;
[co,ro]=ball_roughness_normal_candidate(outer); assert(ro.converged);
assert(isequaln(co.inner,smooth.inner));
assert(all(co.outer.lambda<smooth.outer.lambda) && all(co.outer.Qasperity>=smooth.outer.Qasperity) && all(co.outer.chiA>=smooth.outer.chiA));
check_side(ci.inner); check_side(ci.outer); check_side(co.inner); check_side(co.outer);
disp('STAGE4B_LEVEL1_CANDIDATE_PASS');
end

function s=fixture()
p=struct('RqSurface1',0,'RqSurface2',0,'Ered',2e11,'nominalArea',1e-6,'etaAsperity',1e10,'betaAsperity',1e-6,'C_GT',1,'muFluid',.05,'muBoundary',.12);
s=struct('loadedElementIds',[2 5],'deltaGeomInner',[1e-4 1.1e-4],'deltaGeomOuter',[9e-5 1.05e-4], ...
    'legacyQInner',[1000 1200],'legacyQOuter',[850 1100], ...
    'innerPointGeometry',struct('K',[1e9 1200/(1.1e-4)^1.5]), ...
    'outerPointGeometry',struct('K',[850/(9e-5)^1.5 1100/(1.05e-4)^1.5]), ...
    'innerLubricant',struct('h',[0 0]),'outerLubricant',struct('h',[0 0]), ...
    'innerRoughPair',p,'outerRoughPair',p,'contactAngle',[.2 .21], ...
    'tractionState',struct('T1',[1 2],'T2',[3 4]));
end
function check_side(x)
v=[x.Q(:);x.h(:);x.Qfluid(:);x.Qasperity(:);x.chiA(:)];
assert(all(isfinite(v)) && isreal(v) && all(x.Q>=0) && all(x.Qfluid>=0) && all(x.Qasperity>=0));
assert(all(isreal(x.lambda)) && all(~isnan(x.lambda)));
end
