function test_ball_level1_normal_feedback(projectRoot)
if nargin<1, projectRoot=fileparts(fileparts(fileparts(mfilename('fullpath')))); end
addpath(fullfile(projectRoot,'roughness')); addpath(fullfile(projectRoot,'球轴承程序'));

snapshot=fixture(); original=snapshot;

% Feedback OFF and Level 0 retain the exact legacy normal path.
cfg=config(false,0,0.75);
[qOff,sOff]=ball_roughness_normal_feedback(snapshot,cfg);
assert(isequal(qOff.inner,snapshot.legacyQInner) && isequal(qOff.outer,snapshot.legacyQOuter));
assert(~sOff.feedback_active && isequaln(snapshot,original));
cfg.roughness.enabled=true;
[qLevel0,sLevel0]=ball_roughness_normal_feedback(snapshot,cfg);
assert(isequal(qLevel0,qOff) && ~sLevel0.feedback_active && isequaln(snapshot,original));

% Reference candidate proves Qrough comes from the validated candidate path.
[candidate,candidateReport]=ball_roughness_normal_candidate(snapshot);
assert(candidateReport.converged && any(candidate.inner.Qasperity>0) && any(candidate.outer.Qasperity>0));

gammas=[0 0.25 0.50 0.75 1];
ffStates=cell(size(gammas)); jfStates=cell(size(gammas)); residuals=zeros(size(gammas));
for k=1:numel(gammas)
    cfg=config(true,1,gammas(k));
    [qF,ffStates{k}]=ball_roughness_normal_feedback(snapshot,cfg);
    [qJ,jfStates{k}]=ball_roughness_normal_feedback(snapshot,cfg);
    expectedInner=(1-gammas(k))*snapshot.legacyQInner+gammas(k)*candidate.inner.Q;
    expectedOuter=(1-gammas(k))*snapshot.legacyQOuter+gammas(k)*candidate.outer.Q;
    assert(max(abs(qF.inner-expectedInner))<1e-10 && max(abs(qF.outer-expectedOuter))<1e-10);
    assert(isequaln(qF,qJ) && isequaln(ffStates{k},jfStates{k}));
    assert(isequal(ffStates{k}.inner.Qrough,candidate.inner.Q));
    assert(isequal(ffStates{k}.outer.Qrough,candidate.outer.Q));
    assert(isequal(ffStates{k}.elementIds,snapshot.loadedElementIds));
    assert(ffStates{k}.gamma==gammas(k) && jfStates{k}.gamma==gammas(k));
    assert_state_safe(ffStates{k});
    residuals(k)=normal_residual(qF,snapshot);
end

% gamma=0 is exactly legacy; gamma=1 is exactly the candidate state.
assert(isequal(ffStates{1}.inner.Qgamma,snapshot.legacyQInner));
assert(isequal(ffStates{1}.outer.Qgamma,snapshot.legacyQOuter));
assert(isequal(ffStates{end}.inner.Qgamma,candidate.inner.Q));
assert(isequal(ffStates{end}.outer.Qgamma,candidate.outer.Q));

% Five-point path is continuous and leaves the immutable mechanical state unchanged.
assert(all(isfinite(residuals)) && isreal(residuals));
assert(all(abs(diff(residuals,2))<1e-8));
assert(isequaln(snapshot,original));

% ffLOAD and JffLOAD must consume the same unified Qgamma state.
ff=fileread(fullfile(projectRoot,'球轴承程序','ffLOAD.m'));
jf=fileread(fullfile(projectRoot,'球轴承程序','JffLOAD.m'));
call='ball_roughness_normal_feedback(feedback_snapshot,micro_config)';
assert(contains(ff,call) && contains(jf,call));
assert(contains(ff,'Q1(i)=feedback_load.outer') && contains(jf,'Q1(i)=feedback_load.outer'));
assert(contains(ff,'Q2(i)=feedback_load.inner') && contains(jf,'Q2(i)=feedback_load.inner'));
assert(~contains(ff,'ball_roughness_normal_feedback(Q1(i),Q1(i)'));
assert(~contains(jf,'ball_roughness_normal_feedback(Q1(i),Q1(i)'));
assert(contains(ff,'Q1traction=Q1legacy') && contains(jf,'Q1traction=Q1legacy'));
assert(contains(ff,'Q2traction=Q2legacy') && contains(jf,'Q2traction=Q2legacy'));
assert(contains(ff,'fs1(i)=miuI(i)*Q1traction') && contains(jf,'fs1(i)=miuI(i)*Q1traction'));
assert(contains(ff,'fs2(i)=miuO(i)*Q2traction') && contains(jf,'fs2(i)=miuO(i)*Q2traction'));

% Invalid gamma is rejected, never clipped.
bad={-0.1,1.1,NaN};
for k=1:numel(bad)
    cfg=config(true,1,bad{k}); failed=false;
    try, ball_roughness_normal_feedback(snapshot,cfg); catch, failed=true; end
    assert(failed,'Invalid normal-feedback gamma was not rejected.');
end

disp('STAGE4B_2B_GAMMA_PATH_PASS');
disp('GO_TO_STAGE4B_2C_NEWTON_CONTINUATION');
end

function cfg=config(enabled,level,gamma)
cfg=struct('roughness',struct('enabled',enabled,'feedback_level',level, ...
    'normal_feedback_gamma',gamma));
end

function s=fixture()
outer=pair(2e-8,1e-8,2.0e11,1.2e-6);
inner=pair(3e-8,1e-8,2.1e11,1.0e-6);
s=struct('loadedElementIds',[2 5], ...
    'deltaGeomInner',[1e-4 1.1e-4],'deltaGeomOuter',[9e-5 1.05e-4], ...
    'legacyQInner',[1000 1200],'legacyQOuter',[850 1100], ...
    'innerPointGeometry',struct('K',[1000/(1e-4)^1.5 1200/(1.1e-4)^1.5]), ...
    'outerPointGeometry',struct('K',[850/(9e-5)^1.5 1100/(1.05e-4)^1.5]), ...
    'innerLubricant',struct('h',[0 0]),'outerLubricant',struct('h',[0 0]), ...
    'innerRoughPair',inner,'outerRoughPair',outer, ...
    'contactAngle',struct('inner',[.20 .21],'outer',[.18 .19]), ...
    'tractionState',struct('T1',[1 2],'T2',[3 4]), ...
    'speedState',struct('U1',[10 11],'U2',[12 13]));
end

function p=pair(rq1,rq2,E,area)
p=struct('RqSurface1',rq1,'RqSurface2',rq2,'Ered',E,'nominalArea',area, ...
    'etaAsperity',1e10,'betaAsperity',1e-6,'C_GT',1,'muFluid',.05,'muBoundary',.12);
end

function value=normal_residual(q,s)
value=sum(-q.outer.*sin(s.contactAngle.outer)+q.inner.*sin(s.contactAngle.inner) ...
    +s.tractionState.T1-s.tractionState.T2);
end

function assert_state_safe(s)
assert(strcmp(s.status,'STAGE4B_2B_GAMMA_PATH_PASS') && s.feedback_active);
assert(isequal(fieldnames(s.inner),{'Qlegacy';'Qrough';'Qgamma';'h';'lambda';'Qfluid';'Qasperity';'chiA'}));
assert(isequal(fieldnames(s.outer),fieldnames(s.inner)));
for side={'inner','outer'}
    x=s.(side{1}); finite=[x.Qlegacy x.Qrough x.Qgamma x.h x.Qfluid x.Qasperity x.chiA];
    assert(all(isfinite(finite)) && isreal(finite) && all(x.Qgamma>=0));
    assert(all(~isnan(x.lambda)) && isreal(x.lambda));
end
end
