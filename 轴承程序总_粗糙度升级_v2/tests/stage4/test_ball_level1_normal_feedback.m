function test_ball_level1_normal_feedback(projectRoot)
if nargin<1, projectRoot=fileparts(fileparts(fileparts(mfilename('fullpath')))); end
addpath(fullfile(projectRoot,'球轴承程序'));
qLegacy=[100 250 700]; qCandidate=[120 200 900];
cfg=struct('roughness',struct('enabled',false,'feedback_level',0));
[q0,s0]=ball_roughness_normal_feedback(qLegacy,qCandidate,cfg);
assert(isequal(q0,qLegacy) && s0.gamma==0 && ~s0.feedback_active);
cfg.roughness.enabled=true; cfg.roughness.feedback_level=1;
[q1,s1]=ball_roughness_normal_feedback(qLegacy,qCandidate,cfg);
assert(isequal(q1,qLegacy) && s1.gamma==0 && ~s1.feedback_active);
cfg.roughness.normal_feedback_gamma=0.25;
failed=false; try, ball_roughness_normal_feedback(qLegacy,qCandidate,cfg); catch, failed=true; end
assert(failed,'Stage 4B-2A must reject nonzero normal-feedback gamma.');
ff=fileread(fullfile(projectRoot,'球轴承程序','ffLOAD.m'));
jf=fileread(fullfile(projectRoot,'球轴承程序','JffLOAD.m'));
needle='ball_roughness_normal_feedback';
assert(contains(ff,needle) && contains(jf,needle));
assert(all(isfinite(q0)) && isreal(q0) && all(q0>=0));
disp('STAGE4B_LEVEL1_NORMAL_FEEDBACK_INTERFACE_PASS');
end
