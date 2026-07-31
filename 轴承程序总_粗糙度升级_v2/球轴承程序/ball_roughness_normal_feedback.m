function [Qused,state] = ball_roughness_normal_feedback(snapshot,cfg)
%BALL_ROUGHNESS_NORMAL_FEEDBACK Deterministic Stage 4B-2B gamma path.
assert(isstruct(snapshot),'ball_roughness_normal_feedback:Input','snapshot must be a struct.');
assert(isstruct(cfg) && isfield(cfg,'roughness') && isstruct(cfg.roughness), ...
    'ball_roughness_normal_feedback:Input','cfg.roughness is required.');

required={'loadedElementIds','legacyQInner','legacyQOuter'};
for k=1:numel(required)
    assert(isfield(snapshot,required{k}),'ball_roughness_normal_feedback:Input', ...
        'Missing snapshot.%s.',required{k});
end
validateattributes(snapshot.legacyQInner,{'numeric'},{'real','finite','nonnegative'});
validateattributes(snapshot.legacyQOuter,{'numeric'},{'real','finite','nonnegative'});
assert(isequal(size(snapshot.legacyQInner),size(snapshot.legacyQOuter)) && ...
    numel(snapshot.loadedElementIds)==numel(snapshot.legacyQInner), ...
    'ball_roughness_normal_feedback:Input','Legacy loads and element ids must align.');

enabled=isfield(cfg.roughness,'enabled') && isscalar(cfg.roughness.enabled) && ...
    logical(cfg.roughness.enabled);
level=0;
if isfield(cfg.roughness,'feedback_level'), level=cfg.roughness.feedback_level; end
active=enabled && isequal(level,1);
if ~active
    Qused=struct('inner',snapshot.legacyQInner,'outer',snapshot.legacyQOuter);
    state=struct('status','LEGACY_PATH','feedback_active',false,'gamma',0, ...
        'elementIds',snapshot.loadedElementIds, ...
        'inner',legacy_side(snapshot.legacyQInner), ...
        'outer',legacy_side(snapshot.legacyQOuter));
    return;
end

gamma=0;
if isfield(cfg.roughness,'normal_feedback_gamma')
    gamma=cfg.roughness.normal_feedback_gamma;
end
validateattributes(gamma,{'numeric'},{'real','finite','scalar','>=',0,'<=',1});

[candidate,report]=ball_roughness_normal_candidate(snapshot);
assert(report.converged && strcmp(report.status,'STAGE4B_LEVEL1_CANDIDATE_PASS'), ...
    'ball_roughness_normal_feedback:CandidateFailure', ...
    'Real rough normal candidate did not converge.');

inner=blend_side(snapshot.legacyQInner,candidate.inner,gamma);
outer=blend_side(snapshot.legacyQOuter,candidate.outer,gamma);
Qused=struct('inner',inner.Qgamma,'outer',outer.Qgamma);
state=struct('status','STAGE4B_2B_GAMMA_PATH_PASS','feedback_active',true, ...
    'gamma',gamma,'elementIds',snapshot.loadedElementIds,'inner',inner,'outer',outer);
end

function side=blend_side(Qlegacy,candidate,gamma)
Qrough=candidate.Q;
Qgamma=(1-gamma).*Qlegacy+gamma.*Qrough;
side=struct('Qlegacy',Qlegacy,'Qrough',Qrough,'Qgamma',Qgamma, ...
    'h',candidate.h,'lambda',candidate.lambda,'Qfluid',candidate.Qfluid, ...
    'Qasperity',candidate.Qasperity,'chiA',candidate.chiA);
end

function side=legacy_side(Qlegacy)
side=struct('Qlegacy',Qlegacy,'Qrough',Qlegacy,'Qgamma',Qlegacy, ...
    'h',nan(size(Qlegacy)),'lambda',nan(size(Qlegacy)), ...
    'Qfluid',Qlegacy,'Qasperity',zeros(size(Qlegacy)),'chiA',zeros(size(Qlegacy)));
end
