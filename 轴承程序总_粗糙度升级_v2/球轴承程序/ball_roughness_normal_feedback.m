function [Qused,state] = ball_roughness_normal_feedback(Qlegacy,Qcandidate,cfg)
%BALL_ROUGHNESS_NORMAL_FEEDBACK Stage 4B-2A consistent, disabled interface.
validateattributes(Qlegacy,{'numeric'},{'real','finite','nonnegative'});
validateattributes(Qcandidate,{'numeric'},{'real','finite','nonnegative','size',size(Qlegacy)});
gamma=0;
if isstruct(cfg) && isfield(cfg,'roughness') && isstruct(cfg.roughness) && ...
        isfield(cfg.roughness,'normal_feedback_gamma')
    gamma=cfg.roughness.normal_feedback_gamma;
end
validateattributes(gamma,{'numeric'},{'real','finite','scalar','>=',0,'<=',1});
assert(gamma==0,'ball_roughness_normal_feedback:StageBoundary', ...
    'Stage 4B-2A keeps actual rough normal feedback disabled; gamma must be zero.');
Qused=(1-gamma).*Qlegacy+gamma.*Qcandidate;
state=struct('gamma',gamma,'feedback_active',false);
end
