function result = test_stage3c_ffspeed_hook(projectRoot)
%TEST_STAGE3C_FFSPEED_HOOK Verify the only Level-2 addition is a pure hook.
if nargin < 1 || isempty(projectRoot)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
fileName = fullfile(projectRoot,'滚子轴承程序','ffSPEED.m');
source = fileread(fileName);
callCount = numel(strfind(source,'apply_roller_traction_override(')); %#ok<STREMP>
assert(callCount == 1, ...
    'test_stage3c_ffspeed_hook:Hook','ffSPEED must call the pure traction override exactly once.');
assert(~isempty(strfind(source,'traction_override_report')), ... %#ok<STREMP>
    'test_stage3c_ffspeed_hook:Report','ffSPEED must reject an unsuccessful override report.');
assert(isempty(strfind(source,'qiujieall(')) && isempty(strfind(source,'ff2(')) && ...
    isempty(strfind(source,'ff3(')) && isempty(strfind(source,'ffLOAD(')), ... %#ok<STREMP>
    'test_stage3c_ffspeed_hook:Scope','ffSPEED must not re-enter a normal solver.');
result = struct('passed',true,'call_count',callCount);
end
