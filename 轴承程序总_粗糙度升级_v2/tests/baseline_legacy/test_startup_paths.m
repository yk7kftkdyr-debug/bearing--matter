function tests = test_startup_paths
%TEST_STARTUP_PATHS Verify the stage-0 startup isolates legacy functions.
tests = functiontests(localfunctions);
end

function testStartupResolvesProjectFunctions(testCase)
projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
run(fullfile(projectRoot, 'startup_roughness_v2.m'));

verifyEqual(testCase, fileparts(which('qiujieend')), ...
    fullfile(projectRoot, '球轴承程序'));
verifyEqual(testCase, fileparts(which('qiujieall')), ...
    fullfile(projectRoot, '滚子轴承程序'));
verifyEqual(testCase, fileparts(which('ffLOAD')), ...
    fullfile(projectRoot, '球轴承程序'));
verifyEqual(testCase, fileparts(which('ff2')), ...
    fullfile(projectRoot, '滚子轴承程序'));
verifyEqual(testCase, fileparts(which('ff3')), ...
    fullfile(projectRoot, '滚子轴承程序'));
verifyEqual(testCase, fileparts(which('make_micro_interface_config')), projectRoot);
end
