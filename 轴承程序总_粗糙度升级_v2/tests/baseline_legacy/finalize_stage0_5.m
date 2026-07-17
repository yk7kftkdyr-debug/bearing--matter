function decision = finalize_stage0_5(projectRoot)
%FINALIZE_STAGE0_5 Make the Stage 0.5 gate decision from saved evidence.
if nargin < 1 || isempty(projectRoot)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
stage0 = fullfile(projectRoot,'reports','stage0');
stage05 = fullfile(projectRoot,'reports','stage0_5');
required = {fullfile(stage0,'stage0_numerical_scales.mat'), ...
    fullfile(stage0,'stage0_numerical_scales.csv'), ...
    fullfile(stage0,'stage0_mat_isolation.csv'), ...
    fullfile(stage0,'stage0_physics_hash.csv'), ...
    fullfile(stage05,'wrapper_test_result.mat'), ...
    fullfile(stage05,'wrapper_test_result.csv')};
assert(all(cellfun(@isfile,required)),'finalize_stage0_5:MissingEvidence', ...
    'Stage 0.5 evidence is incomplete.');

wrapper = load(fullfile(stage05,'wrapper_test_result.mat'));
isolation = readtable(fullfile(stage0,'stage0_mat_isolation.csv'));
hashes = readtable(fullfile(stage0,'stage0_physics_hash.csv'));
scales = load(fullfile(stage0,'stage0_numerical_scales.mat'));
requiredTests = ["ball_wrapper_equivalence";"roller_wrapper_equivalence"; ...
    "ball_order_independence";"roller_order_independence"];
testsPresent = all(ismember(requiredTests,string(wrapper.results.test_name)));
testsPass = testsPresent && all(wrapper.results.pass);
isolationPass = all(isolation.result_exists & isolation.result_readable & ...
    isolation.path_isolated & isolation.key_variables_present & ~isolation.pollution_detected);
hashPass = all(strcmp(string(hashes.status),"OK")) && all(hashes.nan_count == 0) && ...
    all(hashes.inf_count == 0) && all(hashes.complex_count == 0);
scalePass = isfield(scales,'scales') && height(scales.scales) > 0;
if testsPass && isolationPass && hashPass && scalePass
    decision = 'GO_TO_STAGE1';
else
    decision = 'HOLD_STAGE0_5';
end
fid = fopen(fullfile(stage05,'stage0_5_decision.txt'),'w');
assert(fid >= 0,'finalize_stage0_5:WriteFailure','Cannot write Stage 0.5 decision.');
fprintf(fid,'%s\n',decision);
fclose(fid);
end
