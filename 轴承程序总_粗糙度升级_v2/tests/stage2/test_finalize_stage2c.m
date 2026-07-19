function test_finalize_stage2c()
%TEST_FINALIZE_STAGE2C Result-only aggregation accepts persisted rough evidence.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
external = '/private/tmp/stage2c_regression';
out = fullfile(external,'aggregation_test_output');
if isfolder(out), rmdir(out,'s'); end
mkdir(out);
cleanup = onCleanup(@() rmdir(out,'s')); %#ok<NASGU>

summary = finalize_stage2c(root,external,out);
assert(summary.ball_off_passed, ...
    'Diagnostic result333 NaN must not invalidate ball OFF physical acceptance.');
assert(summary.roller_feedback_passed, ...
    'Persisted roller rough-feedback evidence must pass Stage 2C aggregation.');
assert(summary.stage2_total_passed);
assert(strcmp(summary.decision,'GO_TO_STAGE3_FRICTION'));
assert(isfile(fullfile(out,'stage2_total_validation.mat')));
assert(isfile(fullfile(out,'stage2c_result.mat')));
assert(isfile(fullfile(out,'stage2c_decision.txt')));
end
