function tests = test_compare_stage0_results
tests = functiontests(localfunctions);
end

function testMetadataRuntimeDoesNotChangePhysicalComparison(testCase)
root = tempname; mkdir(root); cleanup = onCleanup(@() rmdir(root, 's'));
result = struct('loadj', 3, 'runtime_s', 1, 'stdout', 'first', ...
    'generated', struct('Q1', struct('Q1', [1; 2])));
save(fullfile(root, 'a.mat'), 'result');
result.runtime_s = 99; result.stdout = 'second';
save(fullfile(root, 'b.mat'), 'result');
report = compare_stage0_results(fullfile(root, 'a.mat'), fullfile(root, 'b.mat'), struct());
verifyEqual(testCase, report.status, 'NUMERICALLY_IDENTICAL');
end

function testIdenticalPhysicalValuesAreNumericallyIdentical(testCase)
root = tempname;
mkdir(root);
cleanup = onCleanup(@() rmdir(root, 's'));
result = struct('loadj', 3, 'generated', struct('Q1', struct('Q1', [1; 2])));
save(fullfile(root, 'a.mat'), 'result');
save(fullfile(root, 'b.mat'), 'result');
report = compare_stage0_results(fullfile(root, 'a.mat'), fullfile(root, 'b.mat'), struct());
verifyEqual(testCase, report.status, 'NUMERICALLY_IDENTICAL');
verifyEqual(testCase, report.loaded_element_count_equal, true);
end
