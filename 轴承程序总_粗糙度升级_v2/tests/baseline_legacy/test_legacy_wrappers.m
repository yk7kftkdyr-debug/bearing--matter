function results = test_legacy_wrappers(projectRoot)
%TEST_LEGACY_WRAPPERS Verify isolated wrapper equivalence and call ordering.
if nargin < 1 || isempty(projectRoot)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
addpath(projectRoot);
addpath(fullfile(projectRoot,'tests','baseline_legacy'));
baselineRoot = fullfile(projectRoot,'reports','baseline');
reportRoot = fullfile(projectRoot,'reports','stage0_5');
if ~isfolder(reportRoot), mkdir(reportRoot); end

tempRoot = tempname; mkdir(tempRoot);
cleanup = onCleanup(@() cleanupTemp(tempRoot)); %#ok<NASGU>
ballBase = fullfile(baselineRoot,'ball_repeat1.mat');
rollerBase = fullfile(baselineRoot,'roller_repeat1.mat');

% Required sequence: ball -> roller -> ball.
[ballStart, ~] = legacy_run_ball(caseSpec('ball_sequence_start',ballBase,tempRoot));
[rollerMiddle, ~] = legacy_run_roller(caseSpec('ball_sequence_middle_roller',rollerBase,tempRoot));
[ballEnd, ~] = legacy_run_ball(caseSpec('ball_sequence_end',ballBase,tempRoot));
% Required sequence: roller -> ball -> roller.
[rollerStart, ~] = legacy_run_roller(caseSpec('roller_sequence_start',rollerBase,tempRoot));
[~, ~] = legacy_run_ball(caseSpec('roller_sequence_middle_ball',ballBase,tempRoot));
[rollerEnd, ~] = legacy_run_roller(caseSpec('roller_sequence_end',rollerBase,tempRoot));

% Each legacy call resets MATLAB's path; restore comparison helpers once.
addpath(projectRoot);
addpath(fullfile(projectRoot,'tests','baseline_legacy'));
rows = [makeRow('ball_wrapper_equivalence',ballBase,ballStart.raw_result_file); ...
        makeRow('roller_wrapper_equivalence',rollerBase,rollerMiddle.raw_result_file); ...
        makeRow('ball_order_independence',ballStart.raw_result_file,ballEnd.raw_result_file); ...
        makeRow('roller_order_independence',rollerStart.raw_result_file,rollerEnd.raw_result_file)];
results = struct2table(rows);
save(fullfile(reportRoot,'wrapper_test_result.mat'),'results');
writetable(results,fullfile(reportRoot,'wrapper_test_result.csv'));
assert(all(results.pass), 'test_legacy_wrappers:Failed', 'Legacy wrapper validation did not pass.');
end

function c = caseSpec(caseId, inputFile, tempRoot)
c = struct('case_id',caseId,'input_file',inputFile, ...
    'output_dir',fullfile(tempRoot,caseId),'rng_seed',0);
end

function row = makeRow(testName, fileA, fileB)
c = compare_stage0_results(fileA,fileB,struct('relative_tolerance',1e-12));
hashEqual = strcmp(physicsHash(fileA),physicsHash(fileB));
physical = physicalIntegrity(fileA,fileB);
diagnosticNaN = diagnosticNaNCount(fileA) + diagnosticNaNCount(fileB);
pass = c.readable && c.loaded_element_count_equal && hashEqual && ...
    physical.nan_count == 0 && physical.inf_count == 0 && physical.complex_count == 0 && ...
    (strcmp(c.status,'NUMERICALLY_IDENTICAL') || ...
     (strcmp(c.status,'WITHIN_TOLERANCE') && c.max_relative_error <= 1e-12));
row = struct('test_name',testName,'pass',pass,'status',string(c.status), ...
    'max_absolute_error',c.max_absolute_error,'max_relative_error',c.max_relative_error, ...
    'loaded_element_count_equal',c.loaded_element_count_equal,'physics_hash_equal',hashEqual, ...
    'nan_count',physical.nan_count,'inf_count',physical.inf_count,'complex_count',physical.complex_count, ...
    'diagnostic_nan_count',diagnosticNaN);
end

function digest = physicsHash(fileName)
values = physicalValues(fileName);
md = java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(values(:),'uint8'));
digest = lower(reshape(dec2hex(typecast(md.digest,'uint8'),2).',1,[]));
end

function x = flatten(value)
if isnumeric(value)
    x = double(value(:));
elseif isstruct(value)
    x = []; fields = sort(fieldnames(value));
    for k = 1:numel(fields), x = [x; flatten(value.(fields{k}))]; end %#ok<AGROW>
else
    x = [];
end
end

function info = physicalIntegrity(fileA,fileB)
a=physicalValues(fileA); b=physicalValues(fileB); v=[a;b];
info=struct('nan_count',sum(isnan(v)),'inf_count',sum(isinf(v)), ...
    'complex_count',sum(~isreal(v)));
end
function v = physicalValues(fileName)
d=load(fileName); r=d.result;
g=r.generated;
% Whitelist final physical outputs; raw kk workspace includes legacy sentinels.
v=[double(r.loadj); value(g,'Q1','Q1'); value(g,'Q2','Q2'); ...
   value(g,'oilh1','oilh1'); value(g,'oilh2','oilh2'); ...
   value(g,'Ph1','Ph1'); value(g,'Ph2','Ph2'); value(g,'kk','kk'); ...
   value(g,'deltaw','deltaw'); value(g,'roller_slip_speed_check','roller_slip_speed_check'); ...
   double(r.returndata(:))];
end
function x=value(g,container,field)
x=[]; if isfield(g,container) && isfield(g.(container),field), x=double(g.(container).(field)(:)); end
end
function n = diagnosticNaNCount(fileName)
d=load(fileName); n=0;
if isfield(d,'result') && isfield(d.result,'result333')
    n=sum(isnan(flatten(d.result.result333)));
end
end

function cleanupTemp(pathName)
if isfolder(pathName), rmdir(pathName,'s'); end
end
