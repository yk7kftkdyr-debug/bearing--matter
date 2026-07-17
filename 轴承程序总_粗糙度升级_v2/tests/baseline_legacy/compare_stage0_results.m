function report = compare_stage0_results(fileA, fileB, options)
%COMPARE_STAGE0_RESULTS Compare legacy results without metadata-only failure.
if nargin < 3, options = struct(); end
options = defaults(options);
report = struct('file_a', fileA, 'file_b', fileB, 'status', 'FAIL', ...
    'readable', false, 'variable_names_equal', false, ...
    'loaded_element_count_equal', false, 'max_absolute_error', NaN, ...
    'max_relative_error', NaN, 'rms_error', NaN, 'nan_count', 0, ...
    'inf_count', 0, 'complex_count', 0, 'differences', {{}});
try
    a = load(fileA); b = load(fileB);
catch err
    report.differences = {err.message}; return;
end
report.readable = true;
namesA = sort(fieldnames(a)); namesB = sort(fieldnames(b));
report.variable_names_equal = isequal(namesA, namesB);
if ~report.variable_names_equal
    report.differences = {'Top-level MAT variable names differ'}; return;
end
[metrics, equal] = compareValue(physicalOnly(a), physicalOnly(b), options.scale_floor);
report.max_absolute_error = metrics.max_abs;
report.max_relative_error = metrics.max_rel;
report.rms_error = metrics.rms;
report.nan_count = metrics.nan_count; report.inf_count = metrics.inf_count;
report.complex_count = metrics.complex_count;
report.loaded_element_count_equal = equalLoadedCount(a, b);
if equal && report.loaded_element_count_equal
    report.status = 'NUMERICALLY_IDENTICAL';
elseif report.max_relative_error <= options.relative_tolerance && report.loaded_element_count_equal
    report.status = 'WITHIN_TOLERANCE';
end

function value = physicalOnly(value)
if isstruct(value)
    metadata = {'runtime_s','stdout','warning','warning_id','input'};
    fields = intersect(fieldnames(value), metadata);
    if ~isempty(fields), value = rmfield(value, fields); end
    fields = fieldnames(value);
    for k=1:numel(fields), value.(fields{k}) = physicalOnly(value.(fields{k})); end
end
end
end

function options = defaults(options)
if ~isfield(options, 'scale_floor'), options.scale_floor = 1e-12; end
if ~isfield(options, 'relative_tolerance'), options.relative_tolerance = 1e-10; end
end

function [m, equal] = compareValue(a, b, floorValue)
m = struct('max_abs', 0, 'max_rel', 0, 'rms', 0, 'nan_count', 0, 'inf_count', 0, 'complex_count', 0);
equal = isequaln(a, b);
if isnumeric(a) && isnumeric(b) && isequal(size(a), size(b))
    d = abs(a(:)-b(:)); denom = max([abs(a(:)), abs(b(:)), floorValue*ones(numel(a),1)], [], 2);
    m.max_abs = max([0; d]); m.max_rel = max([0; d./denom]); m.rms = sqrt(mean(d.^2));
    m.nan_count = sum(isnan(a(:))) + sum(isnan(b(:))); m.inf_count = sum(isinf(a(:))) + sum(isinf(b(:))); m.complex_count = sum(~isreal(a(:))) + sum(~isreal(b(:)));
elseif isstruct(a) && isstruct(b) && isequal(sort(fieldnames(a)), sort(fieldnames(b)))
    fields = fieldnames(a); for k=1:numel(fields), [x, e] = compareValue(a.(fields{k}), b.(fields{k}), floorValue); m.max_abs=max(m.max_abs,x.max_abs);m.max_rel=max(m.max_rel,x.max_rel);m.rms=max(m.rms,x.rms);m.nan_count=m.nan_count+x.nan_count;m.inf_count=m.inf_count+x.inf_count;m.complex_count=m.complex_count+x.complex_count; equal=equal&&e; end
end
end

function value = equalLoadedCount(a, b)
value = true;
if isfield(a, 'result') && isfield(b, 'result') && isfield(a.result, 'loadj') && isfield(b.result, 'loadj')
    value = isequal(a.result.loadj, b.result.loadj);
end
end
