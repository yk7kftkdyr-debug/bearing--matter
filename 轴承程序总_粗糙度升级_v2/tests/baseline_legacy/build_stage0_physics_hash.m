function hashes = build_stage0_physics_hash(reportRoot, outputCsv)
%BUILD_STAGE0_PHYSICS_HASH Canonical SHA256 hashes of legacy physical outputs.
files = dir(fullfile(reportRoot, '*.mat'));
rows = {};
for k = 1:numel(files)
    if startsWith(files(k).name, 'stage0_'), continue; end
    data = load(fullfile(files(k).folder, files(k).name));
    if ~isfield(data, 'result'), continue; end
    r = data.result; values = [double(r.loadj); flatten(r.generated); flatten(r.returndata)];
    bytes = typecast(values(:), 'uint8');
    digest = lower(reshape(dec2hex(sha256(bytes),2).',1,[]));
    missing = ''; if isempty(fieldnames(r.generated)), missing = 'generated'; end
    rows(end+1,:) = {erase(files(k).name,'.mat'), r.bearing_type, digest, numel(values), missing, sum(isnan(values)), sum(isinf(values)), sum(~isreal(values)), ternary(r.success,'OK','FAILED')}; %#ok<AGROW>
end
hashes = cell2table(rows, 'VariableNames', {'case_id','bearing_type','physics_hash','variable_count','missing_variables','nan_count','inf_count','complex_count','status'});
writetable(hashes, outputCsv);
end

function x = flatten(value)
if isnumeric(value), x = double(value(:));
elseif isstruct(value), x = []; f=sort(fieldnames(value)); for i=1:numel(f), x=[x; flatten(value.(f{i}))]; end
else, x=[]; end
end
function h = sha256(bytes)
md = java.security.MessageDigest.getInstance('SHA-256'); md.update(bytes); h = typecast(md.digest,'uint8');
end
function value=ternary(condition,a,b), if condition,value=a;else,value=b;end, end
