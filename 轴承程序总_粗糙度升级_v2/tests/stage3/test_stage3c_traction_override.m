function result = test_stage3c_traction_override(projectRoot)
%TEST_STAGE3C_TRACTION_OVERRIDE Contract test for the pure roller T override.
if nargin < 1 || isempty(projectRoot)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
addpath(projectRoot);
addpath(fullfile(projectRoot,'roughness'));

loadi = [8;5;2];
loadii = [5;2;8;7];
T1 = [10;20;30;40];
T2 = [11;21;31;41];
smoothState = traction_fixture(false,0);
roughState = traction_fixture(true,1);

[out1,out2,off] = apply_roller_traction_override(T1,T2,loadi,loadii,smoothState,config(0));
assert(isequal(out1,T1) && isequal(out2,T2) && off.success && off.legacy_unchanged, ...
    'test_stage3c_traction_override:Off','Level zero must retain legacy traction exactly.');
[out1,out2,levelOne] = apply_roller_traction_override(T1,T2,loadi,loadii,smoothState,config(1));
assert(isequal(out1,T1) && isequal(out2,T2) && levelOne.success && levelOne.legacy_unchanged, ...
    'test_stage3c_traction_override:LevelOne','Level one must retain legacy traction exactly.');
[smooth1,smooth2,smooth] = apply_roller_traction_override(T1,T2,loadi,loadii,smoothState,config(2));
assert(isequal(smooth1,T1) && isequal(smooth2,T2) && smooth.success && ...
    smooth.mapping_valid && smooth.used_frozen_snapshot, ...
    'test_stage3c_traction_override:Smooth','Level two smooth state must retain legacy traction exactly.');

[rough1,rough2,rough] = apply_roller_traction_override(T1,T2,loadi,loadii,roughState,config(2));
[found,location] = ismember(loadi,loadii);
assert(all(found), 'test_stage3c_traction_override:Fixture','Fixture map is invalid.');
expected1 = T1; expected2 = T2;
expected1(location) = [roughState.outer.Tused_outer].';
expected2(location) = [roughState.inner.Tused_inner].';
assert(isequal(rough1,expected1) && isequal(rough2,expected2) && rough.success && ...
    rough.mapping_valid && rough.outer_override_count == numel(loadi) && ...
    rough.inner_override_count == numel(loadi) && rough.finite, ...
    'test_stage3c_traction_override:Mapping', ...
    'Level two must use side-specific Tused values through the explicit ID map.');

missing = roughState; missing.outer = missing.outer(1:2);
[bad1,bad2,bad] = apply_roller_traction_override(T1,T2,loadi,loadii,missing,config(2));
assert(~bad.success && isequal(bad1,T1) && isequal(bad2,T2) && ~bad.mapping_valid, ...
    'test_stage3c_traction_override:MissingId','Missing roller identifiers must fail without changing traction.');
nonfinite = roughState; nonfinite.outer(1).Tused_outer = NaN;
[bad1,bad2,bad] = apply_roller_traction_override(T1,T2,loadi,loadii,nonfinite,config(2));
assert(~bad.success && isequal(bad1,T1) && isequal(bad2,T2) && ~bad.finite, ...
    'test_stage3c_traction_override:Nonfinite','NaN traction state must fail without changing traction.');
nonfinite = roughState; nonfinite.inner(1).Tused_inner = Inf;
[~,~,bad] = apply_roller_traction_override(T1,T2,loadi,loadii,nonfinite,config(2));
assert(~bad.success && ~bad.finite, ...
    'test_stage3c_traction_override:Nonfinite','Inf traction state must fail explicitly.');
nonfinite = roughState; nonfinite.inner(1).Tused_inner = 1+1i;
[~,~,bad] = apply_roller_traction_override(T1,T2,loadi,loadii,nonfinite,config(2));
assert(~bad.success && ~bad.finite, ...
    'test_stage3c_traction_override:Nonfinite','Complex traction state must fail explicitly.');

source = fileread(which('apply_roller_traction_override'));
assert(isempty(regexp(source,'(?m)^\\s*(save|load|global|persistent|evalin|assignin|cd)\\b','once')) && ...
    isempty(strfind(source,'qiujieall')) && isempty(strfind(source,'ffLOAD')), ... %#ok<STREMP>
    'test_stage3c_traction_override:Purity','The override must remain a pure tangential-state function.');
result = struct('off_passed',true,'level1_passed',true,'smooth_passed',true, ...
    'rough_mapping_passed',true,'failure_paths_passed',true,'pure_passed',true, ...
    'report',rough);
end

function value = config(level)
value = struct('roughness',struct('enabled',level > 0,'feedback_level',level));
end

function state = traction_fixture(isRough,gamma)
ids = [8;5;2];
outer = side(ids,[100;80;60],[20;16;12],[0.02;0.03;0.025], ...
    [0.07;0.06;0.055],[2;2.4;1.5],[-4;3;-2],isRough);
inner = side(ids,[90;70;50],[18;14;10],[0.028;0.022;0.032], ...
    [0.068;0.052;0.070],[1.8;1.4;1.25],[5;-1;2],isRough);
state = build_roller_traction_feedback_state(struct('roller_id',ids, ...
    'outer',outer,'inner',inner),gamma);
end

function value = side(ids,Qtotal,Qasperity,muFluid,muMix,Tlegacy,slip,isRough)
if ~isRough, Qasperity(:) = 0; muMix = muFluid; end
value = struct('roller_id',ids,'Qtotal',Qtotal,'Qfluid',Qtotal-Qasperity, ...
    'Qasperity',Qasperity,'muFluid',muFluid,'muMix',muMix, ...
    'Tlegacy',Tlegacy,'slip_velocity',slip);
end
