function summary = summarize_stage0_repeatability(reportRoot)
%SUMMARIZE_STAGE0_REPEATABILITY Write repeatability results for both bearings.
pairs = {'ball_repeat1.mat','ball_repeat2.mat','ball'; 'roller_repeat1.mat','roller_repeat2.mat','roller'};
summary = table('Size',[2 5], 'VariableTypes',{'string','string','double','double','logical'}, ...
    'VariableNames',{'bearing_type','status','max_relative_error','max_absolute_error','loaded_element_count_equal'});
for k=1:2
    r = compare_stage0_results(fullfile(reportRoot,pairs{k,1}), fullfile(reportRoot,pairs{k,2}), struct());
    summary.bearing_type(k)=string(pairs{k,3}); summary.status(k)=string(r.status);
    summary.max_relative_error(k)=r.max_relative_error; summary.max_absolute_error(k)=r.max_absolute_error;
    summary.loaded_element_count_equal(k)=r.loaded_element_count_equal;
end
writetable(summary, fullfile(reportRoot,'stage0_repeatability.csv'));
save(fullfile(reportRoot,'stage0_repeatability.mat'),'summary');
end
