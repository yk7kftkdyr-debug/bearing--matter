function isolation = check_stage0_mat_isolation(reportRoot, outputCsv)
%CHECK_STAGE0_MAT_ISOLATION Verify each saved case result is individually readable.
files=dir(fullfile(reportRoot,'*.mat')); rows={}; paths=string(fullfile({files.folder},{files.name}));
for k=1:numel(files)
    if startsWith(files(k).name,'stage0_'), continue; end
    path=fullfile(files(k).folder,files(k).name); readable=true; message=''; key=false;
    try, data=load(path); key=isfield(data,'result') && isfield(data.result,'generated'); catch err, readable=false; message=err.message; end
    rows(end+1,:)={erase(files(k).name,'.mat'),true,readable, sum(paths==path)==1,key,~(readable&&key),message}; %#ok<AGROW>
end
isolation=cell2table(rows,'VariableNames',{'case_id','result_exists','result_readable','path_isolated','key_variables_present','pollution_detected','message'});
writetable(isolation,outputCsv);
end
