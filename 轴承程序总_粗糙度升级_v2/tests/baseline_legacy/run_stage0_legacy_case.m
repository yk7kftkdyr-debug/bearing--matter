function result = run_stage0_legacy_case(projectRoot, bearingType, loadFactor, speedFactor, outputFile)
%RUN_STAGE0_LEGACY_CASE Execute one isolated legacy baseline case.
run(fullfile(projectRoot, 'startup_roughness_v2.m'));
clear global;
lastwarn('');
rng(0, 'twister');
caseStart = tic;
warningText = '';
try
    if strcmp(bearingType, 'ball')
        input = ballInput();
        input(16:20) = input(16:20) * loadFactor;
        input([15, 69]) = input([15, 69]) * speedFactor;
        workDir = fullfile(projectRoot, '球轴承程序');
        runnerText = '[loadj, returndata] = qiujieend(input, make_micro_interface_config());';
    else
        input = defaultBearingInput();
        input(19:22) = input(19:22) * loadFactor;
        input(17:18) = input(17:18) * speedFactor;
        workDir = fullfile(projectRoot, '滚子轴承程序');
        runnerText = '[loadj, returndata] = qiujieall(input, make_micro_interface_config());';
    end
    cd(workDir);
    stdout = evalc(runnerText);
    result.success = true;
    result.loadj = loadj;
    result.returndata = returndata;
catch err
    stdout = evalc('disp(getReport(err, ''extended'', ''hyperlinks'', ''off''))');
    result.success = false;
    result.loadj = NaN;
    result.returndata = [];
end
[warningText, warningId] = lastwarn;
result.bearing_type = bearingType;
result.load_factor = loadFactor;
result.speed_factor = speedFactor;
result.input = input;
result.stdout = stdout;
result.warning = warningText;
result.warning_id = warningId;
result.runtime_s = toc(caseStart);
result.generated = collectLegacyOutputs(workDir);
result.result333 = parseResult333(stdout);
save(outputFile, 'result');
end

function input = ballInput()
input=zeros(75,1); input(1)=15;input(2)=0.02223;input(3)=0.1253;input(4)=0.5232;input(5)=0.5232;input(6)=40;input(7)=0.0115;input(35)=5;
input(61)=7800;input(68)=7800;input(51)=7800;input(50)=7800;input(9:11)=2.06e11;input([12:14,66])=0.3;input(15)=10000;input(16)=20000;input(18)=3000;input(21)=0.275;input(22)=0.225;input(23:25)=1e-7;input(26)=970;input(27)=20;input(28)=0.0966;input(29)=0.0318;input(30)=1.28e-8;input(31)=3.2e-2;input(32)=1;input(33)=0.4e-3;input(34)=3.8;input(74)=2;input(36)=370e-3;input(37)=220e-3;input(38)=120e-3;input(39)=225e-3;input(40)=1.96e11;input(41)=2.18e11;input(42:43)=0.3;input(44)=0.5e-3;input(45)=-0.5e-3;input(54:58)=[11.6 11.8 11.8 11.8 11.8]*1e-6;input(46)=180;input(47)=170;input(48)=190;input(49)=27;input(59)=195;input(60)=160;input(50)=7870;input(51)=7870;input(52)=7860;input(53)=8360;input(70)=0;input(71:73)=0.2;
end

function outputs = collectLegacyOutputs(workDir)
names = {'Q1','Q2','oilh1','oilh2','Ph1','Ph2','kk','deltaw','roller_slip_speed_check'};
outputs = struct();
for k = 1:numel(names)
    path = fullfile(workDir, [names{k} '.mat']);
    if isfile(path)
        data = load(path); outputs.(names{k}) = data;
    end
end
end

function info = parseResult333(stdout)
tokens = regexp(stdout, 'result333\\s*=\\s*([+-]?\\d*\\.?\\d+(?:[eE][+-]?\\d+)?)', 'tokens');
values = cellfun(@(x) str2double(x{1}), tokens);
info.classification = 'LEGACY_VERBOSE_CONVERGENCE_OUTPUT';
info.count = numel(values); info.initial = NaN; info.final = NaN; info.maximum = NaN; info.nonfinite = any(~isfinite(values));
if ~isempty(values), info.initial=values(1); info.final=values(end); info.maximum=max(values); end
end
