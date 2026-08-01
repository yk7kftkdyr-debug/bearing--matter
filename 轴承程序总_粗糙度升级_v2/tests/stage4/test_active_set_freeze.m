function test_active_set_freeze(projectRoot)
%TEST_ACTIVE_SET_FREEZE Stage 4B-2C-pre active-set/Newton separation.
if nargin<1
    projectRoot=fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
ballRoot=fullfile(projectRoot,'球轴承程序');
addpath(projectRoot); addpath(ballRoot);

% Run the legacy and frozen modes away from the repository.  The legacy
% solver writes MAT state by design; the temporary directory is removed on
% return so this test leaves no result artifacts behind.
originalDir=pwd;
workDir=tempname;
mkdir(workDir);
cleanup=onCleanup(@() cleanup_workspace(originalDir,workDir));
cd(workDir);

micro_config=make_micro_interface_config();
save('micro_config_runtime.mat','micro_config');
datafromvb=fixture_data();

% Test 1: gamma=0 legacy solution is retained by freeze_newton mode.
qiujieLOAD(datafromvb);
legacy=read_solution();

control=struct('active_set_mode','freeze_newton', ...
    'initial_loadi',1:datafromvb(1));
maxActiveIter=datafromvb(1)+1;
stable=false;
allHistories={};
for activeIter=1:maxActiveIter
    frozenLoadi=control.initial_loadi;
    [loadi,www3,activeState]=qiujieLOAD(datafromvb,control); %#ok<ASGLU>

    % Test 2: every recorded Newton iterate used the same ordered loadi.
    assert(isequal(loadi,frozenLoadi), ...
        'qiujieLOAD changed loadi inside a frozen Newton solve.');
    assert(~isempty(activeState.loadi_history), ...
        'Frozen Newton solve did not record its activity-set history.');
    assert(all(cellfun(@(ids) isequal(ids,frozenLoadi), ...
        activeState.loadi_history)), ...
        'loadi changed during a Newton iteration.');
    allHistories{end+1}=activeState.loadi_history; %#ok<AGROW>

    % Test 3: only the caller applies the proposed update between solves.
    if activeState.active_set_stable
        assert(isequal(activeState.next_loadi,frozenLoadi));
        stable=true;
        break;
    end
    assert(~isequal(activeState.next_loadi,frozenLoadi), ...
        'An unstable activity set did not propose an outer update.');
    control.initial_loadi=activeState.next_loadi;
end
assert(stable,'Outer activity-set iteration did not stabilize.');
assert(~isempty(allHistories));

frozen=read_solution();
assert(isequal(frozen.loadi,legacy.loadi),'Final loadi differs from legacy.');
assert_close(frozen.Q1,legacy.Q1,'Q1');
assert_close(frozen.Q2,legacy.Q2,'Q2');
assert_close(frozen.a1,legacy.a1,'a1');
assert_close(frozen.a2,legacy.a2,'a2');
assert_close(frozen.www3,legacy.www3,'displacement');

% Exercise a real outer update without changing load, gap, speed,
% lubrication, or tolerances: start from a restricted ordered subset.
updateControl=struct('active_set_mode','freeze_newton','initial_loadi',1:14);
updateSeen=false;
updateStable=false;
for activeIter=1:maxActiveIter
    frozenLoadi=updateControl.initial_loadi;
    [loadi,~,activeState]=qiujieLOAD(datafromvb,updateControl);
    assert(isequal(loadi,frozenLoadi));
    assert(all(cellfun(@(ids) isequal(ids,frozenLoadi), ...
        activeState.loadi_history)));
    if activeState.active_set_stable
        updateStable=true;
        break;
    end
    assert(~isempty(activeState.next_loadi));
    updateSeen=true;
    updateControl.initial_loadi=activeState.next_loadi;
end
assert(updateSeen,'Restricted fixture did not exercise an outer update.');
assert(updateStable,'Updated activity set did not stabilize.');

% qiujieend owns the outer activity-set loop in production.
entrySource=fileread(fullfile(ballRoot,'qiujieend.m'));
assert(contains(entrySource,'''active_set_mode'',''freeze_newton'''));
assert(contains(entrySource,'activeState.next_loadi'));
assert(contains(entrySource,'activeState.active_set_stable'));

disp('STAGE4B_2C_ACTIVE_SET_FREEZE_PASS');
disp('GO_TO_STAGE4B_2C_NEWTON_CONTINUATION');
end

function result=read_solution()
ids=load('loadi.mat','loadi');
position=load('www3.mat','www3');
contact=load('q1q2a1a2.mat','Q1','Q2','a1','a2');
result=struct('loadi',ids.loadi,'www3',position.www3, ...
    'Q1',contact.Q1,'Q2',contact.Q2,'a1',contact.a1,'a2',contact.a2);
end

function assert_close(actual,expected,label)
scale=max(1,max(abs(expected(:))));
assert(isequal(size(actual),size(expected)) && ...
    max(abs(actual(:)-expected(:)))<=1e-10*scale, ...
    '%s differs from the legacy result.',label);
end

function cleanup_workspace(originalDir,workDir)
cd(originalDir);
if exist(workDir,'dir')==7
    rmdir(workDir,'s');
end
end

function datafromvb=fixture_data()
datafromvb=zeros(75,1);
datafromvb(1)=15; datafromvb(2)=0.02223; datafromvb(3)=0.1253;
datafromvb(4)=0.5232; datafromvb(5)=0.5232; datafromvb(6)=40;
datafromvb(7)=0.0115; datafromvb(9:11)=2.06e11;
datafromvb(12:14)=0.3; datafromvb(15)=10000;
datafromvb(16)=20000; datafromvb(17)=0; datafromvb(18)=3000;
datafromvb(19)=0; datafromvb(20)=0;
datafromvb(21)=0.275; datafromvb(22)=0.225;
datafromvb(23:25)=1e-7; datafromvb(26)=970; datafromvb(27)=20;
datafromvb(28)=0.0966; datafromvb(29)=0.0318;
datafromvb(30)=1.28e-8; datafromvb(31)=3.2e-2;
datafromvb(32)=1; datafromvb(33)=0.4e-3; datafromvb(34)=3.8;
datafromvb(35)=5; datafromvb(36)=370e-3; datafromvb(37)=220e-3;
datafromvb(38)=120e-3; datafromvb(39)=225e-3;
datafromvb(40)=1.96e11; datafromvb(41)=2.18e11;
datafromvb(42)=0.3; datafromvb(43)=0.3;
datafromvb(44)=0.5e-3; datafromvb(45)=-0.5e-3; datafromvb(46)=180;
datafromvb(47)=170; datafromvb(48)=190; datafromvb(49)=27;
datafromvb(50)=7870; datafromvb(51)=7870; datafromvb(52)=7860;
datafromvb(53)=8360; datafromvb(54)=11.6e-6;
datafromvb(55:58)=11.8e-6; datafromvb(59)=195; datafromvb(60)=160;
datafromvb(61)=7800; datafromvb(65)=2.06e11;
datafromvb(66)=0.3; datafromvb(68)=7800; datafromvb(69)=0;
datafromvb(70)=0; datafromvb(71:73)=0.2; datafromvb(74)=2;
end
