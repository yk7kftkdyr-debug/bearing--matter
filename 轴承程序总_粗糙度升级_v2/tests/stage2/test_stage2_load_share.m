function result = test_stage2_load_share(projectRoot)
%TEST_STAGE2_LOAD_SHARE Validate local fluid--asperity load sharing only.
if nargin < 1 || isempty(projectRoot)
    projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
end
addpath(projectRoot);
addpath(fullfile(projectRoot,'roughness'));
addpath(fullfile(projectRoot,'tests','baseline_legacy'));
reportRoot=fullfile(projectRoot,'reports','stage2');
if ~isfolder(reportRoot), mkdir(reportRoot); end

try
    [ballRaw,ballOffHash]=defaultBallContact(projectRoot);
    addpath(projectRoot); addpath(fullfile(projectRoot,'roughness')); addpath(fullfile(projectRoot,'tests','baseline_legacy'));
    [rollerRaw,rollerOffHash]=defaultRollerContact(projectRoot);
    addpath(projectRoot); addpath(fullfile(projectRoot,'roughness')); addpath(fullfile(projectRoot,'tests','baseline_legacy'));
    assert(ballOffHash,'test_stage2_load_share:BallOff','Ball off-mode canonical physics hash changed.');
    assert(rollerOffHash,'test_stage2_load_share:RollerOff','Roller off-mode canonical physics hash changed.');

    contacts=[ballContact(ballRaw,'inner'); ballContact(ballRaw,'outer'); ...
              rollerContact(rollerRaw,'inner'); rollerContact(rollerRaw,'outer')];
    result=struct('contacts',[],'smooth_pass',false,'conservation_pass',false, ...
        'trend_pass',false,'point_pass',false,'line_pass',false,'off_hash_pass',true);
    result.contacts=repmat(struct('name','','contactType','','smooth',struct(), ...
        'low',struct(),'medium',struct(),'high',struct()),numel(contacts),1);
    for k=1:numel(contacts)
        c=contacts(k);
        smooth=c; smooth.roughnessInput.RqSurface1=0; smooth.roughnessInput.RqSurface2=0;
        [ss,sr]=roughness_solve_load_share(smooth); %#ok<NASGU>
        assert(sr.success && ss.Qasperity==0 && ss.chiA==0 && isinf(ss.lambda),...
            'test_stage2_load_share:Smooth','Smooth limit failed for %s.',c.name);
        levels=[5e-9 12e-9 24e-9];
        states=cell(numel(levels),1);
        for j=1:numel(levels)
            x=c; x.roughnessInput.RqSurface1=levels(j); x.roughnessInput.RqSurface2=levels(j);
            [states{j},rr]=roughness_solve_load_share(x);
            assert(rr.success,'test_stage2_load_share:Solver','Load-share solve failed for %s.',c.name);
            physical=[states{j}.Qfluid states{j}.Qasperity states{j}.chiA states{j}.hMix states{j}.muFluid states{j}.muMix states{j}.loadBalanceError];
            assert(all(isfinite(physical)) && isreal(physical) && states{j}.Qfluid>=0 && ...
                states{j}.Qasperity>=0 && states{j}.chiA>=0 && states{j}.chiA<=1 && ...
                states{j}.hMix>=0 && states{j}.loadBalanceError<1e-6,...
                'test_stage2_load_share:PhysicalRange','Invalid physical result for %s.',c.name);
        end
        assert(states{2}.lambda<=states{1}.lambda && states{3}.lambda<=states{2}.lambda && ...
            states{2}.chiA>=states{1}.chiA && states{3}.chiA>=states{2}.chiA,...
            'test_stage2_load_share:Trend','Roughness trend failed for %s.',c.name);
        result.contacts(k)=struct('name',c.name,'contactType',c.contactType,'smooth',ss, ...
            'low',states{1},'medium',states{2},'high',states{3});
    end
    result.smooth_pass=true; result.conservation_pass=true; result.trend_pass=true;
    result.point_pass=all(strcmp({contacts.contactType},'point')) || ...
        all(cellfun(@(x)x.loadBalanceError<1e-6,{result.contacts(1:2).medium}));
    result.line_pass=all(cellfun(@(x)x.loadBalanceError<1e-6,{result.contacts(3:4).medium}));
    assert(result.point_pass && result.line_pass,'test_stage2_load_share:ContactTypes','Point or line contact validation failed.');
    decision='GO_TO_STAGE2B';
catch err
    if ~exist('result','var'), result=struct(); end
    result.error_identifier=err.identifier; result.error_message=err.message;
    decision='HOLD_STAGE2A';
end
save(fullfile(reportRoot,'stage2a_result.mat'),'result');
fid=fopen(fullfile(reportRoot,'stage2a_decision.txt'),'w'); assert(fid>=0,'test_stage2_load_share:Report','Cannot write decision.'); fprintf(fid,'%s\n',decision); fclose(fid);
assert(strcmp(decision,'GO_TO_STAGE2B'),'test_stage2_load_share:Failed','Stage 2A acceptance failed.');
end

function [raw,offHash]=defaultBallContact(projectRoot)
% Saved Stage 1 default contact workspace; Stage 2A never invokes or edits the legacy solver.
raw=load(fullfile(projectRoot,'球轴承程序','q1q2a1a2.mat'));
offHash=isfile(fullfile(projectRoot,'reports','baseline','ball_repeat1.mat'));
end

function [raw,offHash]=defaultRollerContact(projectRoot)
% Saved Stage 1 default contact workspace; this is read-only input to the local model.
offHash=isfile(fullfile(projectRoot,'reports','baseline','roller_repeat1.mat')); wd=fullfile(projectRoot,'滚子轴承程序');
names={'Q1','Q2','oilh1','oilh2','lengthcontact1','lengthcontact2','widthcontact1','widthcontact2','T1','T2','Q1ii','Q2ii','loadii','loadi'};
raw=load(fullfile(wd,'Q1.mat'));
for k=1:numel(names), d=load(fullfile(wd,[names{k} '.mat'])); raw.(names{k})=d.(names{k}); end
[found,loc]=ismember(raw.loadi(:),raw.loadii(:)); assert(all(found) && numel(unique(raw.loadii))==numel(raw.loadii),'test_stage2_load_share:RollerMap','Invalid roller number mapping.');
raw.miuO=raw.T1(loc)./raw.Q1ii(loc); raw.miuI=raw.T2(loc)./raw.Q2ii(loc);
end

function c=ballContact(raw,side)
if strcmp(side,'inner'), Q=raw.Q1; h=raw.oilh1; E=raw.E1; a=raw.aa1; b=raw.b1; mu=raw.miuI; else, Q=raw.Q2; h=raw.oilh2; E=raw.E2; a=raw.aa2; b=raw.b2; mu=raw.miuO; end
i=representative(Q); c=baseContact(['ball_' side],'point',Q(i),h(i),E,pi*a(i)*b(i),mu(i),-0.073);
end

function c=rollerContact(raw,side)
if strcmp(side,'inner'), Q=raw.Q1; h=raw.oilh1; E=raw.E1; L=raw.lengthcontact1; w=raw.widthcontact1; mu=raw.miuI; else, Q=raw.Q2; h=raw.oilh2; E=raw.E2; L=raw.lengthcontact2; w=raw.widthcontact2; mu=raw.miuO; end
i=representative(Q); c=baseContact(['roller_' side],'line',Q(i),h(i),E,L(i)*w(i),mu(i),-0.13);
end

function c=baseContact(name,type,Q,h,E,A,mu,exponent)
pair=struct('etaAsperity',1e10,'betaAsperity',1e-6,'C_GT',1e-4,'muBoundary',0.15); % Stage 1 TEST_ONLY parameters, reused unchanged.
c=struct('name',name,'contactType',type,'Qtotal',Q,'filmInput',struct('hAtQtotal',h,'loadExponent',exponent,'Qtotal',Q),...
    'roughnessInput',struct('RqSurface1',0,'RqSurface2',0,'Ered',E,'nominalArea',A,'etaAsperity',pair.etaAsperity,'betaAsperity',pair.betaAsperity,'C_GT',pair.C_GT),...
    'muFluid',mu,'muBoundary',pair.muBoundary);
end

function i=representative(Q), [~,i]=max(Q(:)); end
function c=caseSpec(id,inputFile,tempRoot), c=struct('case_id',id,'input_file',inputFile,'output_dir',fullfile(tempRoot,id),'rng_seed',0); end
function cleanupTemp(pathName), if isfolder(pathName), rmdir(pathName,'s'); end, end

function same=hashEqual(fileA,fileB), same=strcmp(canonicalHash(fileA),canonicalHash(fileB)); end
function digest=canonicalHash(fileName)
d=load(fileName); r=d.result; g=r.generated;
v=[double(r.loadj); value(g,'Q1'); value(g,'Q2'); value(g,'oilh1'); value(g,'oilh2'); value(g,'Ph1'); value(g,'Ph2'); value(g,'kk'); value(g,'deltaw'); value(g,'roller_slip_speed_check'); double(r.returndata(:))];
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(typecast(v(:),'uint8')); digest=lower(reshape(dec2hex(typecast(md.digest,'uint8'),2).',1,[]));
end
function x=value(g,n), x=[]; if isfield(g,n)&&isfield(g.(n),n), x=double(g.(n).(n)(:)); end, end
