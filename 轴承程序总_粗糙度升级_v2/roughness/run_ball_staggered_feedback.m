function [result,report] = run_ball_staggered_feedback(caseInput,roughConfig)
%RUN_BALL_STAGGERED_FEEDBACK Fixed-film staggered Picard coupling for balls.
% The correction vectors are indexed by physical ball number and are held
% fixed throughout each complete legacy Newton solve.

validateattributes(caseInput,{'struct'},{'scalar'});
assert(isfield(caseInput,'input') && isfield(caseInput,'output_dir'), ...
    'run_ball_staggered_feedback:Input', ...
    'caseInput must provide input and output_dir.');

gammaSequence = option(roughConfig,'gamma_sequence',[0, 0.5, 1]);
maxOuter = option(roughConfig,'max_outer_iterations',8);
omega = option(roughConfig,'relaxation',0.5);
assert(isnumeric(gammaSequence) && isreal(gammaSequence) && ...
    all(isfinite(gammaSequence)) && all(gammaSequence >= 0) && ...
    all(gammaSequence <= 1), 'run_ball_staggered_feedback:Gamma', ...
    'gamma_sequence must lie in [0,1].');
assert(isscalar(maxOuter) && maxOuter >= 1 && maxOuter == floor(maxOuter), ...
    'run_ball_staggered_feedback:Iterations','max_outer_iterations must be a positive integer.');
assert(isscalar(omega) && omega > 0 && omega <= 1, ...
    'run_ball_staggered_feedback:Relaxation','relaxation must lie in (0,1].');

root = fileparts(fileparts(mfilename('fullpath')));
work = caseInput.output_dir;
if isfolder(work)
    rmdir(work,'s');
end
copyfile(root,work);
old = pwd;
cleanup = onCleanup(@()cleanup_workspace(old,work));
cd(fullfile(work,'球轴承程序'));

input = caseInput.input;
n = input(1);
validateattributes(n,{'numeric'},{'scalar','integer','positive','finite'});
deltaOuter = zeros(n,1);
deltaInner = zeros(n,1);
history = struct('gamma',{},'iteration',{},'delta_h_outer_used',{}, ...
    'delta_h_inner_used',{},'delta_h_outer_updated',{}, ...
    'delta_h_inner_updated',{},'relative_update',{});
states = struct('outer',[],'inner',[]);
returndata = [];

try
    for gamma = gammaSequence(:).'
        stableCount = 0;
        for iteration = 1:maxOuter
            deltaOuterUsed = deltaOuter;
            deltaInnerUsed = deltaInner;
            config = make_micro_interface_config(struct('roughness',roughConfig));
            config.roughness.enabled = true;
            config.roughness.feedback_level = 1;
            config.roughness.feedback.ball.delta_h_outer = deltaOuterUsed;
            config.roughness.feedback.ball.delta_h_inner = deltaInnerUsed;

            % qiujieend completes the full legacy Newton solve before target
            % corrections are calculated below; config is never mutated inside it.
            evalc('[~, returndata] = qiujieend(input,config);');
            data = load('q1q2a1a2.mat');
            [targetOuter,targetInner,states] = roughness_targets(data,roughConfig,n);

            deltaOuter = (1-omega)*deltaOuterUsed + omega*gamma*targetOuter;
            deltaInner = (1-omega)*deltaInnerUsed + omega*gamma*targetInner;
            assert_physical_vector(deltaOuter,'delta_h_outer');
            assert_physical_vector(deltaInner,'delta_h_inner');
            hScale = max(max(abs([data.oilh1legacy(:); data.oilh2legacy(:)])),1e-12);
            relativeUpdate = max(abs([deltaOuter-deltaOuterUsed; deltaInner-deltaInnerUsed])) / hScale;

            history(end+1) = struct('gamma',gamma,'iteration',iteration, ...
                'delta_h_outer_used',deltaOuterUsed,'delta_h_inner_used',deltaInnerUsed, ...
                'delta_h_outer_updated',deltaOuter,'delta_h_inner_updated',deltaInner, ...
                'relative_update',relativeUpdate); %#ok<AGROW>

            if gamma == 0
                break;
            end
            if relativeUpdate < 1e-4
                stableCount = stableCount + 1;
            else
                stableCount = 0;
            end
            if stableCount >= 2
                break;
            end
        end
        if gamma ~= 0 && maxOuter > 1 && stableCount < 2
            error('run_ball_staggered_feedback:NoConvergence', ...
                'OUT_OF_MODEL_DOMAIN: staggered feedback did not stabilize.');
        end
    end
    result = struct('delta_h_outer',deltaOuter,'delta_h_inner',deltaInner, ...
        'history',history,'states',states,'returndata',returndata);
    report = struct('success',true,'status','OK','message','OK');
catch exception
    result = struct('delta_h_outer',deltaOuter,'delta_h_inner',deltaInner, ...
        'history',history,'states',states,'returndata',returndata);
    report = struct('success',false,'status','OUT_OF_MODEL_DOMAIN', ...
        'message',getReport(exception,'basic','hyperlinks','off'));
end
end

function [targetOuter,targetInner,states] = roughness_targets(data,roughConfig,n)
required = {'loadj','loadi','Q1','Q2','aa1','b1','aa2','b2', ...
    'oilh1legacy','oilh2legacy','E1','E2','miuO','miuI'};
assert(all(isfield(data,required)), 'run_ball_staggered_feedback:Output', ...
    'The legacy mechanical solve did not save the required feedback fields.');
loadedIds = data.loadi(:);
assert(numel(loadedIds) == data.loadj && all(loadedIds >= 1) && ...
    all(loadedIds <= n) && all(loadedIds == floor(loadedIds)) && ...
    numel(unique(loadedIds)) == numel(loadedIds), ...
    'run_ball_staggered_feedback:BallMapping','Loaded-ball identifiers are not a unique valid mapping.');

targetOuter = zeros(n,1);
targetInner = zeros(n,1);
states = struct('outer',repmat(empty_state(),data.loadj,1), ...
    'inner',repmat(empty_state(),data.loadj,1),'ball_id',loadedIds);
for index = 1:data.loadj
    ballId = loadedIds(index);
    outerArea = pi*data.aa1(index)*data.b1(index);
    innerArea = pi*data.aa2(index)*data.b2(index);
    validate_contact_inputs(data.Q1(index),data.oilh1legacy(index),outerArea, ...
        data.miuO(index),'outer');
    validate_contact_inputs(data.Q2(index),data.oilh2legacy(index),innerArea, ...
        data.miuI(index),'inner');

    outerInput = struct('contactType','point','Qtotal',data.Q1(index), ...
        'filmInput',film_input(data.oilh1legacy(index),data.Q1(index)), ...
        'roughnessInput',roughness_input(roughConfig.Rq_outer, ...
            roughConfig.Rq_element,data.E1,outerArea,roughConfig.outer_pair), ...
        'muFluid',data.miuO(index),'muBoundary',roughConfig.outer_pair.muBoundary);
    innerInput = struct('contactType','point','Qtotal',data.Q2(index), ...
        'filmInput',film_input(data.oilh2legacy(index),data.Q2(index)), ...
        'roughnessInput',roughness_input(roughConfig.Rq_inner, ...
            roughConfig.Rq_element,data.E2,innerArea,roughConfig.inner_pair), ...
        'muFluid',data.miuI(index),'muBoundary',roughConfig.inner_pair.muBoundary);

    [outerState,outerReport] = roughness_solve_load_share(outerInput);
    [innerState,innerReport] = roughness_solve_load_share(innerInput);
    assert(outerReport.success && innerReport.success, ...
        'run_ball_staggered_feedback:LoadShare','Local load-sharing solve failed.');
    assert_physical_state(outerState,'outer');
    assert_physical_state(innerState,'inner');
    targetOuter(ballId) = outerState.hMix-data.oilh1legacy(index);
    targetInner(ballId) = innerState.hMix-data.oilh2legacy(index);
    states.outer(index) = outerState;
    states.inner(index) = innerState;
end
end

function value = option(input,name,defaultValue)
if isfield(input,name)
    value = input.(name);
else
    value = defaultValue;
end
end

function value = film_input(hAtQtotal,Qtotal)
value = struct('hAtQtotal',hAtQtotal,'loadExponent',-0.073,'Qtotal',Qtotal);
end

function value = roughness_input(surface1,surface2,Ered,area,pair)
value = struct('RqSurface1',surface1,'RqSurface2',surface2,'Ered',Ered, ...
    'nominalArea',area,'etaAsperity',pair.etaAsperity, ...
    'betaAsperity',pair.betaAsperity,'C_GT',pair.C_GT);
end

function validate_contact_inputs(Q,h,area,mu,side)
assert(all(isfinite([Q,h,area,mu])) && isreal(Q) && isreal(h) && ...
    isreal(area) && isreal(mu) && Q > 0 && h > 0 && area > 0 && mu >= 0, ...
    'run_ball_staggered_feedback:PhysicalState', ...
    'Invalid legacy %s contact output.',side);
end

function assert_physical_state(state,side)
fields = {'Qtotal','Qfluid','Qasperity','hMix','chiA','muFluid','muMix','loadBalanceError'};
values = cell2mat(cellfun(@(name)state.(name),fields,'UniformOutput',false));
assert(all(isfinite(values)) && isreal(values) && state.Qfluid >= 0 && ...
    state.Qasperity >= 0 && state.hMix > 0 && state.chiA >= 0 && state.chiA <= 1, ...
    'run_ball_staggered_feedback:PhysicalState', ...
    'Nonphysical %s roughness state.',side);
end

function assert_physical_vector(value,name)
assert(isreal(value) && all(isfinite(value)), ...
    'run_ball_staggered_feedback:PhysicalState','%s contains a nonfinite or complex value.',name);
end

function state = empty_state()
state = struct('Qtotal',[],'Qfluid',[],'Qasperity',[],'hMix',[], ...
    'lambda',[],'chiA',[],'muFluid',[],'muMix',[], ...
    'loadBalanceError',[],'iterations',[]);
end

function cleanup_workspace(old,work)
cd(old);
if isfolder(work)
    rmdir(work,'s');
end
end
