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
omega = option(roughConfig,'omega',option(roughConfig,'relaxation',0.5));
errQTol = option(roughConfig,'errQ_tol',1e-3);
errHTol = option(roughConfig,'errH_tol',1e-4);
closureTol = option(roughConfig,'closure_tol',1e-6);
assert(isnumeric(gammaSequence) && isreal(gammaSequence) && ...
    all(isfinite(gammaSequence)) && all(gammaSequence >= 0) && ...
    all(gammaSequence <= 1), 'run_ball_staggered_feedback:Gamma', ...
    'gamma_sequence must lie in [0,1].');
assert(isscalar(maxOuter) && maxOuter >= 1 && maxOuter == floor(maxOuter), ...
    'run_ball_staggered_feedback:Iterations','max_outer_iterations must be a positive integer.');
assert(isscalar(omega) && omega > 0 && omega <= 1, ...
    'run_ball_staggered_feedback:Relaxation','relaxation must lie in (0,1].');
assert(all(cellfun(@(x) isscalar(x) && isfinite(x) && x >= 0, ...
    {errQTol,errHTol,closureTol})), ...
    'run_ball_staggered_feedback:Tolerances','Feedback tolerances must be finite nonnegative scalars.');

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
lastData = struct();
previousMechanical = struct('ball_id',[],'Qouter',[],'Qinner',[]);
metrics = struct('gamma_final',NaN,'outer_iterations',0,'errQ',NaN, ...
    'errH',NaN,'closure_error',NaN,'active_set_stable',false, ...
    'outer_converged',false,'closure_pass',false,'load_share_pass',false);
traction = empty_traction();

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
            [errQ,activeSetStable] = mechanical_change(data,previousMechanical);
            previousMechanical = struct('ball_id',data.loadi(:), ...
                'Qouter',data.Q1(:),'Qinner',data.Q2(:));

            deltaOuter = (1-omega)*deltaOuterUsed + omega*gamma*targetOuter;
            deltaInner = (1-omega)*deltaInnerUsed + omega*gamma*targetInner;
            assert_physical_vector(deltaOuter,'delta_h_outer');
            assert_physical_vector(deltaInner,'delta_h_inner');
            hScale = max(max(abs([data.oilh1legacy(:); data.oilh2legacy(:)])),1e-12);
            relativeUpdate = max(abs([deltaOuter-deltaOuterUsed; deltaInner-deltaInnerUsed])) / hScale;
            closureError = max(abs([states.outer.loadBalanceError, ...
                states.inner.loadBalanceError]));
            metrics = struct('gamma_final',gamma,'outer_iterations',iteration, ...
                'errQ',errQ,'errH',relativeUpdate,'closure_error',closureError, ...
                'active_set_stable',activeSetStable,'outer_converged',false, ...
                'closure_pass',closureError <= closureTol, ...
                'load_share_pass',closureError <= 1e-6);
            lastData = data;

            history(end+1) = struct('gamma',gamma,'iteration',iteration, ...
                'delta_h_outer_used',deltaOuterUsed,'delta_h_inner_used',deltaInnerUsed, ...
                'delta_h_outer_updated',deltaOuter,'delta_h_inner_updated',deltaInner, ...
                'relative_update',relativeUpdate); %#ok<AGROW>

            if gamma == 0
                metrics.outer_converged = true;
                break;
            end
            if errQ <= errQTol && relativeUpdate <= errHTol && ...
                    closureError <= closureTol && activeSetStable
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
    if ~isempty(history) && history(end).gamma ~= 0
        metrics.outer_converged = metrics.outer_converged || stableCount >= 2;
    end
    if option(roughConfig,'feedback_level',1) == 2
        traction = run_frozen_traction(input,roughConfig,lastData,states, ...
            deltaOuter,deltaInner,n);
        assert(traction.normal_state_unchanged, ...
            'run_ball_staggered_feedback:NormalMutation', ...
            'Frozen traction feedback changed the Stage 2 normal state.');
    end
    result = struct('delta_h_outer',deltaOuter,'delta_h_inner',deltaInner, ...
        'history',history,'states',states,'returndata',returndata, ...
        'legacy',legacy_snapshot(lastData),'metrics',metrics,'traction',traction);
    report = struct('success',true,'status','OK','message','OK');
catch exception
    result = struct('delta_h_outer',deltaOuter,'delta_h_inner',deltaInner, ...
        'history',history,'states',states,'returndata',returndata, ...
        'legacy',legacy_snapshot(lastData),'metrics',metrics,'traction',traction);
    report = struct('success',false,'status','OUT_OF_MODEL_DOMAIN', ...
        'message',getReport(exception,'basic','hyperlinks','off'));
end

function traction = run_frozen_traction(input,roughConfig,data,states,deltaOuter,deltaInner,n)
% Apply only frozen traction magnitudes in the pre-existing speed residual.
% The Stage 2 normal result above is complete before this function runs.
assert(~isempty(fieldnames(data)) && isfield(states,'ball_id'), ...
    'run_ball_staggered_feedback:Traction','No converged normal state is available for Level 2 traction.');
legacy = legacy_traction_state(data);
normal = struct('ball_id',states.ball_id(:),'outer',states.outer,'inner',states.inner);
normalSnapshot = capture_normal_snapshot(data);
sequence = option(roughConfig,'gamma_mu_sequence',[0 0.5 1]);
assert(isnumeric(sequence) && isreal(sequence) && all(isfinite(sequence)) && ...
    all(sequence >= 0) && all(sequence <= 1) && sequence(1) == 0 && sequence(end) == 1, ...
    'run_ball_staggered_feedback:TractionGamma', ...
    'gamma_mu_sequence must begin at zero and end at one.');
for index = 1:numel(sequence)
    gammaMu = sequence(index);
    state = build_ball_traction_feedback_state(normal,legacy,gammaMu);
    assert(state.finite,'run_ball_staggered_feedback:TractionPhysical', ...
        'Mixed traction state is nonphysical.');
    micro_config = make_micro_interface_config(struct('roughness',roughConfig));
    micro_config.roughness.enabled = true;
    % Level 1 keeps the already-validated frozen normal-film path active;
    % the nested traction switch is the sole Level-2 addition to ffSPEED1.
    micro_config.roughness.feedback_level = 1;
    micro_config.roughness.feedback.ball.delta_h_outer = deltaOuter;
    micro_config.roughness.feedback.ball.delta_h_inner = deltaInner;
    micro_config.roughness.feedback.ball.traction = struct('enabled',true, ...
        'ball_id',state.ball_id(:),'Tused_outer',by_ball_id(state.outer,n,'Tused'), ...
        'Tused_inner',by_ball_id(state.inner,n,'Tused'),'gamma_mu',gammaMu);
    micro_config.roughness.feedback.ball.normal_snapshot = normalSnapshot;
    save('micro_config_runtime.mat','micro_config');
    evalc('qiujieSPEED1(input);');
    assert_speed_state();
    restore_normal_snapshot(normalSnapshot);
end
after = load('q1q2a1a2.mat');
traction = state;
traction.gamma_mu_final = sequence(end);
traction.speed_residual = load_scalar('result1134.mat','result1134');
traction.limit_pass = smooth_limit(state);
[traction.normal_state_unchanged,traction.normal_state_diagnostic] = ...
    same_normal_state(data,after);
end

function snapshot = capture_normal_snapshot(data)
% Level 2 operates on an immutable Stage 2 normal solution.  This local
% snapshot is passed to ffSPEED1 and restored after each speed iteration.
required = {'loadi','loadj','q1q2a1a2','Q1','Q2','a1','a2','oilh1','oilh2'};
assert(all(isfield(data,required)), ...
    'run_ball_staggered_feedback:NormalSnapshot', ...
    'Final Stage 2 output lacks a required normal-state field.');
snapshot = struct();
for index = 1:numel(required)
    name = required{index};
    snapshot.(name) = data.(name);
end
snapshot.Ph1 = load_required_field('Ph1.mat','Ph1');
snapshot.Ph2 = load_required_field('Ph2.mat','Ph2');
snapshot.aa1 = load_required_field('aa1.mat','aa1');
snapshot.aa2 = load_required_field('aa2.mat','aa2');
end

function restore_normal_snapshot(snapshot)
% Do not leave a speed-only Level 2 evaluation with altered normal outputs.
q1q2a1a2 = snapshot.q1q2a1a2; %#ok<NASGU>
Q1 = snapshot.Q1; Q2 = snapshot.Q2; %#ok<NASGU>
a1 = snapshot.a1; a2 = snapshot.a2; %#ok<NASGU>
oilh1 = snapshot.oilh1; oilh2 = snapshot.oilh2; %#ok<NASGU>
Ph1 = snapshot.Ph1; Ph2 = snapshot.Ph2; %#ok<NASGU>
aa1 = snapshot.aa1; aa2 = snapshot.aa2; %#ok<NASGU>
loadi = snapshot.loadi; loadj = snapshot.loadj; %#ok<NASGU>
save('q1q2a1a2.mat','q1q2a1a2','Q1','Q2','a1','a2','oilh1','oilh2','loadi','loadj');
save('oilh1.mat','oilh1'); save('oilh2.mat','oilh2');
save('Ph1.mat','Ph1'); save('Ph2.mat','Ph2');
save('aa1.mat','aa1'); save('aa2.mat','aa2');
end

function value = load_required_field(fileName,fieldName)
assert(isfile(fileName), 'run_ball_staggered_feedback:NormalSnapshot', ...
    'Missing frozen normal output %s.',fileName);
source = load(fileName,fieldName);
assert(isfield(source,fieldName), 'run_ball_staggered_feedback:NormalSnapshot', ...
    'Missing frozen normal field %s.',fieldName);
value = source.(fieldName);
end

function legacy = legacy_traction_state(data)
required = {'loadi','loadj'};
assert(all(isfield(data,required)), ...
    'run_ball_staggered_feedback:Traction','Missing loaded-ball map for traction feedback.');
legacy = struct('ball_id',data.loadi(:), ...
    'Touter',load_vector('T1.mat','T1',data.loadj), ...
    'Tinner',load_vector('T2.mat','T2',data.loadj), ...
    'slip_outer',load_vector('deltaU1.mat','deltaU1',data.loadj), ...
    'slip_inner',load_vector('deltaU2.mat','deltaU2',data.loadj));
end

function value = load_vector(fileName,fieldName,count)
assert(isfile(fileName),'run_ball_staggered_feedback:Traction', ...
    'Missing legacy traction file %s.',fileName);
source = load(fileName);
assert(isfield(source,fieldName),'run_ball_staggered_feedback:Traction', ...
    'Missing %s in %s.',fieldName,fileName);
value = source.(fieldName)(:);
assert(numel(value) == count && all(isfinite(value)) && isreal(value) && ...
    all(value > 0),'run_ball_staggered_feedback:Traction', ...
    'Invalid %s for the loaded-ball traction map.',fieldName);
end

function values = by_ball_id(contacts,n,fieldName)
values = zeros(n,1);
for index = 1:numel(contacts)
    ballId = contacts(index).ball_id;
    assert(ballId >= 1 && ballId <= n && values(ballId) == 0, ...
        'run_ball_staggered_feedback:BallMapping','Invalid or duplicate ball identifier.');
    values(ballId) = contacts(index).(fieldName);
end
end

function assert_speed_state()
residual = load_scalar('result1134.mat','result1134');
speed = load_scalar('wwmin34.mat','wwmin34');
assert(~isempty(residual) && isscalar(residual) && isfinite(residual) && ...
    ~isempty(speed) && all(isfinite(speed(:))) && isreal(speed), ...
    'run_ball_staggered_feedback:TractionConvergence', ...
    'Legacy speed solve did not return a finite frozen-traction state.');
end

function passed = smooth_limit(traction)
outer = traction.outer;
inner = traction.inner;
passed = all(abs([outer.muMix]-[outer.muFluid]) <= 1e-12) && ...
    all(abs([inner.muMix]-[inner.muFluid]) <= 1e-12) && ...
    all(abs([outer.Tused]-[outer.Tlegacy]) <= 1e-12) && ...
    all(abs([inner.Tused]-[inner.Tlegacy]) <= 1e-12);
end

function [passed,diagnostic] = same_normal_state(before,after)
% Preserve field-level evidence before the Level-2 safety assertion.
% q1q2a1a2.mat is written by ffLOAD in both the final normal solve and
% every ffSPEED1 residual evaluation; the two paths must be distinguishable.
fields = {'Q1','Q2','a1','a2'};
passed = true;
diagnostic = struct('tolerance',1e-12, ...
    'before_source','final Stage 2 normal ffLOAD output (lastData)', ...
    'after_source','qiujieSPEED1 -> ffSPEED1 -> ffLOAD(datafromvb,loadi,www3) -> q1q2a1a2.mat', ...
    'all_within_tolerance',false);
for index = 1:numel(fields)
    name = fields{index};
    assert(isfield(before,name) && isfield(after,name), ...
        'run_ball_staggered_feedback:NormalState','Missing %s after traction solve.',name);
    a = before.(name)(:); b = after.(name)(:);
    item = struct('before_source',diagnostic.before_source, ...
        'after_source',diagnostic.after_source,'before_count',numel(a), ...
        'after_count',numel(b),'size_equal',isequal(size(a),size(b)), ...
        'max_absolute_difference',Inf,'max_relative_difference',Inf, ...
        'exceeds_tolerance_count',Inf,'within_tolerance',false);
    if item.size_equal
        difference = abs(a-b);
        relative = difference ./ max(abs(a),1);
        item.max_absolute_difference = max(difference);
        item.max_relative_difference = max(relative);
        item.exceeds_tolerance_count = nnz(relative > diagnostic.tolerance);
        item.within_tolerance = item.exceeds_tolerance_count == 0;
    end
    diagnostic.(name) = item;
    if ~item.within_tolerance
        passed = false;
    end
end
diagnostic.all_within_tolerance = passed;
end

function traction = empty_traction()
traction = struct('ball_id',[],'gamma_mu',NaN,'gamma_mu_final',NaN, ...
    'outer',[],'inner',[],'finite',false,'speed_residual',NaN, ...
    'limit_pass',false,'normal_state_unchanged',false);
end

function [errQ,activeSetStable] = mechanical_change(data,previous)
if isempty(previous.ball_id)
    errQ = 0;
    activeSetStable = true;
    return;
end
ids = data.loadi(:);
activeSetStable = isequal(ids,previous.ball_id);
if ~activeSetStable
    errQ = Inf;
    return;
end
current = [data.Q1(:); data.Q2(:)];
old = [previous.Qouter(:); previous.Qinner(:)];
errQ = max(abs(current-old) ./ max(abs(old),1));
end

function legacy = legacy_snapshot(data)
legacy = struct('Q1',[],'Q2',[],'oilh1',[],'oilh2',[],'a1',[],'a2',[], ...
    'Ph1',[],'Ph2',[],'kk',[],'deltaw',[],'loadj',NaN);
if isempty(fieldnames(data))
    return;
end
fields = {'Q1','Q2','oilh1','oilh2','a1','a2','Ph1','Ph2','loadj'};
for index = 1:numel(fields)
    name = fields{index};
    if isfield(data,name)
        legacy.(name) = data.(name);
    end
end
legacy.kk = load_scalar('kk.mat','kk');
legacy.deltaw = load_scalar('deltaw.mat','deltaw');
end

function value = load_scalar(fileName,fieldName)
value = [];
if isfile(fileName)
    data = load(fileName);
    if isfield(data,fieldName)
        value = data.(fieldName);
    end
end
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
