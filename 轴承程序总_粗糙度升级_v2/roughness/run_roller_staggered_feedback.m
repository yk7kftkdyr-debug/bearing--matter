function step = run_roller_staggered_feedback(runDir,microConfig,previousState,gamma)
%RUN_ROLLER_STAGGERED_FEEDBACK Execute one frozen-film roller feedback step.
% This kernel intentionally contains neither a gamma loop nor process control.
% It calls the legacy mechanical solver exactly once, then evaluates one GT
% load-sharing problem per loaded roller and per contact side.

validateattributes(gamma,{'numeric'},{'real','finite','scalar','>=',0,'<=',1});
runDir = char(runDir);
assert(isfolder(runDir),'run_roller_staggered_feedback:RunDir','run_dir does not exist: %s',runDir);
addpath(runDir,'-begin');
pathCleanup = onCleanup(@()rmpath(runDir)); %#ok<NASGU>
assert(isstruct(microConfig) && isfield(microConfig,'roughness'), ...
    'run_roller_staggered_feedback:Config','micro_config.roughness is required.');
rough = microConfig.roughness;
assert(rough.enabled && rough.feedback_level == 1, ...
    'run_roller_staggered_feedback:Config','Roller normal feedback requires enabled feedback_level 1.');

n = defaultBearingInput();
n = n(1);
[deltaOuter,deltaInner,priorIds,priorQ1,priorQ2,restart] = previous_fields(previousState,n);
microConfig.roughness.feedback.roller.delta_h_outer = deltaOuter;
microConfig.roughness.feedback.roller.delta_h_inner = deltaInner;

mechanical = legacy_run_roller(runDir,microConfig,restart);
assert(mechanical.qiujieall_call_count == 1, ...
    'run_roller_staggered_feedback:CallCount','The legacy adapter must call qiujieall exactly once.');
data = extract_mechanical(mechanical,n);
[outer,inner,targetOuter,targetInner] = gt_update(data,rough,n,gamma);

nextOuter = targetOuter - smooth_by_id(data.roller_id,data.hSmoothOuter,n);
nextInner = targetInner - smooth_by_id(data.roller_id,data.hSmoothInner,n);
hScale = max([abs(data.hSmoothOuter(:));abs(data.hSmoothInner(:));1e-12]);
errH = max(abs([nextOuter-deltaOuter;nextInner-deltaInner])) / hScale;
errQ = load_change(data.roller_id,data.Q1,data.Q2,priorIds,priorQ1,priorQ2);
closureError = max([[outer.loadBalanceError] [inner.loadBalanceError]]);
stable = isempty(priorIds) || isequal(data.roller_id(:),priorIds(:));
physical = physical_contacts(outer) && physical_contacts(inner) && ...
    all(isfinite([data.Q1(:);data.Q2(:);data.hSmoothOuter(:);data.hSmoothInner(:); ...
                  mechanical.T1(:);mechanical.T2(:)])) && ...
    isreal([data.Q1(:);data.Q2(:);data.hSmoothOuter(:);data.hSmoothInner(:)]);

restartState = struct('roller_id',data.roller_id,'Q1',data.Q1,'Q2',data.Q2, ...
    'slice_load_outer',mechanical.slice_load_outer,'slice_load_inner',mechanical.slice_load_inner, ...
    'delta_h_outer',nextOuter,'delta_h_inner',nextInner);
step = struct('success',mechanical.converged && physical,'gamma',gamma, ...
    'qiujieall_call_count',mechanical.qiujieall_call_count, ...
    'mechanical_converged',mechanical.converged,'gt_converged',all([outer.solver_success]) && all([inner.solver_success]), ...
    'errQ',errQ,'errH',errH,'closure_error',closureError, ...
    'active_set_stable',stable,'physical_finite_real',physical, ...
    'Q1',data.Q1,'Q2',data.Q2,'oilh1',mechanical.oilh1,'oilh2',mechanical.oilh2, ...
    'T1',mechanical.T1,'T2',mechanical.T2,'loadj',mechanical.loadj, ...
    'roller_id',data.roller_id,'outer_to_inner_map',data.traction_location, ...
    'outer_contacts',outer,'inner_contacts',inner, ...
    'delta_h_outer',nextOuter,'delta_h_inner',nextInner,'restart_state',restartState, ...
    'legacy',mechanical);
end

function [dOuter,dInner,ids,q1,q2,restart] = previous_fields(previous,n)
if nargin < 1 || isempty(previous), previous = struct(); end
dOuter = field_or(previous,'delta_h_outer',zeros(n,1));
dInner = field_or(previous,'delta_h_inner',zeros(n,1));
ids = field_or(previous,'roller_id',[]); q1 = field_or(previous,'Q1',[]); q2 = field_or(previous,'Q2',[]);
restart = field_or(previous,'restart_state',struct());
validateattributes(dOuter,{'numeric'},{'real','finite','vector','numel',n});
validateattributes(dInner,{'numeric'},{'real','finite','vector','numel',n});
dOuter = dOuter(:); dInner = dInner(:); ids = ids(:); q1 = q1(:); q2 = q2(:);
assert((isempty(ids) && isempty(q1) && isempty(q2)) || ...
    (numel(ids) == numel(q1) && numel(ids) == numel(q2)), ...
    'run_roller_staggered_feedback:Previous','Previous roller state is inconsistent.');
end

function data = extract_mechanical(m,n)
t = m.traction;
terms = m.frozen_terms;
need = {'loadi','Q1','Q2','oilh1_legacy','oilh2_legacy','widthcontact11','widthcontact22','lenroller'};
assert(all(isfield(terms,need)),'run_roller_staggered_feedback:Snapshot','Missing frozen-film snapshot terms.');
ids = terms.loadi(:);
assert(numel(ids) == m.loadj && all(ids >= 1 & ids <= n & ids == floor(ids)) && ...
    numel(unique(ids)) == numel(ids),'run_roller_staggered_feedback:RollerId','Invalid loaded roller IDs.');
[found,loc] = ismember(ids,t.loadii(:));
assert(all(found) && numel(unique(t.loadii(:))) == numel(t.loadii), ...
    'run_roller_staggered_feedback:RollerMap','loadi to loadii is not one-to-one.');
qo = t.Q1ii(loc); qi = t.Q2ii(loc); qScale = max([abs(qo(:));abs(qi(:));1]); qTol = 1e-12*qScale;
assert(all(qo > qTol) && all(qi > qTol),'run_roller_staggered_feedback:Traction','Invalid T/Q denominator.');
muOuter = t.T1(loc)./qo; muInner = t.T2(loc)./qi;
assert(all(isfinite(muOuter) & isreal(muOuter) & muOuter >= 0) && ...
    all(isfinite(muInner) & isreal(muInner) & muInner >= 0), ...
    'run_roller_staggered_feedback:Traction','Invalid legacy T/Q traction coefficient.');
assert(size(terms.widthcontact11,1) == 150 && size(terms.widthcontact22,1) == 150, ...
    'run_roller_staggered_feedback:Slices','The legacy 150-slice mechanical integration is required.');
data = struct('roller_id',ids,'traction_location',loc(:),'Q1',terms.Q1(:),'Q2',terms.Q2(:), ...
    'hSmoothOuter',terms.oilh1_legacy(:),'hSmoothInner',terms.oilh2_legacy(:), ...
    'widthOuter',terms.widthcontact11,'widthInner',terms.widthcontact22, ...
    'contactLength',terms.lenroller,'Eouter',m.E1,'Einner',m.E2, ...
    'muOuter',muOuter(:),'muInner',muInner(:));
assert(numel(data.Q1) == m.loadj && numel(data.Q2) == m.loadj && ...
    numel(data.hSmoothOuter) == m.loadj && numel(data.hSmoothInner) == m.loadj, ...
    'run_roller_staggered_feedback:Snapshot','Snapshot dimensions do not match loaded roller count.');
end

function [outer,inner,targetOuter,targetInner] = gt_update(d,rough,n,gamma)
blank = contact_template();
outer = repmat(blank,numel(d.roller_id),1); inner = outer;
targetOuter = zeros(n,1); targetInner = zeros(n,1);
for k = 1:numel(d.roller_id)
    outer(k) = solve_contact(d,k,rough,'outer',gamma);
    inner(k) = solve_contact(d,k,rough,'inner',gamma);
    targetOuter(d.roller_id(k)) = outer(k).hUsed;
    targetInner(d.roller_id(k)) = inner(k).hUsed;
end
end

function c = solve_contact(d,k,rough,side,gamma)
if strcmp(side,'outer')
    Q = d.Q1(k); h = d.hSmoothOuter(k); width = d.widthOuter(:,k); E = d.Eouter;
    Rq = rough.Rq_outer; pair = rough.outer_pair; mu = d.muOuter(k); source = 'legacy_T1_over_Q1ii';
else
    Q = d.Q2(k); h = d.hSmoothInner(k); width = d.widthInner(:,k); E = d.Einner;
    Rq = rough.Rq_inner; pair = rough.inner_pair; mu = d.muInner(k); source = 'legacy_T2_over_Q2ii';
end
assert(Q > 0 && h > 0 && isfinite(Q) && isfinite(h) && isreal(Q) && isreal(h), ...
    'run_roller_staggered_feedback:Physical','Invalid %s contact state.',side);
assert(all(width(:) > 0 & isfinite(width(:)) & isreal(width(:))), ...
    'run_roller_staggered_feedback:Area','Invalid %s contact widths.',side);
area = d.contactLength/150 * sum(2*width(:));
in = struct('contactType','line','Qtotal',Q, ...
    'filmInput',struct('hAtQtotal',h,'loadExponent',-0.13,'Qtotal',Q), ...
    'roughnessInput',struct('RqSurface1',Rq,'RqSurface2',rough.Rq_element, ...
        'Ered',E,'nominalArea',area,'etaAsperity',pair.etaAsperity, ...
        'betaAsperity',pair.betaAsperity,'C_GT',pair.C_GT), ...
    'muFluid',mu,'muBoundary',pair.muBoundary);
[s,r] = roughness_solve_load_share(in);
assert(r.success,'run_roller_staggered_feedback:LoadShare','%s load sharing failed: %s',side,r.status);
hUsed = h + gamma*(s.hMix-h);
c = struct('roller_id',d.roller_id(k),'Qtotal',s.Qtotal,'Qfluid',s.Qfluid, ...
    'Qasperity',s.Qasperity,'hSmooth',h,'hMix',s.hMix,'hUsed',hUsed, ...
    'lambda',s.lambda,'chiA',s.chiA,'loadBalanceError',s.loadBalanceError, ...
    'solver_success',r.success,'muFluid',s.muFluid,'muMix',s.muMix, ...
    'muFluidSource',source);
end

function value = smooth_by_id(ids,h,n)
value = zeros(n,1); value(ids(:)) = h(:);
end

function err = load_change(ids,q1,q2,oldIds,oldQ1,oldQ2)
if isempty(oldIds), err = 0; return; end
if ~isequal(ids(:),oldIds(:)), err = Inf; return; end
now = [q1(:);q2(:)]; before = [oldQ1(:);oldQ2(:)];
err = max(abs(now-before)./max(abs(before),1));
end

function ok = physical_contacts(c)
for k = 1:numel(c)
    v = [c(k).Qtotal c(k).Qfluid c(k).Qasperity c(k).hSmooth c(k).hMix c(k).hUsed c(k).chiA c(k).loadBalanceError];
    validLambda = (isfinite(c(k).lambda) && c(k).lambda >= 0) || isinf(c(k).lambda);
    if ~all(isfinite(v)) || ~isreal(v) || ~isreal(c(k).lambda) || ~validLambda || c(k).Qfluid < 0 || c(k).Qasperity < 0 || ...
            c(k).hMix <= 0 || c(k).hUsed <= 0 || c(k).chiA < 0 || c(k).chiA > 1 || ...
            c(k).loadBalanceError >= 1e-6 || ~c(k).solver_success
        ok = false; return;
    end
end
ok = true;
end

function c = contact_template()
c = struct('roller_id',[],'Qtotal',[],'Qfluid',[],'Qasperity',[],'hSmooth',[], ...
    'hMix',[],'hUsed',[],'lambda',[],'chiA',[],'loadBalanceError',[], ...
    'solver_success',false,'muFluid',[],'muMix',[],'muFluidSource','');
end

function value = field_or(s,name,defaultValue)
if isfield(s,name) && ~isempty(s.(name)), value = s.(name); else, value = defaultValue; end
end
