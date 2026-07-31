function diagnostic = ball_roughness_diagnostic(legacy,cfg)
%BALL_ROUGHNESS_DIAGNOSTIC Stage 4B Level-0 read-only ball contact diagnostic.
assert(isfield(cfg,'roughness') && cfg.roughness.feedback_level==0, ...
    'ball_roughness_diagnostic:Config','Level 0 diagnostics require feedback_level=0.');
required={'Q','delta','h','roughPair'};
for k=1:numel(required), assert(isfield(legacy,required{k}),'ball_roughness_diagnostic:Input','Missing %s.',required{k}); end
n=numel(legacy.Q); diagnostic=repmat(empty(),n,1);
for k=1:n
    q=legacy.Q(k); d=legacy.delta(k);
    if ~(isfinite(q)&&q>0&&isfinite(d)&&d>0), continue; end
    g=struct('K',q/d^1.5); l=struct('filmInput',@(x) legacy.h(k));
    [s,r]=solve_point_contact_from_gap(d,g,l,legacy.roughPair,struct('Q',q),struct());
    assert(r.converged,'ball_roughness_diagnostic:Local','Local contact diagnostic failed.');
    diagnostic(k)=struct('h',s.h,'lambda',s.lambda,'Qfluid',s.Qfluid,'Qasperity',s.Qasperity, ...
        'chiA',s.chiA,'muMix',mix_mu(s.chiA,legacy.roughPair),'kEq',1.5*s.Q/d, ...
        'status',s.status);
end
end
function x=empty(), x=struct('h',NaN,'lambda',NaN,'Qfluid',NaN,'Qasperity',NaN,'chiA',NaN,'muMix',NaN,'kEq',NaN,'status','NO_CONTACT'); end
function mu=mix_mu(chi,p), mu=(1-chi)*p.muFluid-chi*p.muBoundary; end
