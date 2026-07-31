function [candidate,report] = ball_roughness_normal_candidate(snapshot)
%BALL_ROUGHNESS_NORMAL_CANDIDATE Read-only Stage 4B-1 local normal candidate.
required={'loadedElementIds','deltaGeomInner','deltaGeomOuter','legacyQInner','legacyQOuter', ...
    'innerPointGeometry','outerPointGeometry','innerLubricant','outerLubricant', ...
    'innerRoughPair','outerRoughPair'};
for k=1:numel(required)
    assert(isfield(snapshot,required{k}),'ball_roughness_normal_candidate:Input','Missing %s.',required{k});
end
n=numel(snapshot.loadedElementIds);
vectors={snapshot.deltaGeomInner,snapshot.deltaGeomOuter,snapshot.legacyQInner,snapshot.legacyQOuter, ...
    snapshot.innerPointGeometry.K,snapshot.outerPointGeometry.K,snapshot.innerLubricant.h,snapshot.outerLubricant.h};
assert(all(cellfun(@numel,vectors)==n),'ball_roughness_normal_candidate:Input','Contact arrays must match loadedElementIds.');
candidate=struct('inner',empty_side(n),'outer',empty_side(n));
converged=true; maxBalance=0;
for k=1:n
    [si,ri]=solve_one(snapshot.deltaGeomInner(k),snapshot.innerPointGeometry.K(k), ...
        snapshot.innerLubricant.h(k),snapshot.innerRoughPair,snapshot.legacyQInner(k));
    [so,ro]=solve_one(snapshot.deltaGeomOuter(k),snapshot.outerPointGeometry.K(k), ...
        snapshot.outerLubricant.h(k),snapshot.outerRoughPair,snapshot.legacyQOuter(k));
    candidate.inner=assign(candidate.inner,k,si);
    candidate.outer=assign(candidate.outer,k,so);
    converged=converged && ri.converged && ro.converged;
    maxBalance=max([maxBalance ri.load_balance_error ro.load_balance_error]);
end
if converged, status='STAGE4B_LEVEL1_CANDIDATE_PASS'; else, status='STAGE4B_LEVEL1_CANDIDATE_HOLD'; end
report=struct('status',status,'converged',converged,'contact_count',n,'max_load_balance_error',maxBalance);
end

function [s,r]=solve_one(delta,K,h,pair,qLegacy)
geometry=struct('K',K); lubricant=struct('filmInput',@(q) h+0*q);
[s,r]=solve_point_contact_from_gap(delta,geometry,lubricant,pair,struct('Q',qLegacy),struct());
end
function s=empty_side(n)
s=struct('Q',nan(1,n),'h',nan(1,n),'lambda',nan(1,n),'Qfluid',nan(1,n), ...
    'Qasperity',nan(1,n),'chiA',nan(1,n));
end
function out=assign(out,k,in)
out.Q(k)=in.Q; out.h(k)=in.h; out.lambda(k)=in.lambda; out.Qfluid(k)=in.Qfluid;
out.Qasperity(k)=in.Qasperity; out.chiA(k)=in.chiA;
end
