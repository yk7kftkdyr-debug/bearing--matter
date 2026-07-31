function test_local_contact_solver(projectRoot)
if nargin < 1, projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath')))); end
addpath(projectRoot); addpath(fullfile(projectRoot,'roughness'));
pair = struct('RqSurface1',0,'RqSurface2',0,'Ered',2e11,'nominalArea',1e-6, ...
    'etaAsperity',1e10,'betaAsperity',1e-6,'C_GT',1,'muFluid',0.05,'muBoundary',0.12);
geom = struct('K',1e9); lubricant = struct('filmInput',@(Q) 0*Q);
[s,r] = solve_point_contact_from_gap(1e-4,geom,lubricant,pair,struct(),struct());
assert(r.converged && strcmp(r.status,'CONVERGED_STRICT'));
assert(abs(s.Q - geom.K*(1e-4)^(3/2))/s.Q < 1e-6);
assert(abs(s.Qfluid+s.Qasperity-s.Q)/s.Q < 1e-6);
assert(all(isfinite([s.Q s.Qfluid s.Qasperity s.h s.chiA])) && isreal(s.lambda));
previous=s;
for gap=[1.01e-4 1.02e-4]
    [next,rr]=solve_point_contact_from_gap(gap,geom,lubricant,pair,previous,struct());
    assert(rr.converged && next.Q>previous.Q && isfinite(next.Q));
    previous=next;
end
pair.RqSurface1=1e-9;
[rough,rr]=solve_point_contact_from_gap(1e-4,geom,lubricant,pair,s,struct());
assert(rr.converged && rough.Q>0 && abs(rough.Qfluid+rough.Qasperity-rough.Q)/rough.Q < 1e-6);
end
