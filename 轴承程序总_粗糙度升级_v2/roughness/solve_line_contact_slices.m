function [state,report] = solve_line_contact_slices(deltaGeomSlices,lineGeom,lubricant,roughPair,prevState,opts)
%SOLVE_LINE_CONTACT_SLICES Stage 4A line-contact interface; no roller integration.
validateattributes(deltaGeomSlices,{'numeric'},{'real','finite','vector'});
assert(isstruct(lineGeom) && isfield(lineGeom,'K'),'solve_line_contact_slices:Input','lineGeom.K is required.');
K=sum(lineGeom.K(:)); delta=max(deltaGeomSlices(:));
[state,report]=solve_point_contact_from_gap(delta,struct('K',K),lubricant,roughPair,prevState,opts);
end
