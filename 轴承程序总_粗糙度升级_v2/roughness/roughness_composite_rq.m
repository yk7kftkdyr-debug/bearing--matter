function sigmaComposite = roughness_composite_rq(RqSurface1,RqSurface2)
%ROUGHNESS_COMPOSITE_RQ Composite RMS roughness; inputs are metres.
validateattributes(RqSurface1,{'numeric'},{'real','finite','nonnegative'});
validateattributes(RqSurface2,{'numeric'},{'real','finite','nonnegative'});
assert(isscalar(RqSurface1)||isscalar(RqSurface2)||isequal(size(RqSurface1),size(RqSurface2)), ...
    'roughness_composite_rq:Size','Inputs must be scalar or same-sized arrays.');
sigmaComposite=sqrt(RqSurface1.^2+RqSurface2.^2);
end
