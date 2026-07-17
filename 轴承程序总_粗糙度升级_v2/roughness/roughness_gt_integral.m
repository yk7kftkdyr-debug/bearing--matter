function value = roughness_gt_integral(lambda,order)
%ROUGHNESS_GT_INTEGRAL Greenwood--Tripp standard-normal tail integral.
validateattributes(lambda,{'numeric'},{'real','finite'});
assert(isscalar(order)&&(order==2||order==2.5),'roughness_gt_integral:Order','order must be 2 or 2.5.');
value=arrayfun(@(x) integral(@(s)(s-x).^order.*exp(-0.5*s.^2)/sqrt(2*pi),x,Inf,'RelTol',1e-10,'AbsTol',1e-14),lambda);
assert(all(isfinite(value(:))&value(:)>=0),'roughness_gt_integral:InvalidOutput','Integral must be finite and nonnegative.');
end
