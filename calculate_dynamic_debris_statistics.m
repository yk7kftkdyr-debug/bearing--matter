function stats = calculate_dynamic_debris_statistics(datafromvb, ballCount, cageSpeed, contact, debris, contactSolver)
%CALCULATE_DYNAMIC_DEBRIS_STATISTICS Build cage-period load statistics.
%
% The paper ud law still receives w_norm=(Q2-w_mean)/sigma_w. This helper
% changes only how w_mean/sigma_w are estimated. At every cage phase the
% original baseline contact solver is called again to obtain Q2_i(theta).

if nargin < 6 || ~isa(contactSolver, 'function_handle')
    error('calculate_dynamic_debris_statistics:InvalidInput', ...
        'A contact-solver function handle is required for dynamic resampling.');
end

N_theta = debris.N_theta;
if ~isfinite(N_theta) || N_theta < 2
    N_theta = 360;
end
N_theta = max(2, round(N_theta));
ballCount = max(1, round(ballCount));

theta = linspace(0, 2*pi, N_theta).';
Q_samples = zeros(N_theta, ballCount);
for k = 1:N_theta
    phaseContact = contactSolver(theta(k));
    if ~isstruct(phaseContact) || ~isfield(phaseContact, 'loadi') || ~isfield(phaseContact, 'Q2')
        error('calculate_dynamic_debris_statistics:InvalidSolverOutput', ...
            'Contact solver returned no loadi/Q2 data at theta index %d.', k);
    end
    validIds = phaseContact.loadi(:).' >= 1 & phaseContact.loadi(:).' <= ballCount;
    ids = phaseContact.loadi(validIds);
    loads = phaseContact.Q2(validIds);
    Q_samples(k, ids) = loads;
end

finiteLoadedMask = isfinite(Q_samples) & Q_samples > debris.Q_floor;
validMask = finiteLoadedMask & Q_samples <= 8000;
w_ref = Q_samples(validMask);

stats = struct();
stats.theta = theta;
stats.Q_samples = Q_samples;
stats.valid_mask = validMask;
stats.w_ref = w_ref(:);
stats.Q_sample_count = numel(stats.w_ref);
stats.Q_out_of_domain_count = nnz(finiteLoadedMask & Q_samples > 8000);
stats.Q_nonfinite_count = nnz(~isfinite(Q_samples));
stats.cage_speed_rad_s = cageSpeed;
stats.N_theta = N_theta;
stats.N_ball = ballCount;
stats.Q_floor = debris.Q_floor;
stats.bearing_params = datafromvb;
if isstruct(contact) && isfield(contact, 'Q2')
    stats.reference_loaded_count = nnz(isfinite(contact.Q2) & contact.Q2 > debris.Q_floor);
else
    stats.reference_loaded_count = NaN;
end

if stats.Q_sample_count >= 2
    stats.w_mean = mean(stats.w_ref);
    stats.sigma_Q = std(stats.w_ref, 0);
    stats.Q_min = min(stats.w_ref);
    stats.Q_max = max(stats.w_ref);
else
    stats.w_mean = NaN;
    stats.sigma_Q = NaN;
    stats.Q_min = NaN;
    stats.Q_max = NaN;
end

if isfinite(stats.sigma_Q) && stats.sigma_Q > debris.sigma_floor
    w_norm = (stats.w_ref - stats.w_mean) ./ stats.sigma_Q;
    stats.w_norm_min = min(w_norm);
    stats.w_norm_max = max(w_norm);
else
    stats.w_norm_min = NaN;
    stats.w_norm_max = NaN;
end

stats.method = 'cage_period_contact_resolve';
stats.position_formula = 'phi_i(theta)=2*pi*i/N_ball + theta';
end
