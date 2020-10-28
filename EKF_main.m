%% Setup problem/workspace configuration
config_ws();
%% Run EKF over all timesteps
% Init storage variables
beliefs(1:9) = bel_0;
xt_1 = x0;
bel = bel_0;

% Loop over all timesteps
for t = 1:n
    % Get command and measurement
    ut = all_ut(t);
    zt = all_zt(t);
    % Run EKf
    bel = EKF(bel, ut, a, zt, sig_r, sig_phi, m);
    beliefs(t+1) = bel;
end

%% Collect state means and covariances
covs = zeros(3, 3, n+1);
locs = zeros(n+1, 3);
for i = 1:n+1
    bel = beliefs(i);
    loc = get_loc(bel);
    locs(i,:) = loc';
    cov = bel.Cov;
    covs(:,:,i) = cov;
end

%% Plot EKF run
close all;
locxs = locs(:,1);
locys = locs(:,2);

plot(true_xs, true_ys, 'm');
hold on;
plot(locxs, locys, 'c');

for i=i:n+1
    error_ellipse(covs(:,:,i),get_vec(beliefs(i).mu));
end

legend('true pose no noise', 'true pose w/ noise', 'estimated pose');











