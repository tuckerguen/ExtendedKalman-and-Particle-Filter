% Setup problem/workspace configuration
config_ws();

% Run EKF over all timesteps
% Init storage variables
beliefs(1:9) = bel_0;
pre_bels(1:9) = bel_0;
xt_1 = x0;
bel = bel_0;

% Loop over all timesteps
for t = 1:n
    % Get command and measurement
    ut = all_ut(t);
    zt = all_zt(t);
    % Run EKf
    [pre_bel, bel] = EKF(bel, ut, a, zt, sig_r, sig_phi, m);
    beliefs(t+1) = bel;
    pre_bels(t+1) = pre_bel;
end

% Plot EKF run
figure('name', 'EKF_Plot');
true_plot = plot(true_xs, true_ys, '--r');
hold on;
[pre_plots, pre_legend] = plot_beliefs(pre_bels, true, 'black');
[post_plots, post_legend] = plot_beliefs(beliefs, false, 'cyan');
all_plots = [true_plot; pre_plots; post_plots];
all_legends = ["true noiseless pose"; pre_legend; post_legend];
legend(all_plots, all_legends);
legend(all_plots, all_legends);
hold off;











