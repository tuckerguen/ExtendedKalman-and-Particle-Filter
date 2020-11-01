% Setup problem/workspace configuration
config_ws();

% Particle Filter Initialization
M = 200;
% Generate set of random particles
X0(1:M, 1) = Particle(State(0,0,0), 0);
xs = rand_range(0, 8, M);
ys = rand_range(0, 6, M);
thetas = rand_range(0, 2*pi, M);
for i = 1:M
    rand_state = State(xs(i), ys(i), thetas(i));
    X0(i) = Particle(rand_state, 1/M); % Uniform weights
end

%Run particle filter 
% Initial particle set/true pose
Xt = X0;
xt_1 = x0;

% Array to store intermediate particle sets
pre_update_cov = [Xt'];
post_update_cov = [Xt'];
% Loop over all timesteps
for t = 1:n
    % Get command and measurement
    ut = all_ut(t);
    zt = all_zt(t);
    % Run particle filter
    [Xt_b, Xt] = particle_filter(Xt, ut, a, zt, sig_r, sig_phi, m);
    pre_update_cov = [pre_update_cov; Xt_b';];
    post_update_cov = [post_update_cov; Xt';];
end

%Plot particles for each timestep
figure('name', 'PF Plot');
pt_size = 5;

pre_pxs = zeros(1,M);
pre_pys = zeros(1,M);

post_pxs = zeros(1,M);
post_pys = zeros(1,M);

plot(true_xs, true_ys, '--r');
hold on;

for j=1:9
    pre_update_particles = pre_update_cov(j,:);
    post_update_particles = post_update_cov(j,:);
    for i=1:M
        pre_p = pre_update_particles(i);
        pre_pxs(i) = pre_p.x.x;
        pre_pys(i) = pre_p.x.y;
        
        post_p = post_update_particles(i);
        post_pxs(i) = post_p.x.x;
        post_pys(i) = post_p.x.y;
    end
    scatter(pre_pxs, pre_pys, pt_size, 'k', 'filled');
    scatter(post_pxs, post_pys, pt_size, 'c', 'filled');
end

legend('true, noiseless pose', 'pre-update particles', 'post-update particles');