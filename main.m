%% Include files from subdirectories
p = genpath('./');
addpath(p);

%% Problem Configuration
% Landmarks
l1 = Landmark(0, 0, 1);
l2 = Landmark(4, 0, 2);
l3 = Landmark(8, 0, 3);
l4 = Landmark(8, 6, 4);
l5 = Landmark(4, 6, 5);
l6 = Landmark(0, 6, 6);
IDs = [1, 2, 3, 4, 5, 6];
landmarks = {l1, l2, l3, l4, l5, l6};

% Map
m = Map(IDs, landmarks);

% Initial State 
x0 = State(2, 2, 0);

% Initial State as a belief
mu_0 = StateMean(2, 2, 0);
Cov_0 = zeros(3, 3, 'double');
bel_0 = StateBelief(mu_0, Cov_0);

% Number of time steps
n = 8;

% List of commands
all_ut = [ Command(1, 0),       Command(1, 0),       Command(1, 0), ...
           Command(pi/2, pi/2), Command(pi/2, pi/2), ...
           Command(1, 0),       Command(1, 0),       Command(1, 0)];

% Motion uncertanties
a = [0.0001; 0.0001; 0.01; 0.0001; 0.0001; 0.0001];
  
% List of Measurements
all_zt = [ Measurement(2.276, 5.249, 2), Measurement(4.321, 5.834, 3), ...
           Measurement(3.418, 5.869, 3), Measurement(3.774, 5.911, 4), ...
           Measurement(2.631, 5.140, 5), Measurement(4.770, 5.791, 6), ...
           Measurement(3.828, 5.742, 6), Measurement(3.153, 5.739, 6)];

% Measurement Uncertanties
sig_r = 0.1;
sig_phi = 0.09;


%% Run EKF over all timesteps
% Init storage variables
beliefs(1:9) = bel_0;
truths(9) = x0;
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
    % Run motion model w/ no noise to get truth
    xt = sample_motion_model_velocity(ut, xt_1, zeros(1,6));
    xt_1 = xt;
    truths(t+1) = xt; 
end

%% Collect state means and covariances
covs = zeros(3, 3, n+1);
locs = zeros(n+1, 3);
t_vecs = zeros(n+1, 3);
for i = 1:n+1
    bel = beliefs(i);
    loc = get_loc(bel);
    locs(i,:) = loc';
    cov = bel.Cov;
    covs(:,:,i) = cov;
    t_vecs(i,:) = get_vec(truths(i));
end

%% Plot EKF run
close all;
locxs = locs(:,1);
locys = locs(:,2);

truexs = t_vecs(:,1);
trueys = t_vecs(:,2);

plot(locxs, locys);
hold on;
plot(truexs, trueys);
for i=1:n+1
    
end

%% Particle Filter Initialization
M = 1000;
% Generate set of random particles
X0(1:M, 1) = Particle(State(0,0,0), 0);
xs = rand_range(0, 8, M);
ys = rand_range(0, 8, M);
thetas = rand_range(0, 2*pi, M);
for i = 1:M
    rand_state = State(xs(i), ys(i), thetas(i));
    X0(i) = Particle(rand_state, 1/M);
end

%% Run particle filter 
% Initial particle set/true pose
Xt = X0;
xt_1 = x0;
% Array to store true poses
truths(1:9) = x0;
% Array to store intermediate particle sets
particle_sets = [Xt'];
% Loop over all timesteps
for t = 1:n
    % Get command and measurement
    ut = all_ut(t);
    zt = all_zt(t);
    % Run EKf
    Xt = particle_filter(Xt, M, ut, a, zt, sig_r, sig_phi, m);
    particle_sets = [particle_sets; Xt'];
    % Run motion model w/ no noise to get truth
    xt = sample_motion_model_velocity(ut, xt_1, zeros(1,6));
    xt_1 = xt;
    truths(t+1) = xt; 
end

%% Plot particles for each timestep
pxs = zeros(1,M);
pys = zeros(1,M);
plot(truexs, trueys);
hold on;
for j=1:9
    particles = particle_sets(j,:);
    for i=1:M
        p = particles(i);
        pxs(i) = p.x.x;
        pys(i) = p.x.y;
    end
    scatter(pxs, pys);
end

























