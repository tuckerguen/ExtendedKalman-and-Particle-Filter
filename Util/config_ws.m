% Clear workspace
clear all;
% Include files from subdirectories
p = genpath('./');
addpath(p);

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
mu_0 = State(2, 2, 0);
Cov_0 = eye(3,3) * 0.5;
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

% Collect true robot motion for all time steps
true_xs = ones(1,n+1) * x0.x;
true_ys = ones(1,n+1) * x0.y;
xt_1 = x0;

for t = 1:n
    ut = all_ut(t);
    zt = all_zt(t);
    % Run motion model w/ no noise to get truth
    xt = sample_motion_model_velocity(ut, xt_1, zeros(1,6));
    xt_1 = xt;
    true_xs(t+1) = xt.x;    
    true_ys(t+1) = xt.y;
end






















