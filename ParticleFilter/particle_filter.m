% The Particle Filter Localization Algorithm
%   for landmark model with known correspondences.
%   Returns a set of particles modeling the posterior distribution
%   of the robot state given a set of motion commands and measurements
%   Given: A sensor measurement realtive to a landmark, 
%   the motion command given before that measurement,
%   and the set of particles generated during the previous
%   timestep.
% Parameters:
%   Xt_1    = particle set at time t-1
%   ut      = Command at time t
%   a       = motion uncertainty parameters 
%   zt      = features, list of measurements of landmarks at time t
%   sig_r   = stddev of distance to landmark measurements
%   sig_phi = stddev of heading to landmark measurements
%   m       = Map robot is operating in

function [Xt_b, Xt] = particle_filter(Xt_1, ut, a, zt, sig_r, sig_phi, map)
    num_particles = length(Xt_1);
    Xt_b(1:num_particles, 1) = Particle(State(0,0,0), 0);
    sum_of_w = 0;
    
    for i=1:num_particles
        pt_1 = Xt_1(i);
        % Sample new state using velocity model
        xt = sample_motion_model_velocity(ut, pt_1.x, a);
        % Compute weight using landmark model
        w = landmark_model(zt, xt, sig_r, sig_phi, map);
        % Add weight to sum of all weights
        sum_of_w = sum_of_w + w;
        % Add sampled particle to list for time t
        Xt_b(i) = Particle(xt, w);
    end
    % Normalize
    for i = 1:num_particles
        Xt_b(i).w = Xt_b(i).w / sum_of_w;
    end
    
    Xt(1:num_particles, 1) = Particle(State(0,0,0), 0);
    r = rand_range(0, 1/num_particles, 1);
    c = Xt_b(1).w;
    i = 1;
    % Resample
    for m = 1:num_particles
        U = r + (m - 1) * (1/num_particles);
        while U > c
            i = i + 1;
            c = c + Xt_b(i).w;
        end
        Xt(m) = Xt_b(i);
    end