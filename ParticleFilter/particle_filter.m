function Xt = particle_filter(Xt_1, num_particles, ut, a, zt, sig_r, sig_phi, map)
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