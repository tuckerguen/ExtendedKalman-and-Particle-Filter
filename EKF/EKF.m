% The Extended Kalman Filter Localization Algorithm
%   for landmark model with known correspondences.
%   Returns belief of the state, given by a mean and covariance
%   of a gaussian distribution
%   Given: A sensor measurement realtive to a landmark, 
%   the motion command given before that measurement,
%   mean and stddev of the estimated distribution of the belief
%   from the previous step
% Parameters:
%   bel_t1  = StateBelief at time t-1
%   ut      = Command at time t
%   a       = motion uncertainty parameters 
%   zt      = features, list of measurements of landmarks at time t
%   sig_r   = stddev of distance to landmark measurements
%   sig_phi = stddev of heading to landmark measurements
%   m       = Map robot is operating in

function [pre_bel, belief] = EKF(bel_t1, ut, a, zt_lis, sig_r, sig_phi, m)
    % Standardized delta t = 1s
    dt = 1;
    % Mean of belief of theta
    th = bel_t1.mu.t;
    % theta' = thetat + w * dt
    thp = th + ut.w * dt;
    
    % Prediction Step
    % ----------------------------------------------
    
    % Account for no angular velocity
    if ut.w ~= 0
        % v / w
        vdw = ut.v/ut.w;
        % Compute Jacobian G of motion model
        G = [
            1 0 (-(vdw) * cos(th)) + (vdw * cos(thp));
            1 0 (-(vdw) * sin(th)) + (vdw * sin(thp));
            0 0                     1                ;
        ];
    
        % Jacobian V, control space to state space approximation
        v11 = (-sin(th) + sin(thp)) / ut.w;
        v12 = ((ut.v * (sin(th) - sin(thp))) / (ut.w ^ 2)) + ...
          ((ut.v * cos(thp) * dt) / ut.w);
        v21 = (cos(th) - cos(thp)) / ut.w;
        v22 = (-(ut.v * (cos(th) - cos(thp))) / (ut.w ^ 2)) + ...
              ((ut.v * sin(thp) * dt) / ut.w);
        V = [
            v11 v12;
            v21 v22;
             0   dt;
        ];
    
        % Values to add to mu
        mu_adj = [
            -vdw * sin(th) + vdw * sin(thp);
             vdw * cos(th) - vdw * cos(thp);
                      ut.w * dt 
        ];
    else
        % Jacobian motion model w/ lim_{w->0}
        G = [
            1 0 0;
            1 0 0;
            0 0 1;
        ];
        % Jacobian w.r.t control
        V = [
            0 0;
            0 0;
            0 dt;
        ];
    
        mu_adj = [
            ut.v * cos(th)
            ut.w * sin(th)
                  0
        ];
    end

    % Covariance of noise in control space
    M = [
        (a(1) * ut.v^2 + a(2) * ut.w^2)                0;
                       0                (a(3) * ut.v^2 + a(3) * ut.w^2);
    ];

    % mu-bar and covariance-bar, intermediate mu and cov values after
    % prediciton step
    mu_b = add_vector(bel_t1.mu, mu_adj);
    Cov_b = G * bel_t1.Cov * G' + V * M * V';  
    pre_bel = StateBelief(mu_b, Cov_b);

    % Correction Step
    % ----------------------------------------------    
    % Covariance of measurement noise
    % Variance in signature reading is ideally 0 in this case
    % but to avoid having S be a non-invertible singular matrix,
    % we pick the value of sigma for signature error to be the smallest
    % floating pt value.
    % This produces numerically very close results to sigma = 0
    Q = [
        sig_r^2    0      0;
           0    sig_phi^2 0;
           0       0      realmin; 
    ];
    
    % Loop over all observed landmarks
    for i = 1:length(zt_lis)
        % Get measurement
        zt = zt_lis(i);
        % Get relevant values from landmark
        j = zt.c;
        mjx = m.landmarks(j).mx;
        mjy = m.landmarks(j).my;
        mjs = m.landmarks(j).ms;
        
        % rt
        q = (mjx - mu_b.x)^2 + (mjy - mu_b.y)^2;
        % Measurement given correspondence
        z_h = [ 
                            sqrt(q); 
            atan2(mjy - mu_b.y, mjx - mu_b.x) - mu_b.t;
                               mjs;
        ];
        % Jacobian of measurement w.r.t location
        H = [
            -(mjx - mu_b.x) / sqrt(q), -(mjy - mu_b.y) / sqrt(q) 0;
             (mjy - mu_b.y) / q      , -(mjx - mu_b.x) / q       -1;
                            0               0 0;
        ];
        
        S = H * Cov_b * H' + Q;
        % Kalman Gain
        K = Cov_b * (H' / S);
        % Get true measurement
        zt_vec = get_measurement_vec(zt);
        
        % Extend kalman gain for multiplication
%         K(3,3) = 0;
        z_err = (zt_vec - z_h);
        % Properly compute rotational error
        e1 = zt_vec(2) + 2*pi - z_h(2);
        e2 = zt_vec(2) - 2*pi - z_h(2);
        z_err(2) = min([z_err(2), e1, e2]);
        
        v =  K * z_err;
        
        % Adjust mu
        mu_b = add_vector(mu_b, v);
        Cov_b = (eye(3) - K * H) * Cov_b;
        % Account for small negatives as a result of realmin
    end
    
    belief = StateBelief(mu_b, Cov_b);




























    

