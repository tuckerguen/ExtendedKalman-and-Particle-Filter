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
%   zt      = features, list of measurements of landmarks at time t
%   m       = Map robot is operating in

function belief = EKF(bel_t1, ut, a, zt_lis, sig_r, sig_phi, m)
    % Standardized delta t = 1s
    dt = 1;
    % Mean of belief of theta
    th = bel_t1.mu.t;
    % theta' = thetat + w * dt
    thp = th + ut.w * dt;
    % Account for vh/wh undefined
    if ut.w == 0
        ut.w = sqrt(realmin);
    end
    % v / w
    vdw = ut.v/ut.w;
    
    % Prediction Step
    % ----------------------------------------------
    % Compute G, approximation of true robot motion
    G = [
        1 0 (-(vdw) * cos(th)) + (vdw * cos(thp));
        1 0 (-(vdw) * sin(th)) + (vdw * sin(thp));
        0 0                     1                ;
    ];

    % Transformation V, mapping motion noise to state space 
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

    % Covariance of noise in control space
    M = [
        (a(1) * ut.v^2 + a(2) * ut.w^2)                0;
                       0                (a(3) * ut.v^2 + a(3) * ut.w^2);
    ];

    % Term to update mu with
    mu_adj = [
        -vdw * sin(th) + vdw * sin(thp);
         vdw * cos(th) - vdw * cos(thp);
                  ut.w * dt
    ];

    % mu-bar and covariance-bar, intermediate mu and cov values after
    % prediciton step
    mu_b = add_vector(bel_t1.mu, mu_adj);
    Cov_b = G * bel_t1.Cov * G' + V * M * V';    
    
    % Correction Step
    % ----------------------------------------------
    % Loop over all observed landmarks (For this assignment is only one)
    
    % Covariance of measurement noise
    Q = [
        sig_r^2    0      0;
           0    sig_phi^2 0;
           0       0      0; 
    ];
    
    % Loop over all observed landmarks
    for i = 1:length(zt_lis)
        zt = zt_lis(i);
        
        j = zt.c;
        mjx = m.landmarks(j).mx;
        mjy = m.landmarks(j).my;
        
        q = (mjx - mu_b.x)^2 + (mjy - mu_b.y)^2;
        
        z_h = [ 
                            sqrt(q); 
            atan2(mjy - mu_b.y, mjx - mu_b.x) - mu_b.t;
                               0;
        ];
    
        H = [
            -(mjx - mu_b.x) / sqrt(q), -(mjy - mu_b.y) / sqrt(q),  0;
             (mjy - mu_b.y) / q      , -(mjx - mu_b.x) / q      , -1;
                            0                           0          0;
        ];
        
        S = H * Cov_b * H' + Q;
        % Kalman Gain
        K = Cov_b * H' / S;
        
        zt_vec = get_measurement_vec(zt);
        
        
        v =  K * (zt_vec - z_h);
        
        mu_b = add_vector(mu_b, v);
        Cov_b = (eye(3) - K * H) * Cov_b;
    end
    
    belief = StateBelief(mu_b, Cov_b);




























    

