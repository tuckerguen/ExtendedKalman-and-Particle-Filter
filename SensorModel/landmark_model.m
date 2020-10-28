function prob = landmark_model(zt, xt, sig_r, sig_phi, map)
    prob = 1;
    % Get location of landmark
    j = zt.c;
    landmark = map.landmarks(j);
    mjx = landmark.mx;
    mjy = landmark.my;
    % Distance
    rh = sqrt((mjx-xt.x)^2+(mjy-xt.y)^2);
    % Direction (relative to heading)
    phih = 2*pi + atan2(mjy-xt.y, mjx-xt.x) - xt.theta;
    % Compute probability
    p1 = normpdf(zt.r - rh,   0, sig_r);
    p2 = normpdf(zt.phi - phih, 0, sig_phi);   
    p_fti = p1 * p2;
    prob = prob * p_fti;
