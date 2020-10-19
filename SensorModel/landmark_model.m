function prob = landmark_model(fti, idx_in_m, xt, m, sig_r, sig_phi)
    prob = 1;
    for i=1:size(fti,1)
        j = idx_in_m(i);
        mjx = m(j,1);
        mjy = m(j,2);
        rh = sqrt((mjx-xt(1))^2+(mjy-xt(2))^2);
        phih = atan2(mjy-xt(2), mjx-xt(1));
        p1 = normpdf(fti(i,1)-rh,   0, sig_r);
        p2 = normpdf(fti(i,2)-phih, 0, sig_phi);   
        p_fti = p1*p2;
        prob = prob * p_fti;
    end