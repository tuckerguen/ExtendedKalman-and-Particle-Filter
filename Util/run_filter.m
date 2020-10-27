function beliefs, truths = run_filter(filter_func, bel_0, x0, a, ...
                                        all_ut, all_zt, m, sig_phi, sig_r)

beliefs = [bel_0];
truths = [x0];
xt_1 = x0;

bel = bel_0;
for t = 1:n
    ut = all_ut(t);
    zt = all_zt(t);
    bel = filter_func(bel, ut, a, zt, sig_r, sig_phi, m);
    beliefs = [beliefs; bel];
    xt = sample_motion_model_velocity(ut, xt_1, zeros(1,6));
    xt_1 = xt;
    truths = [truths; xt]; 
end