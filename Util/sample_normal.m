function dist = sample_normal(u, v, n)
    dist = normrnd(u,sqrt(v),n,1);
   
    