function prob = motion_model_velocity(xt, ut, xt_1)
    dt = 1;
    v = ut.v;
    w = ut.w;
    
    x = xt_1.x; y = xt_1.y; theta = xt_1.theta;
    xp = xt.x; yp = xt.y; thetap = xt.theta;
    
    u = 0.5 * ((x-xp)*cos(theta)+(y-yp)*sin(theta))/...
              ((y-yp)*cos(theta)-(x-xp)*sin(theta));
    
    xs = (x+xp)/2 + u*(y-yp);
    ys = (y+yp)/2 + u*(xp-x);
    rs = sqrt((x-xs)^2+(y-ys)^2);
    
    dtheta = atan2(yp-ys,xp-xs)-atan2(y-ys,x-xs);
        
    vh = dtheta/dt * rs;
    wh = dtheta/dt;
    yh =(thetap-theta)/dt - wh;

    v_err = v-vh;
    w_err = w-wh;
    
    v_stddev = sqrt(a(1)*abs(v)+a(2)*abs(w));
    w_stddev = sqrt(a(3)*abs(v)+a(4)*abs(w));
    y_stddev = sqrt(a(5)*abs(v)+a(6)*abs(w));
    
    prob = normpdf(v_err, 0, v_stddev)*...
           normpdf(w_err, 0, w_stddev)*...
           normpdf(yh,    0, y_stddev);
    
    