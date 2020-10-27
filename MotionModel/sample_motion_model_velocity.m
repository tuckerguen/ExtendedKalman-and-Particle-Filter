% ut = [vt, wt], xt_1 = [x_t-1 y_t-1 w_t-1], a = [a1,...,a6]
function xti = sample_motion_model_velocity(ut, xt_1, a)
    % Collect useful values
    dt = 1;
    vt = ut.v;
    wt = ut.w;
    theta = xt_1.theta;
    
    % Adjust vt and wt for noise given alpha
    vh = vt + sample_normal(0, a(1)*abs(vt)+a(2)*abs(wt), 1);
    wh = wt + sample_normal(0, a(3)*abs(vt)+a(4)*abs(wt), 1);
    yh = sample_normal(0, a(5)*abs(vt)+a(6)*abs(wt), 1);
    
    % Account for vh/wh undefined
    if wh == 0
        wh = 0.0000001;
    end
    
    adjustment = State(...
        -(vh/wh)*sin(theta)+(vh/wh)*sin(theta + (wh * dt)),...
         (vh/wh)*cos(theta)-(vh/wh)*cos(theta + (wh * dt)),...
         (wh * dt) + (yh * dt)...
    );

    xti = add(xt_1, adjustment);
