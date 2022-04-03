function [wind_bar_hat, wind_pos_drift] = WindForcast(h)    
    vel_at6m = 2; % wind velocity at 6m, absolute value
    theta_h = - 5/6*pi + 1/6*pi * h/300; % forcasted wind field
    
    % wind shear model
    W_h = vel_at6m*log(h/0.04572)/log(6.096/0.04572);

    % position drift caused by wind
    % let this shit integrator work in simulink!
    Vz = 1.40; % Check in simulator: ground speed (3)
    N = 1000000;
    z = linspace(0.04572, h, N);
    dz = z(2) - z(1);
    fx = @(z) vel_at6m*log(z/0.04572)/log(6.096/0.04572).*cos(- 5/6*pi + 1/6*pi * z/300);
    fy = @(z) vel_at6m*log(z/0.04572)/log(6.096/0.04572).*sin(- 5/6*pi + 1/6*pi * z/300);
    xw = fx(z)*ones(N,1)*dz/Vz;
    yw = fy(z)*ones(N,1)*dz/Vz;
    wind_pos_drift = [xw; yw; 0];

    % for code robustness
    if h < 0.04572
        wind_bar_hat = zeros(3,1);
        return
    end
    
    % Wind profile forcast at height h[m]
    wind_bar_hat = W_h*[cos(theta_h);
                  sin(theta_h);
                  0];
end