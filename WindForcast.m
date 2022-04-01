function wind_bar_hat = WindForcast(h)
    % h: height, [m]
    
    % Wind Profile forcast
    vel_at6m = 2; % wind velocity at 6m, absolute value
    theta_h = - 5/6*pi + 1/6*pi * h/300; % forcasted wind field

    W_h = vel_at6m*log(h/0.04572)/log(6.096/0.04572);
    if W_h < 0
        W_h = 0;
    end
    
    wind_bar_hat = W_h*[cos(theta_h);
                  sin(theta_h);
                  0];

end