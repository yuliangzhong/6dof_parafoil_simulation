% Curve fitting for MPCC
% test height from h0 --> 0

% guidance = out.guidance.signals.values(:,:,end);
figure(1)
hold on
grid on
scatter(guidance(2,:), guidance(1,:))
figure(2)
hold on
grid on
scatter(heights, wind_profile_hat(1,:),'g')
scatter(heights, wind_profile_hat(2,:),'g')

% h0 = 2;
% interpolate guidance & wind profile
h0s = linspace(10, 0, 10);
% h0s = linspace(100, 10, 10);

dh = mpcc_Ts*(vel_info(3)+0);

for i = 1:10
    % set break point here, check fitting result
    h0 = h0s(i);

    h_range = 1.1*time_horizon_N*dh;
    h_max = h0 + h_range; 
    h_min = max(0 - h_range, h0 - 2*h_range);
    extrap_heights_num = (h_max - h_min)/dh+1;
    interp_heights = linspace(h_max, h_min, extrap_heights_num);
    interp_method = 'spline';
    if h0 < 10
    interp_method = 'linear'; % to avoid strange fitting near the ground
    end
    interp_guidance = [interp1(guidance(3,:), guidance(1,:), interp_heights, interp_method, 'extrap');
                       interp1(guidance(3,:), guidance(2,:), interp_heights, interp_method, 'extrap');
                       interp_heights;
                       interp1(guidance(3,:), guidance(4,:), interp_heights, interp_method, 'extrap');
                       interp1(guidance(3,:), guidance(5,:), interp_heights, interp_method, 'extrap')];
    
    interp_wind_pf = [interp1(heights, wind_profile_hat(1,:), interp_heights, 'linear', 'extrap');
                      interp1(heights, wind_profile_hat(2,:), interp_heights, 'linear', 'extrap')];

    % fx
    px = polyfit(interp_guidance(3,:), interp_guidance(1,:), 10);
    fx = @(x) px*[x.^10; x.^9; x.^8; x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
    dfx = @(x) px*[10*x.^9; 9*x.^8; 8*x.^7; 7*x.^6; 6*x.^5; 5*x.^4; 4*x.^3; 3*x.^2; 2*x; ones(1,size(x,2)); zeros(1,size(x,2))];
    
    % fy
    py = polyfit(interp_guidance(3,:), interp_guidance(2,:), 10);
    fy = @(x) py*[x.^10; x.^9; x.^8; x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
    dfy = @(x) py*[10*x.^9; 9*x.^8; 8*x.^7; 7*x.^6; 6*x.^5; 5*x.^4; 4*x.^3; 3*x.^2; 2*x; ones(1,size(x,2)); zeros(1,size(x,2))];
    
    % wx
    pwx = polyfit(interp_heights, interp_wind_pf(1,:), 10);
    wx = @(x) pwx*[x.^10; x.^9; x.^8; x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
    % wy
    pwy = polyfit(interp_heights, interp_wind_pf(2,:), 10);
    wy = @(x) pwy*[x.^10; x.^9; x.^8; x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
    
    figure(1)
    plot(fy(interp_heights), fx(interp_heights))

    figure(2)
    plot(interp_heights, wx(interp_heights),'r')
    plot(interp_heights, wy(interp_heights),'b')
end
disp('done')