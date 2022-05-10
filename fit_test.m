%% fitting --> fx, fy, wx, wy
guidance = out.guidance.signals.values(:,:,end);
hold on
grid on
plot(guidance(2,:), guidance(1,:))

h0 = 2;

% interpolate guidance & wind profile
h_max = h0+50; h_min = max(-1,h0-30);
extrap_heights_num = 1000;
interp_heights = linspace(h_max, h_min, extrap_heights_num);
interp_method = 'spline';
if h0 < 20
    interp_method = 'spline';
end
interp_guidance = [interp1(guidance(3,:), guidance(1,:), interp_heights, interp_method, 'extrap');
                   interp1(guidance(3,:), guidance(2,:), interp_heights, interp_method, 'extrap');
                   interp_heights];
interp_wind_pf = [interp1(heights, wind_profile_hat(1,:), interp_heights, 'linear', 'extrap');
                  interp1(heights, wind_profile_hat(2,:), interp_heights, 'linear', 'extrap')];
[~, id] = min(vecnorm(interp_guidance(1:2,:) - init_cond(1:2)*ones(1,size(interp_guidance,2))));
hr = interp_guidance(3,id);

%% fitting --> fx, fy, wx, wy

% fx
px = polyfit(interp_guidance(3,:), interp_guidance(1,:), 5);
fx = @(x) px*[x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
dfx = @(x) px*[5*x.^4; 4*x.^3; 3*x.^2; 2*x; ones(1,size(x,2)); zeros(1,size(x,2))];

% fy
py = polyfit(interp_guidance(3,:), interp_guidance(2,:), 5);
fy = @(x) py*[x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
dfy = @(x) py*[5*x.^4; 4*x.^3; 3*x.^2; 2*x; ones(1,size(x,2)); zeros(1,size(x,2))];

HH = linspace(h0, h0-13.9,50);
scatter(interp_guidance(2,:), interp_guidance(1,:),5)
plot(fy(HH), fx(HH),'m')
