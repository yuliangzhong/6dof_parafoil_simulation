function [flag, control] = MPCC(N, Ts, vel_info, psi_dot_m, init_cond, guidance, heights, wind_profile_hat, wind_dis)

tic

%% Parameters Reading
ch = 1.225;
cz = 2.256e-5;
ce = 4.2559;

% functions
h0 = init_cond(3);
rho = @(h) ch*(1-h*cz).^ce; % air density model
Vh = sqrt(rho(vel_info(1))*vel_info(2)^2/rho(h0)); % xy-vel no wind at current height
Vz = sqrt(rho(vel_info(1))*vel_info(3)^2/rho(h0)); % z-vel no wind at current height
um = psi_dot_m;
Au = [1, 0; -1, 0; 0, 1; 0, -1];
bu = [um; um; 2*Vz; -0.0*Vz];

% interpolate guidance & wind profile
h_max = h0+50; h_min = max(-1 ,h0-30);
extrap_heights_num = 1000;
interp_heights = linspace(h_max, h_min, extrap_heights_num);
interp_method = 'spline';
if h0 < 20
    interp_method = 'linear';
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

% wx
pwx = polyfit(interp_heights, interp_wind_pf(1,:), 5);
wx = @(x) pwx*[x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
% wy
pwy = polyfit(interp_heights, interp_wind_pf(2,:), 5);
wy = @(x) pwy*[x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];

%% MPC formulation

P = diag([10000, 10000]);
Q = diag([10000, 10000]);
R = diag([10000, 1000]);
q_eta = 500;
q_u = 10000;

Prob = casadi.Opti();
% --- define optimization variables ---
X = Prob.variable(5, N+1); % [x, y, h, psi, eta]
U = Prob.variable(2, N);   % [u, v]

% --- calculate objective --- 
objective = 0;

for i = 2:N
    gamma = atan2(-dfy(X(5,i)), -dfx(X(5,i))); % since eta is decreasing!
    es_l = -cos(gamma)*(X(1,i) - fx(X(5,i))) - sin(gamma)*(X(2,i) - fy(X(5,i)));
    es_c = sin(gamma)*(X(1,i) - fx(X(5,i))) - cos(gamma)*(X(2,i) - fy(X(5,i)));

    du = U(1,i) - U(1,i-1);
    dv = U(2,i) - U(2,i-1);
    objective = objective + [es_c, es_l]* Q * [es_c, es_l]' ...
                          + q_eta * X(5,i) ...
                          + [du, dv]* R *[du, dv]' ...
                          + q_u * U(1,i)^2;
end
gamma = atan2(-dfy(X(5,N+1)), -dfx(X(5,N+1))); % since eta is decreasing!
es_l = -cos(gamma)*(X(1,N+1) - fx(X(5,N+1))) - sin(gamma)*(X(2,N+1) - fy(X(5,N+1)));
es_c = sin(gamma)*(X(1,N+1) - fx(X(5,N+1))) - cos(gamma)*(X(2,N+1) - fy(X(5,N+1)));

objective = objective + [es_c, es_l]* P * [es_c, es_l]';
Prob.minimize(objective)

% --- define constraints ---
Prob.subject_to(X(:,1)==[init_cond; hr]);

for i = 1:N
    Prob.subject_to(Au*U(:,i)<=bu);
    Prob.subject_to(X(:,i+1) == X(:,i) + Ts* [Vh * cos(X(4,i)) + wx(X(3,i)) + wind_dis(1);
                                              Vh * sin(X(4,i)) + wy(X(3,i)) + wind_dis(2);
                                              - (Vz + wind_dis(3));
                                              U(1,i);
                                              -U(2,i)]);
end

% --- define solver ---
Prob.solver('ipopt', struct('print_time', 0), struct('print_level', 0));
sol = Prob.solve();

% --- output ---
flag = sol.stats.success;
if flag
    xs = sol.value(X);
    us = sol.value(U);
    control = [xs(3:4,1:N); % h, psi
               us(1,1:N)] ;  % psi_dot
else
    disp("MPC Solution not found")
    control = zeros(3,N);
end

%% plot
hold on
grid on
scatter(fy(xs(5,:)), fx(xs(5,:)) , 'g')
plot(xs(2,:),xs(1,:), 'b','LineWidth',1)
% scatter(xs(2,:),xs(1,:),'b')
% scatter(guidance(2,:), guidance(1,:), 'm')
% plot(interp_guidance(2,:), interp_guidance(1,:),'k')
% scatter(init_cond(2), init_cond(1),'b','filled')

toc
text(init_cond(2)+30, init_cond(1), num2str(toc))
end
