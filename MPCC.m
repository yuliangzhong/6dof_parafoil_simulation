function [flag, xs, us, e_h] = MPCC(N, Ts, sys_param, init_cond, wind_err, guidance, heights, wind_profile_hat)

%% Params
Vh = sys_param(1);
Vz = sys_param(2);
psi_dot_m = sys_param(3);
um = psi_dot_m;
Au = [1, 0; -1, 0; 0, 1; 0, -1];
bu = [um; um; 2*Vz; 0];

[~, id] = min(vecnorm(guidance(1:2,:) - init_cond(1:2)*ones(1,2000)));
hr = guidance(3,id);
e_h = init_cond(3) - hr;

%% fitting --> fx, fy, wx, wy
ids = max(id - 50, 1);
idn = min(id+2*N, 2000);

% fx
px = polyfit(guidance(3,ids:idn), guidance(1,ids:idn), 7);
fx = @(x) px*[x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
dfx = @(x) px*[7*x.^6; 6*x.^5; 5*x.^4; 4*x.^3; 3*x.^2; 2*x; ones(1,size(x,2)); zeros(1,size(x,2))];

% fy
py = polyfit(guidance(3,ids:idn), guidance(2,ids:idn), 7);
fy = @(x) py*[x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
dfy = @(x) py*[7*x.^6; 6*x.^5; 5*x.^4; 4*x.^3; 3*x.^2; 2*x; ones(1,size(x,2)); zeros(1,size(x,2))];

% 
ppsi = polyfit(guidance(3,ids:idn), guidance(4,ids:idn), 7);
fpsi = @(x) ppsi*[x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];

% wx
pwx = polyfit(heights, wind_profile_hat(1,:), 7);
wx = @(x) pwx*[x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
% wy
pwy = polyfit(heights, wind_profile_hat(2,:), 7);
wy = @(x) pwy*[x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];

%% MPC formulation

% P = diag([100,100,10000]);
Q = diag([1000, 1000]);
R = diag([1000, 1000]);
q_eta = 100;
q_u = 1000;

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

objective = objective + [es_c, es_l]* Q * [es_c, es_l]';
Prob.minimize(objective)

% --- define constraints ---
Prob.subject_to(X(:,1)==[init_cond; hr]);
for i = 1:N
    Prob.subject_to(Au*U(:,i)<=bu);
    Prob.subject_to(X(:,i+1) == X(:,i) + Ts* [Vh * cos(X(4,i)) + wx(X(3,i)) + wind_err(1);
                                              Vh * sin(X(4,i)) + wy(X(3,i)) + wind_err(2);
                                              - (Vz+wind_err(3));
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
else
    disp("MPC Solution not found")
    xs = zeros(5,N+1);
    us = zeros(2,N);
end

%% plot
% scatter(fy(xs(5,:)), fx(xs(5,:)) ,'g')
plot(xs(2,:),xs(1,:), 'b')

end
