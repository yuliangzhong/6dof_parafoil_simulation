% solving guidance by casadi + ipopt

function [flag, guidance] = GuidanceSolve(N, sys_param, init_cond, psi_d, psi_dot0, heights, wind_profile_hat)
%% Parameters Reading
Vh0 = sys_param(1);
Vz0 = sys_param(2);
psi_dot_m = sys_param(3);
x0 = init_cond(1);
y0 = init_cond(2);
h0 = init_cond(3);
psi_0 = init_cond(4);

%% Altitude Evolution
ch = 1.225;
cz = 2.256e-5;
ce = 4.2559;
cf = ce/2+1;

% functions
rho = @(h) ch*(1-h*cz).^ce;
h = @(t) 1/cz*(ones(1,size(t,2)) - (cz*cf*Vz0*sqrt(rho(h0))/sqrt(ch)*t + (1-cz*h0)^cf*ones(1,size(t,2))).^(1/cf));
Vh = @(h) Vh0*sqrt(rho(h0)./rho(h));

% time gap
tf = sqrt(ch)/(cz*cf*Vz0*sqrt(rho(h0)))*(1-(1-cz*h0)^cf);
dt = tf/(N-1);
% time mesh
times = linspace(0, tf, N);
hs = h(times);
Vhs = Vh(hs);
Ws = [interp1(heights, wind_profile_hat(1,:), hs, 'spline','extrap');
      interp1(heights, wind_profile_hat(2,:), hs, 'spline','extrap')];

%% Optimization
lambda1 = 100;
lambda2 = 100;
lambda3 = 0.1;

Au = [1; -1];
bu = [psi_dot_m; psi_dot_m];

Prob = casadi.Opti();
x = Prob.variable(3, N);
u = Prob.variable(1, N);

cost = lambda1*(x(1,end)^2 + x(2,end)^2) + lambda2*(1-cos(x(3,end)-psi_d));

Prob.subject_to(x(:,1) == [x0; y0; psi_0]);
Prob.subject_to(u(1) == psi_dot0);

for i = 1:N-1
    Prob.subject_to(x(:,i+1) == x(:,i) + dt*[(Vhs(i)*cos(x(3,i))+Vhs(i+1)*cos(x(3,i+1)))/2 + (Ws(1,i)+Ws(1,i+1))/2;
                                             (Vhs(i)*sin(x(3,i))+Vhs(i+1)*sin(x(3,i+1)))/2 + (Ws(2,i)+Ws(2,i+1))/2;
                                             u(i)]);
    cost = cost + lambda3*(i/N)^2*u(i)^2*dt + (u(i+1) - u(i))^2/dt;
    Prob.subject_to(Au*u(i) <= bu);
end
Prob.subject_to(Au*u(N) <= bu);

Prob.minimize(cost)
Prob.solver('ipopt', struct('print_time', 0), struct('print_level', 0));
sol = Prob.solve();

% --- output ---
flag = sol.stats.success;
guidance = zeros(5, N);
if flag
    xs = sol.value(x);
    us = sol.value(u);
    guidance(1:2,1:N) = xs(1:2,1:N);
    guidance(3,1:N) = hs(1:N);
    guidance(4,1:N) = xs(3,1:N);
    guidance(5,1:N) = us(1:N);
else
    disp("ERROR! Guidance solver failed!!")
end

end


