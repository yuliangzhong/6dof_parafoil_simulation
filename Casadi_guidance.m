% solving guidance by casadi-ipopt
%% Wind Profile, Pos Compensation and Wind Gust Dynamics
vel_at6m = 1.5; % wind velocity at 6m, absolute value
theta = 120; % [deg] constant

wind_h = @(h) (h>0.04572)*vel_at6m.*log(h/0.04572)/log(6.096/0.04572); % wind shear model
theta_h = @(h) theta/180*pi + pi; % forcasted wind field
GetWindProfile = @(h) [wind_h(h).*cos(theta_h(h));
                       wind_h(h).*sin(theta_h(h));
                       zeros(1,size(h,2))]; % [wx; wy; 0];
wind_pf_size = 7000;
heights = linspace(1e-6, 350, wind_pf_size); % start from 0+ avoiding NaN
wind_profile_hat = GetWindProfile(heights);

%% Parameters
N = 200;
psi_dot_m = 0.2187;
h0 = 262;
Vh0 = 4.59;
Vz0 = 1.39;
x0 = 200;
y0 = -200;
psi_0 = pi*2/4;

sys_param0 = [Vh0, Vz0, psi_dot_m];
init_cond = [x0, y0, h0, psi_0];
psi_d = pi;

%% Parameters Reading
Vh0 = sys_param0(1);
Vz0 = sys_param0(2);
psi_dot_m = sys_param0(3);
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

tf = sqrt(ch)/(cz*cf*Vz0*sqrt(rho(h0)))*(1-(1-cz*h0)^cf);
dt = tf/N;
% time mesh
times = linspace(0, tf, N+1);
hs = h(times);
Vhs = Vh(hs);
Ws = [interp1(heights, wind_profile_hat(1,:), hs, 'linear','extrap');
      interp1(heights, wind_profile_hat(2,:), hs, 'linear','extrap')];

%% Optimization Setup

u0 = [Vh0*cos(psi_0);
      Vh0*sin(psi_0)];

lambda1 = 100;
lambda2 = 10;
eps = 0.0001;

%% Optimization
Prob = casadi.Opti();
x = Prob.variable(2, N+1);
u = Prob.variable(2, N+1);

cost = lambda1*(x(1,end)^2 + x(2,end)^2) + lambda2 * (1-cos(psi_d)*u(1,end)/Vhs(end) - sin(psi_d)*u(2,end)/Vhs(end));

Prob.subject_to(x(:,1) == [x0;y0]);
Prob.subject_to(u(:,1) == u0);
Prob.subject_to(u(1,N+1)^2 + u(2,N+1)^2 <= Vhs(N+1)^2 + eps);
Prob.subject_to(u(1,N+1)^2 + u(2,N+1)^2 >= Vhs(N+1)^2 - eps);

for i = 1:N
    Prob.subject_to(x(:,i+1) == x(:,i) + dt*[(u(1,i)+u(1,i+1))/2 + (Ws(1,i)+Ws(1,i+1))/2;
                                             (u(2,i)+u(2,i+1))/2 + (Ws(2,i)+Ws(2,i+1))/2]);
    Prob.subject_to(u(1,i+1)^2 + u(2,i+1)^2 <= Vhs(i+1)^2 + eps);
    Prob.subject_to(u(1,i+1)^2 + u(2,i+1)^2 >= Vhs(i+1)^2 - eps);
    du = u(:,i+1) - u(:,i);
    Prob.subject_to((du(1)^2 + du(2)^2)/Vhs(i)^2/dt^2 <= psi_dot_m^2)
    cost = cost + (du(1)^2 + du(2)^2)/Vhs(i)^2/dt;
end
Prob.minimize(cost)

Prob.solver('ipopt', struct('print_time', 0), struct('print_level', 0));
sol = Prob.solve();

% --- output ---
flag = sol.stats.success;
if flag
    xs = sol.value(x);
    us = sol.value(u);
    plot(xs(2,:),xs(1,:))
else
    disp("MPC Solution not found")
end

