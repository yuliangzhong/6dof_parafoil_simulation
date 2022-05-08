% SCP Guidance
clear all;
clc;

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
N = 100;
psi_dot_m = 0.2187;
h0 = 262;
Vh0 = 4.59;
Vz0 = 1.39;
x0 = 200;
y0 = -200;
psi_0 = pi*2/4;
psi_d = pi;
% wind_profile_hat
% heights

%% Altitude Evolution

ch = 1.225;
cz = 2.256e-5;
ce = 4.2559;
cf = ce/2+1;

rho = @(h) ch*(1-h*cz).^ce;
h = @(t) 1/cz*(ones(1,size(t,2)) - (cz*cf*Vz0*sqrt(rho(h0))/sqrt(ch)*t + (1-cz*h0)^cf*ones(1,size(t,2))).^(1/cf));
Vh = @(h) Vh0*sqrt(rho(h0)./rho(h));

tf = sqrt(ch)/(cz*cf*Vz0*sqrt(rho(h0)))*(1-(1-cz*h0)^cf);
dt = tf/N;

times = linspace(0, tf, N+1);
hs = h(times);
Vhs = Vh(hs);
Ws = [interp1(heights, wind_profile_hat(1,:), hs, 'linear','extrap');
      interp1(heights, wind_profile_hat(2,:), hs, 'linear','extrap')];

%% Optimization Setup

u0 = [Vh0*cos(psi_0);
      Vh0*sin(psi_0)];
u_bar_normalized = u0/norm(u0)*ones(1,N+1);

eps_h = 0.1;
MAX_ITER = 50;
current_cost = inf;
eps_convergence = 0.1;
lambda1 = 100;
lambda2 = 10;
lambda3 = 1;

x_star = zeros(2,N+1);
u_star = zeros(2,N+1);

%% Optimization Stage One

yalmip('clear')
x = sdpvar(2,N+1); % [x; y]
u = sdpvar(2,N+1); % [v1; v2]
cost = lambda1*(x(1,end)^2 + x(2,end)^2) + lambda2 * (1-cos(psi_d)*u(1,end)/Vhs(end) - sin(psi_d)*u(2,end)/Vhs(end));
constraints = [x(:,1) == [x0;y0], ...
               u(:,1) == u0, ...
               norm(u(:,N+1)) <= Vhs(N+1) + eps_h];
for i = 1:N
    constraints = [constraints, x(:,i+1) == x(:,i) + dt*[(u(1,i)+u(1,i+1))/2 + (Ws(1,i)+Ws(1,i+1))/2;
                                                         (u(2,i)+u(2,i+1))/2 + (Ws(2,i)+Ws(2,i+1))/2] ...
                              , norm(u(:,i)) <= Vhs(i) + eps_h ...
                              , norm(u(:,i+1) - u(:,i))/Vhs(i)/dt <= psi_dot_m];
    cost = cost + lambda3*norm(u(:,i+1) - u(:,i))^2/Vhs(i)^2/dt;
end
options = sdpsettings('verbose',0,'solver','ecos');

for n = 1:MAX_ITER
    disp("At iteration "+num2str(n)+", current cost: "+num2str(current_cost))

    Cons = constraints;
    for i = 1:N+1
        Cons = [Cons, u_bar_normalized(:,i)'*u(:,i) >= Vhs(i) - eps_h];
    end
    sol = optimize(Cons, cost, options);
    if sol.problem ~= 1
        if abs(value(cost) - current_cost) < eps_convergence
            x_star = value(x);
            u_star = value(u);
            disp("At iteration "+num2str(n)+", final cost: "+num2str(value(cost)))
            break
        else
            current_cost = value(cost);
            for j = 1:N+1
                u_sol = value(u);
                u_bar_normalized(:,j) = u_sol(:,j)/norm(u_sol(:,j));
            end
        end
    else
        disp("Something went wrong at iteration "+num2str(n));
        disp(sol.info)
    end
end 