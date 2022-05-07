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

%% mission independent params
ch = 1.225;
cz = 2.256e-5;
ce = 4.2559;
cf = ce/2+1;
rho = @(h) ch*(1-h*cz).^ce;
N = 30;
psi_dot_m = 0.2187;

%% from sensor
H = 1200;
x0 = 400;
y0 = 400;
psi_0 = 0; % init_cond
psi_d = pi; % desired
Vh0 = 18.5; 
Vz0 = 7.9;

tf = sqrt(ch)/(cz*cf*Vz0*sqrt(rho(H)))*(1-(1-cz*H)^cf);
h = @(t) 1/cz*(ones(1,size(t,2)) - (cz*cf*Vz0*sqrt(rho(H))/sqrt(ch)*t + (1-cz*H)^cf*ones(1,size(t,2))).^(1/cf));
Vh = @(h) Vh0*sqrt(rho(H)./rho(h));

dt = tf/N;
Time = linspace(0, tf, N+1);
hs = h(Time);
Vhs = Vh(hs);
u0 = [Vh0*cos(psi_0-psi_d);
      Vh0*sin(psi_0-psi_d)];
u_bar0 = u0*ones(1,N+1);
W = [interp1(heights, wind_profile_hat(1,:), hs, 'linear','extrap');
     interp1(heights, wind_profile_hat(2,:), hs, 'linear','extrap')];

%% Dyn
A = eye(2);
B_m = dt/2*[cos(psi_d), -sin(psi_d);
            sin(psi_d), cos(psi_d)];
B_p = B_m;
eps_h_val = 0.1;
eps_convergence = 0.01;
lambda1 = 100;
lambda2 = 10;
lambda3 = 1;

%% Optimization
x = sdpvar(2,N+1);
u = sdpvar(2,N+1);
u_bar = sdpvar(2,N+1);
eps_h = sdpvar(1);

cost = 0;
constraints = [eps_h >= 0, ...
               x(:,1) == [x0;y0], ...
               u(:,1) == u0, ...
               norm(u(:,N+1)) - Vhs(N+1) <= eps_h ...
               u_bar(:,N+1)'/norm(u_bar(:,N+1))*u(:,N+1) - Vhs(N+1) >= -eps_h];
for i = 1 : N
    constraints = [constraints, x(:,i+1) == A*x(:,i) + B_m*u(:,i) + B_p*u(:,i+1) + dt*W(:,i) ...
                              , norm(u(:,i+1) - u(:,i))/Vhs(i)/dt <= psi_dot_m ...
                              , norm(u(:,i)) - Vhs(i) <= eps_h ...
                              , u_bar(:,i)'/norm(u_bar(:,i))*u(:,i) - Vhs(i) >= -eps_h];
    cost = cost + (norm(u(:,i+1) - u(:,i))/Vhs(i))^2/dt;
end

objective = lambda1*norm(x(:,end)) + lambda2*(1-u(1,end)/Vhs(end)) + lambda3*cost;

MAX_ITER = 50;

%% Verification & Visualization

it_final_position = zeros(1,MAX_ITER);
it_final_angle = zeros(1,MAX_ITER);
it_control_cost = zeros(1,MAX_ITER);
it_cost = zeros(1,MAX_ITER);
X = zeros(2*MAX_ITER,N);
U = zeros(2*MAX_ITER,N);
D = zeros(2*MAX_ITER,N-1);
% 
% problem = cvx.Problem(cvx.Minimize(cost), const + [eps_h ==0.1])
% first_stage_converged = False
% sdpsettings('solver', 'ecos');

options = sdpsettings('verbose',0,'debug',1,'solver', 'ecos'); % It's using FMINCON-STANDARD. should be 'ecos'
stage1 = optimizer([constraints, eps_h==eps_h_val],objective,options,u_bar,{x,u,eps_h});

% print('Iteration number\t Final position\t Final angle\t Control cost\t Total cost') 
% for i in range(MAX_ITER):
% 
%     s = problem.solve(solver=cvx.ECOS, verbose=True, warm_start=True)
%     u_bar.value = np.divide(u.value,np.linalg.norm(u.value, axis=0))
% 
%     x_star = x.value
%     u_star = u.value
%     
%     X[:,:,i] = x_star
%     U[:,:,i] = u_star
%     
%     it_final_position[i] = final_position.value
%     it_final_angle[i] = final_angle.value
%     it_control_cost[i] = control_cost.value
%     it_cost[i] = cost.value
% 
%     print(str(i)+'\t'+'\t'+'\t'+"%10.3E" %it_final_position[i]+'\t'+"%10.3E" %it_final_angle[i]+'\t'+"%10.3E" %it_control_cost[i]+'\t'+"%10.3E" %it_cost[i])
%     if (np.abs(it_cost[i]-it_cost[i-1]) < eps_convergence) and first_stage_converged:
%         print("STAGE 2 CONVERGED AFTER " +str(i)+" ITERATIONS")
%         n_iter = i
%         break
%     if (i>1) and (np.abs(it_cost[i]-it_cost[i-1]) < eps_convergence) and not first_stage_converged:
%         print("STAGE 1 CONVERGED AFTER " +str(i)+" ITERATIONS")
%         cost = cost+ alpha_3*eps_h
%         problem = cvx.Problem(cvx.Minimize(cost), const)
%         first_stage_converged = True
%         n_iter_first = i