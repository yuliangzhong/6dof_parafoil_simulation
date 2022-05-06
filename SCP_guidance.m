% SCP Guidance
% clear all;
% clc;
% W = np.array([[17.96208853,17.56368787,20.61838065,25.18334004,22.52074534,26.77293274,25.57287022,20.51386757,18.42710807,21.36785401,18.12489511,14.94887125, 12.87491946, 13.09745288,  7.21907596, -1.459937 ,   7.740678 ,  10.37488769,  4.48765114  ,5.46872134, -1.68147061 , 4.22306641, 17.8954532  , 2.58745642, 1.11793932, -3.81641426, -3.75646  ,  -2.45822383 ,-0.22924514, -2.24006629],[-11.14862732 ,-13.8301893 , -17.73220257,  -8.10191481 , -2.33771849,  -0.71133681  , 7.3276312  ,  2.85030499 , -2.26370296 , -2.01329856,  -3.88281926 , -0.60767267,  -3.66318881 , -3.93941925,  -1.41578766,   8.08154894 , 10.92129449  , 6.36317964,   6.27415534,  -6.86086859,  -4.49156021  ,-6.23473274 , -7.2650244 ,   0.06751707  ,-7.68552073,  -4.35247244  , 0.49549029 , -4.15317153 , -0.96148253 , -2.29360969]])
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

% u_init = np.array([v*np.cos(psi_0),v*np.sin(psi_0)])
% u_bar.value = np.divide(u_init,np.linalg.norm(u_init, axis=0))

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
options = sdpsettings('verbose',1,'debug',1,'solver','quadprog','quadprog.maxiter',100);
P = optimizer(constraints,objective,options,u_bar,[x,u,eps_h]);

% 
% it_final_position = np.empty((MAX_ITER))
% it_final_angle=np.empty((MAX_ITER))
% it_control_cost =np.empty((MAX_ITER))
% it_cost =np.empty((MAX_ITER))
% X = np.empty((2,N,MAX_ITER))
% U = np.empty((2,N,MAX_ITER))
% D = np.empty((N-1,MAX_ITER))
% 
% problem = cvx.Problem(cvx.Minimize(cost), const + [eps_h ==0.1])
% first_stage_converged = False
% 
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