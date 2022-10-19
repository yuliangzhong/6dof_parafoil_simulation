function SCP_guidance(N, sys_param0, init_cond, psi_d, heights, wind_profile_hat)

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
u_bar_normalized = u0/norm(u0)*ones(1,N+1);

eps_h = 0.05;
MAX_ITER = 50;
current_cost = inf;
eps_convergence = 0.1;
lambda1 = 100;
lambda2 = 10;
lambda3 = 100;

% guidance = zeros(5,N);

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
    cost = cost + norm(u(:,i+1) - u(:,i))^2/Vhs(i)^2/dt;
end
options = sdpsettings('verbose',0,'solver','ecos');

figure(1)
hold on
grid on

for n = 1:MAX_ITER
    Cons = constraints;
    for i = 1:N+1
        Cons = [Cons, u_bar_normalized(:,i)'*u(:,i) >= Vhs(i) - eps_h];
    end
    sol = optimize(Cons, cost, options);
    if sol.problem ~= 1
        if abs(value(cost) - current_cost) < eps_convergence && value(cost)<5
            xs = value(x);
            us = value(u);
            for j = 1:N+1
                u_bar_normalized(:,j) = us(:,j)/norm(us(:,j));
            end
%             guidance(1:2,:) = xs(:,2:end);
%             guidance(3,:) = hs(2:end);
%             guidance(4,:) = atan2(us(2,2:end), us(1,2:end));
%             guidance(5,:) = 
% 
%             us = value(u);
%             flag = true;

            disp("At iteration "+num2str(n)+", final cost: "+num2str(value(cost)))
            disp([lambda1*(xs(1,end)^2 + xs(2,end)^2), lambda2 * (1-cos(psi_d)*us(1,end)/Vhs(end) - sin(psi_d)*us(2,end)/Vhs(end))])
            plot(xs(2,:), xs(1,:),'r')
            break
        else
            current_cost = value(cost);
            disp("At iteration "+num2str(n)+", current cost: "+num2str(current_cost))
            xs = value(x);
            plot(xs(2,:), xs(1,:),'k')
            for j = 1:N+1
                us = value(u);
                u_bar_normalized(:,j) = us(:,j)/norm(us(:,j));
            end
        end
    else
        disp("###############################################")
        disp("[ERROR] Something went wrong at iteration "+num2str(n));
        disp(sol.info)
        disp("###############################################")
    end
end 

%% Optimization Stage Two
disp("Turn in stage 2")
yalmip('clear')
x = sdpvar(2,N+1); % [x; y]
u = sdpvar(2,N+1); % [v1; v2]
eps_h = sdpvar(1);
cost = lambda1*(x(1,end)^2 + x(2,end)^2) + lambda2 * (1-cos(psi_d)*u(1,end)/Vhs(end) - sin(psi_d)*u(2,end)/Vhs(end)) + lambda3*eps_h;
constraints = [x(:,1) == [x0;y0], ...
               u(:,1) == u0, ...
               norm(u(:,N+1)) <= Vhs(N+1) + eps_h, eps_h>=0];
for i = 1:N
    constraints = [constraints, x(:,i+1) == x(:,i) + dt*[(u(1,i)+u(1,i+1))/2 + (Ws(1,i)+Ws(1,i+1))/2;
                                                         (u(2,i)+u(2,i+1))/2 + (Ws(2,i)+Ws(2,i+1))/2] ...
                              , norm(u(:,i)) <= Vhs(i) + eps_h ...
                              , norm(u(:,i+1) - u(:,i))/Vhs(i)/dt <= psi_dot_m];
    cost = cost + lambda3*norm(u(:,i+1) - u(:,i))^2/Vhs(i)^2/dt;
end
options = sdpsettings('verbose',0,'solver','ecos');

figure(2)
hold on
grid on

for n = 1:MAX_ITER
    Cons = constraints;
    for i = 1:N+1
        Cons = [Cons, u_bar_normalized(:,i)'*u(:,i) >= Vhs(i) - eps_h];
    end
    sol = optimize(Cons, cost, options);
    if sol.problem ~= 1
        if abs(value(cost) - current_cost) < eps_convergence*0.01
            xs = value(x);
            us = value(u);
            for j = 1:N+1
                u_bar_normalized(:,j) = us(:,j)/norm(us(:,j));
            end
%             guidance(1:2,:) = xs(:,2:end);
%             guidance(3,:) = hs(2:end);
%             guidance(4,:) = atan2(us(2,2:end), us(1,2:end));
%             guidance(5,:) = 
% 
%             us = value(u);
%             flag = true;

            disp("At iteration "+num2str(n)+", final cost: "+num2str(value(cost)))
            disp([lambda1*(xs(1,end)^2 + xs(2,end)^2), lambda2 * (1-cos(psi_d)*us(1,end)/Vhs(end) - sin(psi_d)*us(2,end)/Vhs(end))])
            plot(xs(2,:), xs(1,:),'r')
            break
        else
            current_cost = value(cost);
            disp("At iteration "+num2str(n)+", current cost: "+num2str(current_cost))
            xs = value(x);
            plot(xs(2,:), xs(1,:),'k')
            for j = 1:N+1
                us = value(u);
                u_bar_normalized(:,j) = us(:,j)/norm(us(:,j));
            end
        end
    else
        disp("###############################################")
        disp("[ERROR] Something went wrong at iteration "+num2str(n));
        disp(sol.info)
        disp("###############################################")
    end
end 

end