% solving guidance by casadi + ipopt
% with safety constraints

function [flag, guidance] = GuidanceSolve(ifhard, N, psi_d, vel_info, psi_dot_m, psi_ddot_m, init_cond, psi_dot0, heights, wind_profile_hat, Ax, bx, old_guidance)

tic

%% Parameters Reading
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
Vh0 = sqrt(rho(vel_info(1))*vel_info(2)^2/rho(h0));
Vz0 = sqrt(rho(vel_info(1))*vel_info(3)^2/rho(h0));
h = @(t) 1/cz*(ones(1,size(t,2)) - (cz*cf*Vz0*sqrt(rho(h0))/sqrt(ch)*t + (1-cz*h0)^cf*ones(1,size(t,2))).^(1/cf));
Vh = @(h) Vh0*sqrt(rho(h0)./rho(h));

% time gap
tf = sqrt(ch)/(cz*cf*Vz0*sqrt(rho(h0)))*(1-(1-cz*h0)^cf);
dt = tf/(N-1);
% time mesh
times = linspace(0, tf, N);
hs = h(times);
Vhs = Vh(hs);
Ws = [interp1(heights, wind_profile_hat(1,:), hs, 'linear','extrap');
      interp1(heights, wind_profile_hat(2,:), hs, 'linear','extrap')];

%% Optimization

lambda1 = 100;
lambda2 = 10;
lambda3 = 0.001;
lambda4 = 0.1;
lambda5 = 1;

Au = [1; -1];
bu = [psi_dot_m; psi_dot_m];

M = diag([1/norm(Ax(1,:)),1/norm(Ax(2,:)),1/norm(Ax(3,:)),1/norm(Ax(4,:))]);

Prob = casadi.Opti();
x = Prob.variable(3, N);
u = Prob.variable(1, N);

% set initial guess from old guidance
if sum(old_guidance==0, 'all')<3
    x_guess = zeros(3,N);
    u_guess = zeros(1,N);
    for i = 1:N
        x_guess(:,i) = [interp1(old_guidance(3,:), old_guidance(1,:),hs(i),'linear','extrap');
                        interp1(old_guidance(3,:), old_guidance(2,:),hs(i),'linear','extrap');
                        interp1(old_guidance(3,:), old_guidance(4,:),hs(i),'linear','extrap')];
        u_guess(i) = interp1(old_guidance(3,:),old_guidance(5,:),hs(i),'linear', 'extrap');
    end
    Prob.set_initial(x, x_guess);
    Prob.set_initial(u, u_guess);
end

% costs and constraints
cost = lambda1*(x(1,end)^2 + x(2,end)^2) + lambda2*(1-cos(x(3,end)-psi_d));

% x1
Prob.subject_to(x(:,1) == [x0; y0; psi_0]);
% u1
Prob.subject_to(u(1) == psi_dot0);

for i = 1:N-1
    % x2~xN
    Prob.subject_to(x(:,i+1) == x(:,i) + dt*[(Vhs(i)*cos(x(3,i))+Vhs(i+1)*cos(x(3,i+1)))/2 + (Ws(1,i)+Ws(1,i+1))/2;
                                             (Vhs(i)*sin(x(3,i))+Vhs(i+1)*sin(x(3,i+1)))/2 + (Ws(2,i)+Ws(2,i+1))/2;
                                             u(i)]);
    % x2~xN
    if ifhard
        Prob.subject_to(Ax*x(1:2,i+1) <= bx); 
        cost = cost + lambda3*u(i)^2*dt;
    else
        cost = cost + lambda3*u(i)^2*dt ...
                    + lambda4*(sum(exp(lambda5*M*(Ax*x(1:2,i+1)-bx))));
    end

    % u2~uN
    Prob.subject_to(Au*u(i+1) <= bu); % control input constraints
    % u2-u1 ~ uN-uN-1
    Prob.subject_to(((u(i+1)-u(i))/dt)^2 <= psi_ddot_m^2) % control input time derivitive constraints
end

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

    % for debugging / tuning
    cost1 = lambda1*(xs(1,end)^2 + xs(2,end)^2);
    cost2 = lambda2*(1-cos(xs(3,end)-psi_d));
    cost3 = 0;
    cost4 = 0;
    for i = 1:N-1
    % x2~xN
        cost3 = cost3 + lambda3*us(i)^2*dt;
        cost4 = cost4 + lambda4*(sum(exp(lambda5*M*(Ax*xs(1:2,i+1)-bx))));
    end
    disp([cost1, cost2, cost3, cost4])

else
    disp("ERROR! Guidance solver failed!!")
end

toc
text(init_cond(2), init_cond(1), num2str(toc))

end


