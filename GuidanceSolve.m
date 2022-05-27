% solving guidance by casadi + ipopt
% with safety constraints

function [flag, guidance] = GuidanceSolve(N, psi_d, vel_info, psi_dot_m, psi_ddot_m, init_cond, psi_dot_now, heights, wind_profile_hat, Axbxh, guidance_guess)

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

%% Compute Safezone Constraints
inds = zeros(size(Axbxh,1),1); % turn h to index
for i = 1:size(Axbxh,1)
    inds(i) = find(hs<Axbxh(i,4),1);
end

%% Optimization Start

lambda1 = 100;
lambda2 = 100;

Au = [1; -1];
bu = [psi_dot_m; psi_dot_m];

Prob = casadi.Opti();
x = Prob.variable(3, N);
u = Prob.variable(1, N);

% set initial guess from guidance guess
if sum(guidance_guess,'all') == 0
    disp("WARNING! Solving guidance without initial guess!!")
else
%     try
        x_guess = [x0; y0; psi_0]*ones(1,N);
        u_guess = psi_dot_now*ones(1,N);
        for i = 1:N-1
            u_last = interp1(guidance_guess(3,:),guidance_guess(5,:),hs(i),'spline', 'extrap');
            % u_guess should satisfy constraints!
            u_guess(i+1) = max([min([u_last, psi_dot_m, u_guess(i)+psi_ddot_m*dt]), -psi_dot_m, u_guess(i)-psi_ddot_m*dt]);
            % compute x_guess from forward Euler
            x_guess(:,i+1) = x_guess(:,i) + dt*[(Vhs(i)*cos(x_guess(3,i))+Vhs(i+1)*cos(x_guess(3,i+1)))/2 + (Ws(1,i)+Ws(1,i+1))/2;
                                                (Vhs(i)*sin(x_guess(3,i))+Vhs(i+1)*sin(x_guess(3,i+1)))/2 + (Ws(2,i)+Ws(2,i+1))/2;
                                                u_guess(i)];
        end
        Prob.set_initial(x, x_guess);
        Prob.set_initial(u, u_guess);
%     catch
%         disp("ERROR! Guidance guess should be interpolable!!")
%         disp("WARNING! Solving guidance without initial guess!!")
%     end
end

% costs and constraints
cost = lambda1*(x(1,end)^2 + x(2,end)^2) + lambda2*(1-cos(x(3,end)-psi_d));

% x1
Prob.subject_to(x(:,1) == [x0; y0; psi_0]);
% u1
Prob.subject_to(u(1) == psi_dot_now);

for i = 1:N-1
    % x2~xN
    Prob.subject_to(x(:,i+1) == x(:,i) + dt*[(Vhs(i)*cos(x(3,i))+Vhs(i+1)*cos(x(3,i+1)))/2 + (Ws(1,i)+Ws(1,i+1))/2;
                                             (Vhs(i)*sin(x(3,i))+Vhs(i+1)*sin(x(3,i+1)))/2 + (Ws(2,i)+Ws(2,i+1))/2;
                                             u(i)]);
    % u2~uN
    cost = cost + u(i+1)^2*dt;
    Prob.subject_to(Au*u(i+1) <= bu); % control input constraints
    % u2-u1 ~ uN-uN-1
    Prob.subject_to(u(i+1)-u(i) <= dt*psi_ddot_m)
    Prob.subject_to(u(i+1)-u(i) >= -dt*psi_ddot_m) % control input time derivitive constraints
end

% constraints of safezone
for j = 1:size(Axbxh,1)
    % x_ind ~ xN
    for k = inds(j):N
        Prob.subject_to(Axbxh(j,1:2)*x(1:2,k) <= Axbxh(j,3));
    end
end

Prob.minimize(cost)
Prob.solver('ipopt', struct('print_time', 0), struct('print_level', 0));

% solve the guidance problem & give output
% try
    sol = Prob.solve();

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
    for i = 1:N-1
    % u2~uN
        cost3 = cost3 + us(i+1)^2*dt;
    end
    disp([cost1, cost2, cost3])
    flag = true;

% catch
%     disp("ERROR! Guidance solver failed!!")
%     flag = false;
%     guidance = zeros(5, N);
% end

toc
text(init_cond(2), init_cond(1), num2str(round(toc,2)))

end


