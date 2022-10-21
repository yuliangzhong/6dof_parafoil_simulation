% solving guidance by casadi + ipopt
% with safety constraints

function [flag, guidance] = GuidanceSolve(N, psi_d, vel_info, psi_dot_m, init_cond, heights, wind_profile_hat, Axbxh)

tic

%% Parameters Reading & Height Discretization
x0 = init_cond(1);
y0 = init_cond(2);
h0 = init_cond(3);
psi_0 = init_cond(4);

Vh = vel_info(1);
Vz = vel_info(2);
tf = h0/Vz;
dt = tf/(N-1);
hs = linspace(h0, 0, N);

Ws = [interp1(heights, wind_profile_hat(1,:), hs, 'linear','extrap');
      interp1(heights, wind_profile_hat(2,:), hs, 'linear','extrap')];

% id_p = floor(N*0.2); % 20% steps
% hp = hs(id_p);
% ratios = exp(-5*(h0*ones(1,N)-hs)/(h0-hp));
% Ws = Ws + [wind_dis(1); wind_dis(2)] .* ratios; % wind error xy

%% Compute Safezone Constraints
inds = zeros(size(Axbxh,1),1); % turn h to index
for i = 1:size(Axbxh,1)
    inds(i) = find(hs<Axbxh(i,4),1);
end

%% Optimization Start

lambda1 = 100;
lambda2 = 10;

Au = [1; -1];
bu = [psi_dot_m; psi_dot_m];

Prob = casadi.Opti();
x = Prob.variable(3, N);
u = Prob.variable(1, N);

% % set initial guess from guidance guess
% if sum(guidance_guess,'all') == 0
%     disp("WARNING! Solving guidance without initial guess!!")
% else
%     x_guess = [x0; y0; psi_0]*ones(1,N);
%     u_guess = psi_dot_now*ones(1,N);
%     for i = 1:N-1
%         u_last = interp1(guidance_guess(3,:),guidance_guess(5,:),hs(i),'spline', 'extrap');
%         % u_guess should satisfy constraints!
%         u_guess(i+1) = max([min([u_last, psi_dot_m, u_guess(i)+psi_ddot_m*dt]), -psi_dot_m, u_guess(i)-psi_ddot_m*dt]);
%         % compute x_guess from forward Euler
%         x_guess(:,i+1) = x_guess(:,i) + dt*[(Vh*cos(x_guess(3,i))+Vh*cos(x_guess(3,i+1)))/2 + (Ws(1,i)+Ws(1,i+1))/2;
%                                             (Vh*sin(x_guess(3,i))+Vh*sin(x_guess(3,i+1)))/2 + (Ws(2,i)+Ws(2,i+1))/2;
%                                             u_guess(i)];
%     end
%     Prob.set_initial(x, x_guess);
%     Prob.set_initial(u, u_guess);
% end

% costs and constraints
cost = lambda1*(x(1,end)^2 + x(2,end)^2) + lambda2*(1-cos(x(3,end)-psi_d));

% x1
Prob.subject_to(x(:,1) == [x0; y0; psi_0]);
% u1
Prob.subject_to(Au*u(1) <= bu);

for i = 1:N-1
    % x2~xN
    Prob.subject_to(x(:,i+1) == x(:,i) + dt*[(Vh*cos(x(3,i))+Vh*cos(x(3,i+1)))/2 + (Ws(1,i)+Ws(1,i+1))/2;
                                             (Vh*sin(x(3,i))+Vh*sin(x(3,i+1)))/2 + (Ws(2,i)+Ws(2,i+1))/2;
                                             u(i)]);
    % u2~uN
    cost = cost + u(i+1)^2*dt;
    Prob.subject_to(Au*u(i+1) <= bu); % control input constraints
%     % u2-u1 ~ uN-uN-1
%     Prob.subject_to(u(i+1)-u(i) <= dt*psi_ddot_m)
%     Prob.subject_to(u(i+1)-u(i) >= -dt*psi_ddot_m) % control input time derivitive constraints
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
try
    sol = Prob.solve();

    xs = sol.value(x);
    us = sol.value(u);

    pos_cost = lambda1*(xs(1,end)^2 + xs(2,end)^2);
    if pos_cost>400
        flag = false;
        guidance = zeros(5, N);
        disp("reject bad guidance!")
    else
        flag = true;
        guidance(1:2,1:N) = xs(1:2,1:N);
        guidance(3,1:N) = hs(1:N);
        guidance(4,1:N) = xs(3,1:N);
        guidance(5,1:N) = us(1:N);
    end

catch
    disp("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    disp("ERROR! Guidance solver failed!!")
    disp("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

    flag = false;
    guidance = zeros(5, N);
    
end

toc
text(init_cond(2), init_cond(1), num2str(round(toc,2)))

end


