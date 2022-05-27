% Guidance example
% Use fminsearch to find p_0_star that yields x(T) = x_T

testN = 8;
R1 = 700;
R2 = 900;
psi_0 = 0;
thetas = linspace(0, 2*pi*(1-1/testN), testN);
init_poses = [R1*cos(thetas), R2*cos(thetas);
              R1*sin(thetas), R2*sin(thetas);
              psi_0*ones(1,2*testN)];

V0 = 4.59/1.39;
um = 0.2187/1.39;
T = 300;
final_pose = [0, 0, pi];
scatter(final_pose(2), final_pose(1),'g','filled')

MAX_ITER = 5;

hold on
grid on
% Guidance planner robustness test
for i = 1:2*testN
    init_pose = init_poses(:,i);

    % solve guidance plan for each initial state
    p_0_guess = zeros(1,3);
    p_0_star = zeros(1,3);
    residual_tmp = inf;
    p_0_star_tmp = zeros(1,3);
    flag = true;

    tic
    for j = 1:MAX_ITER
        [p_0_star, residual, flag] = fminsearch(@(P0) ComputeErr(P0, init_pose, final_pose, T, V0, um),p_0_guess,optimset('TolFun',1e-7,'TolX',1e-7,'MaxFunEvals',1e5,'MaxIter',1e5));
        if ~flag
            disp("guidance solver failed in iteration "+num2str(j)+", No. "+num2str(i))
            scatter(init_pose(2), init_pose(1), 'r')
            text(init_pose(2)+100, init_pose(1)+100, '[failed]')
            break
        elseif abs(residual) > 1
            disp("Unhappy result in iteration "+num2str(j)+", No. "+num2str(i)+", search again")
            if residual < residual_tmp
                p_0_star_tmp = p_0_star;
            end
            p_0_guess = 0.001*rand(1,3);
            if j==MAX_ITER
                p_0_star = p_0_star_tmp;
                text(init_pose(2)+100, init_pose(1)+100, '[unhappy]')
            end
        else
            disp("Good result found in iteration "+num2str(j)+", No. "+num2str(i))
            break
        end
    end

%     p1 = optimizableVariable('p1',[-0.005,0.005]);
%     p2 = optimizableVariable('p2',[-0.005,0.005]);
%     p3 = optimizableVariable('p3',[-0.005,0.005]);
% 
%     fun = @(x) GuidanceSolve(x,init_pose, final_pose, T, V0, um);
%     results = bayesopt(fun,[p1, p2, p3]);

    % rebuild
    if flag
        t_span = linspace(0, T, 2000);
        x0 = [init_pose(1);
              init_pose(2);
              init_pose(3);
              p_0_star(1);
              p_0_star(2);
              p_0_star(3)];
        [time, traj] = ode45(@(t,x) ODEdyn(t,x, V0, um), t_span, x0);
        plot(traj(:,2), traj(:,1))
    end
    toc
    text(init_pose(2)-1, init_pose(1)-1, num2str(toc))
end

function dxdt = ODEdyn(t, x, V0, um) % x = [x, y, psi, p1, p2, p3]^T
    lambda = 1e-9;
    if(x(6) >= 2*um)
        u = -um;
    elseif(x(6) >= -2*um)
        u = -x(6)/2;
    else
        u = um;
    end
    dxdt = [V0*cos(x(3));
            V0*sin(x(3));
            u;
            -2*lambda*x(1);
            -2*lambda*x(2);
            x(4)*V0*sin(x(3)) - x(5)*V0*cos(x(3))];
end

function err = ComputeErr(P0, init_pose, final_pose, T, V0, um)
    t_span = linspace(0, T, 1000);
    x0 = [init_pose(1);
          init_pose(2);
          init_pose(3);
          P0(1);
          P0(2);
          P0(3)];
    [~, traj] = ode45(@(t,x) ODEdyn(t,x, V0, um), t_span, x0);
    err = sqrt((traj(end,1) - final_pose(1))^2 + (traj(end,2) - final_pose(2))^2) + 1e2*(1-cos(traj(end,3) - final_pose(3)));
end

% function [residual, p_0_star, flag] = GuidanceSolve(x, init_pose, final_pose, T, V0, um)
%     [p_0_star, residual, flag] = fminsearch(@(P0) ComputeErr(P0, init_pose, final_pose, T, V0, um),...
%                                                   [x.p1, x.p2, x.p3],...
%                                                   optimset('TolFun',1e-7,'TolX',1e-7,'MaxFunEvals',1e5,'MaxIter',1e5));
% end



% function err = ComputeErr(P0, init_pose, final_pose, T, V0, um)
%     a1 = 100;
%     a2 = 10;
%     t_span = linspace(0, T, 1000);
%     x0 = [init_pose(1);
%           init_pose(2);
%           init_pose(3);
%           P0(1);
%           P0(2);
%           P0(3)];
%     [~, traj] = ode45(@(t,x) ODEdyn(t,x, V0, um), t_span, x0);
%     err = (traj(4,end)-2*a1*traj(1,end))^2 + (traj(5,end)-2*a1*traj(2,end))^2 ...
%         + (traj(6,end) - a2*sin(traj(3,end) - final_pose(3)))^2;
% end