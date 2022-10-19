%% Guidance example
% Use fminsearch to find p_0_star that yields x(T) = x_T
hold on
grid on
box on
axis equal

%% Draw 2 circles
R1 = 150;
R2 = 200;
psi_0 = 0;

testN = 6;
thetas = linspace(0, 2*pi*(1-1/testN), testN);
init_states = [R1*cos(thetas), R2*cos(thetas);
              R1*sin(thetas), R2*sin(thetas);
              psi_0*ones(1,2*testN)];

viscircles(zeros(2,2),[R1;R2],'Color','k','LineStyle','--','LineWidth',1.5);

%% Init
V0 = 2.81/1.23;
um = 0.263/1.23;

z0 = -100;
final_states = [0, 0, pi*5/6];
scatter(final_states(2), final_states(1),50, 'k','filled')
quiver(0,0,40*sin(final_states(3)),40*cos(final_states(3)),3,'k','LineWidth',3)

MAX_ITER = 5;

set(gca,'FontSize',20)
xlabel('\rightarrow East [m]')
ylabel('\rightarrow North [m]')
title('Guidance in xy-plane')

%% Guidance planner robustness test
for i = 1:2*testN
    init_pose = init_states(:,i);

    % solve guidance plan for each initial state
    p_0_guess = 0.001*(rand(1,3)-0.5*ones(1,3));
    p_0_star = zeros(1,3);
    residual_tmp = inf;
    p_0_star_tmp = zeros(1,3);
    flag = true;

    tic
    for j = 1:MAX_ITER
        [p_0_star, residual, flag] = fminsearch(@(P0) ComputeErr(P0, init_pose, final_states, z0, V0, um),p_0_guess,optimset('TolFun',1e-7,'TolX',1e-7,'MaxFunEvals',1e5,'MaxIter',1e5));
        if ~flag
            disp("guidance solver failed in iteration "+num2str(j)+", No. "+num2str(i))
            scatter(init_pose(2), init_pose(1), 50, 'r','filled')
            text(init_pose(2)+1, init_pose(1), '[Failed]','HorizontalAlignment', 'left', 'VerticalAlignment', 'top','FontSize', 16)
            break
        elseif abs(residual) > 1
            disp("Unhappy result in iteration "+num2str(j)+", No. "+num2str(i)+", search again")
            if residual < residual_tmp
                p_0_star_tmp = p_0_star;
            end
            p_0_guess = 0.001*(rand(1,3)-0.5*ones(1,3));
            if j==MAX_ITER
                p_0_star = p_0_star_tmp;
                scatter(init_pose(2), init_pose(1),50,'r','filled')
                text(init_pose(2)+1, init_pose(1), '[Bad]','HorizontalAlignment', 'left', 'VerticalAlignment', 'top','FontSize', 16)
                % rebuild
                z_span = linspace(z0, 0, 2000);
                x0 = [init_pose(1);
                      init_pose(2);
                      init_pose(3);
                      p_0_star(1);
                      p_0_star(2);
                      p_0_star(3)];
                [time, path] = ode45(@(z,x) ODEdyn(z,x, V0, um), z_span, x0);
                plot(path(:,2), path(:,1),'r','LineWidth',1.5)
            end
        else
            scatter(init_pose(2), init_pose(1),50,'g','filled')
            text(init_pose(2)+1, init_pose(1), '[Good]','HorizontalAlignment', 'left', 'VerticalAlignment', 'top','FontSize', 16)
            disp("Good result found in iteration "+num2str(j)+", No. "+num2str(i))
            % rebuild
            z_span = linspace(z0, 0, 2000);
            x0 = [init_pose(1);
                  init_pose(2);
                  init_pose(3);
                  p_0_star(1);
                  p_0_star(2);
                  p_0_star(3)];
            [time, path] = ode45(@(z,x) ODEdyn(z,x, V0, um), z_span, x0);
            plot(path(:,2), path(:,1),'k','LineWidth',1.5)
            break
        end
    end

    toc
    text(init_pose(2)+1, init_pose(1), num2str(toc), 'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom','FontSize', 16) % time cost
end

%% Dynamics
function dxdz = ODEdyn(z, x, V0, um) % x = [x, y, psi, p1, p2, p3]^T
%     lambda = 1e-9;
    lambda = 0;
    if(x(6) >= 2*um)
        u = -um;
    elseif(x(6) >= -2*um)
        u = -x(6)/2;
    else
        u = um;
    end
    dxdz = [V0*cos(x(3));
            V0*sin(x(3));
            u;
            -2*lambda*x(1);
            -2*lambda*x(2);
            x(4)*V0*sin(x(3)) - x(5)*V0*cos(x(3))];
end

%% Error

function err = ComputeErr(P0, init_pose, final_pose, z0, V0, um)
    t_span = linspace(z0, 0, 1000);
    x0 = [init_pose(1);
          init_pose(2);
          init_pose(3);
          P0(1);
          P0(2);
          P0(3)];
    [~, path] = ode45(@(t,x) ODEdyn(t,x, V0, um), t_span, x0);
    err = sqrt((path(end,1) - final_pose(1))^2 + (path(end,2) - final_pose(2))^2) + 1e2*(1-cos(path(end,3) - final_pose(3)));
end
