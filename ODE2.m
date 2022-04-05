% Use fminsearch to find p_0_star that yields x(T) = x_T
global init_cond;
init_cond = [700, -700, pi/2];
global psi_d 
psi_d = pi;

[p_0_star, residual, flag, output] = fminsearch(@(P0) ComputeErr(P0),zeros(3,1),optimset('TolFun',1e-7,'TolX',1e-7,'MaxFunEvals',1e5,'MaxIter',1e5));
disp('-------------')
disp(flag)
disp(residual)

t_span = linspace(0, 250, 1000);
x0 = [init_cond(1);
      init_cond(2);
      init_cond(3);
      p_0_star(1);
      p_0_star(2);
      p_0_star(3)];
[time, traj] = ode45(@(t,x) ODEdyn(t,x), t_span, x0);
err_f = [traj(end,1); traj(end,2); cos(traj(end,3) - psi_d)];
disp(err_f)
plot(traj(:,1), traj(:,2))


function dxdt = ODEdyn(t, x) % x = [x, y, psi, p1, p2, p3]^T
    V0 = 4.6;
    um = 0.2;

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
            0;
            0;
            x(4)*V0*sin(x(3)) - x(5)*V0*cos(x(3))];
end

function err = ComputeErr(P0)
    global init_cond;
    global psi_d;
    t_span = linspace(0, 250, 1000);
    x0 = [init_cond(1);
          init_cond(2);
          init_cond(3);
          P0(1);
          P0(2);
          P0(3)];
    [~, traj] = ode45(@(t,x) ODEdyn(t,x), t_span, x0);
    err = sqrt(traj(end,1)^2 + traj(end,2)^2) - (cos(traj(end,3) - psi_d) -1);
end