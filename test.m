N = 100;
init_cond = [200, -200, 300, pi/2];
psi_dot0 = 0;
wind_dis = [0,0];


[flag, guidance] = GuidanceSolve(N, vel_info, psi_dot_m, init_cond, psi_d, psi_dot0, heights, wind_profile_hat, wind_dis)

HHs = linspace(100,0,1000);
ge = [interp1(guidance(3,:),guidance(1,:),HHs,'spline','extrap');
      interp1(guidance(3,:),guidance(2,:),HHs,'spline','extrap')];
hold on
grid on
plot(ge(2,:), ge(1,:), 'LineWidth',1);
scatter(guidance(2,:), guidance(1,:), 'red', 'filled')
