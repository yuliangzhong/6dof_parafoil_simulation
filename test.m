N = 100;
sys_param = [4.59, 1.39, 0.2187];
init_cond = [200, -200, 300, pi/2];
psi_d = pi;
psi_dot0 = 0;


[flag, guidance] = GuidanceSolve(N, sys_param, init_cond, psi_d, psi_dot0, heights, wind_profile_hat)
