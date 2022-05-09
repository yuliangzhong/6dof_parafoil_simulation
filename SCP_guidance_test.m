% scp guidance test
%% Parameters
N = 200;
psi_dot_m = 0.2187;
h0 = 262;
Vh0 = 4.59;
Vz0 = 1.39;
x0 = 200;
y0 = -200;
psi_0 = pi*2/4;

sys_param0 = [Vh0, Vz0, psi_dot_m];
init_cond = [x0, y0, h0, psi_0];
psi_d = pi;
SCP_guidance(N, sys_param0, init_cond, psi_d, heights, wind_profile_hat);
