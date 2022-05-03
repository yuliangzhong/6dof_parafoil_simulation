NN = 100;
TTs = 0.1;
sys_param = [Vh; Vz; psi_dot_m];
% get current state
guidance = out.guidance.signals.values(:,:,end);
wind_err = out.wind_err.signals.values(:,:,end);
C_BI = out.C_BI.signals.values(:,:,end);
I_r_IB = out.I_r_IB.signals.values(end,:);
dx = mean(wind_err(2,:));
dy = mean(wind_err(3,:));
dz = mean(wind_err(4,:));
wind_error = [dx; dy; dz];

h0 = -I_r_IB(3);
x0 = I_r_IB(1);
y0 = I_r_IB(2);
C_IB = C_BI';
psi0 = atan2(C_IB(2,1), C_IB(1,1));
init_cond = [x0; y0; h0; psi0];


[flag, xs, us] = MPCC(NN, TTs, sys_param, init_cond, wind_error, guidance, heights, wind_profile_hat)
