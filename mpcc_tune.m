% get current state
% load('restoration_failure.mat')

guidance = out.guidance.signals.values(:,:,end);
wind_err = out.wind_err.signals.values(:,:,end);
C_BI = out.C_BI.signals.values(:,:,end);
I_r_IB = out.I_r_IB.signals.values(end,:);
% B_w_IB = out.B_w_IB.signals.values(end,:);

dx = mean(wind_err(2,:));
dy = mean(wind_err(3,:));
dz = mean(wind_err(4,:));
wind_dis = [dx; dy; dz];

h0 = -I_r_IB(3);
x0 = I_r_IB(1);
y0 = I_r_IB(2);
C_IB = C_BI';
psi0 = atan2(C_IB(2,1), C_IB(1,1));
init_cond = [x0; y0; h0; psi0];
% phi_t = atan2(C_IB(3,2), C_IB(3,3));
% theta_t = atan2(-C_IB(3,1), sqrt(C_IB(3,2)^2) + C_IB(3,3)^2);
% psi_dot_now = B_w_IB(2)*sin(phi_t)/cos(theta_t) + B_w_IB(3)*cos(phi_t)/cos(theta_t);

init_dsda = [0.5; 0];


[flag, final_distance, control] = MPCC3d(time_horizon_N, mpcc_Ts, vel_info, vel_info_mpcc, psi_dot_m, delta_dot_m, ...
                  init_cond, init_dsda, guidance, heights, wind_profile_hat, wind_dis);
