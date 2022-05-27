ifUseCurrentState = 1;

if ifUseCurrentState

    C_BI = out.C_BI.signals.values(:,:,end);
    I_r_IB = out.I_r_IB.signals.values(end,:);
    B_w_IB = out.B_w_IB.signals.values(end,:);
    
    
    h0 = -I_r_IB(3);
    x0 = I_r_IB(1);
    y0 = I_r_IB(2);
    C_IB = C_BI';
    psi0 = atan2(C_IB(2,1), C_IB(1,1));
    init_cond = [x0; y0; h0; psi0];
    phi_t = atan2(C_IB(3,2), C_IB(3,3));
    theta_t = atan2(-C_IB(3,1), sqrt(C_IB(3,2)^2) + C_IB(3,3)^2);
    psi_dot_now = B_w_IB(2)*sin(phi_t)/cos(theta_t) + B_w_IB(3)*cos(phi_t)/cos(theta_t);
    guidance_last = out.guidance.signals.values(:,:,end);

else
    psi_dot_now = 0;
    init_cond = [init_pos_in_inertial_frame(1:2); 100; 0];
    guidance_last = GuidanceGuess(guidance_horizon_N, init_pos_in_inertial_frame);
end

SafeZoneCompute(1) % for plot

% tune guidacne
[flag, guidance_tune] = GuidanceSolve(guidance_horizon_N, psi_d, vel_info, psi_dot_m, psi_ddot_m, init_cond, psi_dot_now, heights, wind_profile_hat, Axbxh, guidance_last);
plot3(guidance_tune(2,:),guidance_tune(1,:), guidance_tune(3,:))


