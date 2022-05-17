function I_v_IB = GroundSpeedCompute(rpy, B_v_IB)
    phi = rpy(1); % row
    theta = rpy(2); % pitch
    psi = rpy(3); % yaw

    C = [cos(theta)*cos(psi),    sin(phi)*sin(theta)*cos(psi)-cos(theta)*sin(psi),   cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
         cos(theta)*sin(psi),    sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),     cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
         -sin(theta),            sin(phi)*cos(theta),                                cos(phi)*cos(theta)];
    I_v_IB = C*B_v_IB;
end