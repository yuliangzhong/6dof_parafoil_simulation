function [I_v_IB, quat] = GroundSpeedCompute(rpy, B_v_IB)
    phi = rpy(1); % row
    theta = rpy(2); % pitch
    psi = rpy(3); % yaw

    C = [cos(theta)*cos(psi),    sin(phi)*sin(theta)*cos(psi)-cos(theta)*sin(psi),   cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
            cos(theta)*sin(psi),    sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),     cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
            -sin(theta),            sin(phi)*cos(theta),                                cos(phi)*cos(theta)];
    I_v_IB = C*B_v_IB;

    quat = 0.5*[sqrt(C(1,1) + C(2,2) + C(3,3) + 1);
                Sign(C(3,2)-C(2,3))*sqrt(C(1,1) - C(2,2) - C(3,3) +1);
                Sign(C(1,3)-C(3,1))*sqrt(C(2,2) - C(3,3) - C(1,1) +1);
                Sign(C(2,1)-C(1,2))*sqrt(C(3,3) - C(1,1) - C(2,2) +1)];
end

function s = Sign(x)
    if x>=0
        s = 1;
    else
        s = -1;
    end
end
