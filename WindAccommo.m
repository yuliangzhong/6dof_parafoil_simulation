function wind_pos_drift = WindAccommo(h, Vz)
    % position drift caused by wind
    % let this shit integrator work in simulink!
    N = 1000000;
    z = linspace(0.04572, h, N);
    dz = z(2) - z(1);
    fx = @(z) vel_at6m*log(z/0.04572)/log(6.096/0.04572).*cos(- 5/6*pi + 1/6*pi * z/300);
    fy = @(z) vel_at6m*log(z/0.04572)/log(6.096/0.04572).*sin(- 5/6*pi + 1/6*pi * z/300);
    xw = fx(z)*ones(N,1)*dz/Vz;
    yw = fy(z)*ones(N,1)*dz/Vz;
    wind_pos_drift = [xw; yw; 0];
end