%% Environment
gravity_acc = + 9.81; % z-down
laminar_wind_vel_at6m = 1; % absolute value
laminar_wind_ori_at6m = 45; % 0-north, clockwise, degree
turbulence_vel_at6m = 0; % absolute value
turbulence_ori_at6m = 0; % 0-north, clockwise, degree

%% Parafoil System
system_mass = 2.2; % kg
system_inertia =  [1.68, 0,    0.09;
                   0,    0.80, 0;
                   0.09, 0,    0.32];

parafoil_ref_area = 1.5;  % m2
parafoil_ref_span = 1.88; % m
parafoil_ref_length = 0.80; % m, chord
parafoil_arc_h = 0.1; % m
parafoil_thickness = 0.075; % m, thickness of canopy

S_pd = 0.06; % payload reference area

C_BP = eye(3); % from body frame B to parafoil aero frame P;
B_r_BP = [0; 0; -1.0];

C_BW = eye(3); % from body frame B to payload frame W;
B_r_BW = [0; 0; 0.1];

[Im, Ii] = ImIiCompute(parafoil_arc_h, parafoil_ref_span, ...
                    parafoil_ref_length, parafoil_thickness);


%% Aero Coefficients
c_L0 = 0.24; % lift force coefficient
c_LA = 2.14;
c_LBs = 0.4670;
c_LBa = 0.2350;

c_D0 = 0.12;
c_DA2 = 0.33;
c_DBs = 0.43;
c_DBa = 0.0957;

c_Yb = 0.23;

c_lp = -0.84; % negative
c_lBa = 0.005;
% c_m0 = -0.05; % highly influence alpha! compensate apparent mass moment in Y
% c_mA = -0.70; % highly influence alpha! compensate apparent mass moment in Y
c_m0 = 0; c_mA = 0;

c_mq = -1.49; % negative
c_nr = -0.27; % negative
c_nBa = 0.0115;

c_Dpd = 0.5; % payload drag coefficient

cL = [c_L0; c_LA; c_LBs; c_LBa]; % lift force coefficients
cD = [c_D0; c_DA2; c_DBs; c_DBa]; % drag force coefficients
cM = [c_lp; c_lBa; c_m0; c_mA; c_mq; c_nr; c_nBa]; % moment coefficients

%% Initiation
% init_pos_in_inertial_frame = [-400, -400, -250]; % x-desired heading, z-down
% init_rpy = [0,      0.002,  pi/2]; % yaw-pitch-row; from ground to body frame; x-head, z-done, y-right
% init_uvw = [5.62,   0,      1.25]; % velocity in body frame
% init_pqr = [0,  0,  0]; % angular velocity in body frame

init_pos_in_inertial_frame = [-400, -400, -250]; % x-desired heading, z-down
init_rpy = [0,      0.27,  pi/2]; % yaw-pitch-row; from ground to body frame; x-head, z-done, y-right
init_uvw = [4.05,   0,      1.91]; % velocity in body frame
init_pqr = [0,  0,  0]; % angular velocity in body frame

%% Observer accuracy(accu)
sampling_T = 0.5; % [seconds]
row_pitch_accu = 0.2; % [degree]
yaw_accu = 1; % [degree]
pos_accu = 1; % [m]
vel_accu = 0.05; % [m/s]
acc_accu = 0.5; % [m/s^2]
angVel_accu = 0.02; %[deg/s]
airspeed_var = [0.001, 0.001, 0.001]; % [alpha, beta, Vb], no unit

%% Wind Estimator
mu0 = -[laminar_wind_vel_at6m*cos(laminar_wind_ori_at6m/180*pi); 
       laminar_wind_vel_at6m*sin(laminar_wind_ori_at6m/180*pi);
       0];
wind_est_noise_var = 0.5*eye(3); % d ~ N(0, R)

%% useful function
function [Im, Ii] = ImIiCompute(a, b, c, t)

    a_bar = a/b;
    t_bar = t/c;
    AR = b/c;
    A = 0.666 * (1 + 8/3*a_bar^2) * t^2 * b;
    B = 0.267 * (t^2 + 2*a^2*(1-t_bar^2))*c;
    C = 0.785 * sqrt(1 + 2*a_bar^2*(1-t_bar^2))*AR/(1+AR)*b*c^2;
    D = 0.055 * AR/(1+AR)*b^3*c^2;
    E = 0.0308 * AR/(1+AR)*(1 + pi/6*(1+AR)*AR*a_bar^2*t_bar^2)*b*c^4;
    F = 0.0555 * (1+8*a_bar^2)*t^2*b^3;
    Im = diag([A, B, C]);
    Ii = diag([D, E, F]);

end
