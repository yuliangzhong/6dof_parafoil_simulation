%% Environment
gravity_acc = + 9.81; % z-down
laminar_wind_vel_at6m = 1; % absolute value
laminar_wind_ori_at6m = 0; % 0-north, clockwise, degree
turbulence_vel_at6m = 0.5; % absolute value
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

c_D0 = 0.12;
c_DA2 = 0.33;
c_DBs = 0.43;

c_Yb = 0.23;

c_lp = -0.84; % negative
c_lBa = -0.005; % negative
% c_m0 = -0.05; % highly influence alpha! compensate apparent mass moment in Y
% c_mA = -0.70; % highly influence alpha! compensate apparent mass moment in Y
c_m0 = 0; c_mA = 0;

c_mq = -1.49; % negative
c_nr = -0.27; % negative
c_nBa = 0.0115; % positive

c_Dpd = 0.5; % payload drag coefficient

cL = [c_L0; c_LA; c_LBs]; % lift force coefficients
cD = [c_D0; c_DA2; c_DBs]; % drag force coefficients
cM = [c_lp; c_lBa; c_m0; c_mA; c_mq; c_nr; c_nBa]; % moment coefficients

%% Initiation
% init_pos_in_inertial_frame = [-400, -400, -250]; % x-North, z-down, y-East
% init_rpy = [0,      0,  pi/2]; % yaw-pitch-row; from ground to body frame; x-head, z-done, y-right
% init_uvw = [5.61,   0,  1.25]; % velocity in body frame
% init_pqr = [0,  0,  0]; % angular velocity in body frame

init_pos_in_inertial_frame = [-400, -400, -250]; % x-North, z-down, y-East
init_rpy = [0,      0.27,  pi/2]; % yaw-pitch-row; from ground to body frame; x-head, z-done, y-right
init_uvw = [4.05,   0,      1.90]; % velocity in body frame
init_pqr = [0,  0,  0]; % angular velocity in body frame

%% Observer accuracy(accu)
sampling_T = 0.5; % [seconds]
row_pitch_accu = 0.2; % [degree]
yaw_accu = 1; % [degree]
pos_accu = 1; % [m]
vel_accu = 0.05; % [m/s]
acc_accu = 0.5; % [m/s^2]
angVel_accu = 0.02; %[deg/s]
airspeed_var = [0.001, 0.001, 0.005]; % [alpha, beta, Vb], no unit
                                      % about [2.3deg, 2.3deg, 0.05m/s]

%% Wind Estimator
mu0 = -[laminar_wind_vel_at6m*cos(laminar_wind_ori_at6m/180*pi); 
       laminar_wind_vel_at6m*sin(laminar_wind_ori_at6m/180*pi);
       0]; % wind mean initial guess
sigma0 = eye(3); % wind variance initial guess
wind_est_noise_var = 0.5*eye(3); % d ~ N(0, R), R matrix, sensor noise

%% Aerodynamic Coefficients Estimator
aeroF_co_mu0 = [0 0 0; 
                0 0 0; 
                0 0 0;
                0 0 0]; % [cD; cYb; cL; cDpd], delta_s = 0; 0.5; 1;
aeroF_co_sigma0 = 0.05*[eye(4), eye(4), eye(4)];
aeroF_est_noise_var = 0.05*eye(3);






