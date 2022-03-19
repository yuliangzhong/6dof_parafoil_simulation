%% Environment
gravity_acc = + 9.81; % z-down
laminar_wind_vel_at6m = 10; % absolute value
laminar_wind_ori_at6m = 0; % 0-north, clockwise, degree
turbulence_vel_at6m = 3; % absolute value
turbulence_ori_at6m = 0; % 0-north, clockwise, degree

%% Parafoil System
system_mass = 1;
system_inertia =  [1.68, 0,    -0.05;
                   0,    0.80, 0;
                   -0.05,0,    0.32];
parafoil_ref_area = 1.5;  % m2
parafoil_ref_span = 1.88; % m
parafoil_ref_length = 0.8;

C_BP = eye(3); % from body frame B to parafoil aero frame P;
B_r_BP = [0; 0; -1.2];

%% Aero Coefficients
c_L0 = 0.205; % lift force coefficient
c_LA = 0.0389;
c_LBs = 0.4670;
c_LBa = 0.2350;
c_D0 = 0.1150;
c_DA2 = 0.0083;
c_DBs = 0.1900;
c_DBa = 0.0957;

c1 = [c_L0; c_LA; c_LBs; c_LBa];
c2 = [c_D0; c_DA2; c_DBs; c_DBa];
coefs = [c1, c2];
% 
% C_D = 1; % drag force coefficient
% C_c = 1; % cross-wind force coefficient, 
% C_rm = 1; % rolling moment coefficient
% C_pm = 1; % pitching moment coefficient
% C_ym = 1; % yawing moment coefficient
% coefficients = [C_D, C_c, C_L, C_rm, C_pm, C_ym];

%% Initiation
init_pos_in_inertial_frame = [0, 0, -100]; % x-desired pose, z-down
init_rpy = [0,0,0]; % yaw-pitch-row; from ground to body frame; x-head, z-done, y-right
init_uvw = [1,1,1]; % velocity in body frame
init_pqr = [0, 0, 0]; % angular velocity in body frame
