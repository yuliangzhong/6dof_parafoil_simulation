%% Environment
gravity_acc = + 9.81; % z-down
laminar_wind_vel_at6m = 10; % absolute value
laminar_wind_ori_at6m = 0; % 0-north, clockwise, degree
turbulence_vel_at6m = 3; % absolute value
turbulence_ori_at6m = 0; % 0-north, clockwise, degree

%% Parafoil System
system_mass = 1;
system_inertia =  [3.14, 0,    1.58;
                     0,    5.12, 0;
                     1.58, 0,    9.75];
parafoil_ref_area = 0.01;
parafoil_ref_span = 0.01;
parafoil_ref_length = 0.01;
wingspan = 10;

%% Aero Coefficients
C_L = 1; % lift force coefficient
C_D = 1; % drag force coefficient
C_c = 1; % cross-wind force coefficient, 
C_rm = 1; % rolling moment coefficient
C_pm = 1; % pitching moment coefficient
C_ym = 1; % yawing moment coefficient
coefficients = [C_D, C_c, C_L, C_rm, C_pm, C_ym];

%% Initiation
init_pos_in_inertial_frame = [0, 0, -100]; % x-desired pose, z-down
init_rpy = [0,0,0]; % yaw-pitch-row; from ground to body frame; x-head, z-done, y-right
init_uvw = [1,1,1]; % velocity in body frame
init_pqr = [0, 0, 0]; % angular velocity in body frame
