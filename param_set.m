%% Environment
gravity_acc = + 9.81; % z-down

%% Wind Profile and Wind Gust Dynamics
vel_at6m = 2 * 0.5144; % wind velocity at 6m, absolute value, [knot]->[m/s]
wind_gust_max = 2.4 * 0.5144; % maximum wind gust from wind forcast, [knot]->[m/s]
theta = 45; % [deg] constant

wind_h = @(h) (h>0.04572)*vel_at6m.*log(h/0.04572)/log(6.096/0.04572); % wind shear model
theta_h = @(h) theta/180*pi + pi; % forcasted wind field
GetWindProfile = @(h) [wind_h(h).*cos(theta_h(h));
                       wind_h(h).*sin(theta_h(h));
                       zeros(1,size(h,2))]; % [wx; wy; 0];
wind_pf_size = 1500;
heights = linspace(1e-6, 150, wind_pf_size); % start from 0+ avoiding NaN
wind_profile_hat = GetWindProfile(heights);

xi = 0.0* [randn();
           randn();
           0]; % wind profile error at 200[m]

delta_w_init = [(wind_gust_max - vel_at6m)*randn(2,1); 0]; % delta_w in [x, y, z]
a_w = -0.00385;
b_w = 0.251;
sigma_zeta = b_w^2*diag([1, 1, 0.1]); % diagonal matrix
% tune sigma_zeta for gust simulation
% params from paper 14/16

%% Parafoil System
% params from paper 12

system_mass = 2.2; % kg
system_inertia =  [1.68, 0,    0.09;
                   0,    0.80, 0;
                   0.09, 0,    0.32];

parafoil_ref_area = 1.5;  % m2
parafoil_ref_span = 1.88; % m
parafoil_ref_length = 0.80; % m, chord
parafoil_arc_h = 0.1; % m
parafoil_thickness = 0.075; % m, thickness of canopy

S_pd = 0.06; % m2 payload reference area 

C_BP = eye(3); % from body frame B to parafoil aero frame P;
B_r_BP = [0; 0; -1.2];

C_BW = eye(3); % from body frame B to payload frame W;
B_r_BW = [0; 0; 0.1];

[Im, Ii] = ImIiCompute(parafoil_arc_h, parafoil_ref_span, ...
                       parafoil_ref_length, parafoil_thickness);

%% Aero Coefficients
%--- from paper 12 and book ch5
c_L0 = 0.24; % lift force coefficient
c_La = 2.14;
c_Lds = 0.2783;

c_D0 = 0.12;
c_Da2 = 0.33;
c_Dds = 0.43; % large enough to enable gliding ratio control 17.5-26.3

c_Yb = 0.23;

c_lp = -0.84; % negative
c_lda = -0.005; % negative
c_m0 = 0.1; 
c_ma = -0.72;
c_mq = -1.49; % negative
c_nr = -0.27; % negative
c_nda = 0.0115; % positive

c_Dpd = 3.2; % payload drag coefficient

cL = [c_L0; c_La; c_Lds]; % lift force coefficients
cD = [c_D0; c_Da2; c_Dds]; % drag force coefficients
cM = [c_lp; c_lda; c_m0; c_ma; c_mq; c_nr; c_nda]; % moment coefficients

%% Safe Zone and Initiation
[Axbxh, init_xy_pos] = SafeZoneCompute(0);
init_pos_in_inertial_frame = [init_xy_pos; -100]; % x-North, z-down, y-East
init_rpy = [0; 0; 0/180*pi]; % yaw-pitch-row; from ground to body frame; x-head, z-done, y-right
init_uvw = [3.88; 0; 1.62]; % velocity in body frame % shouldn't be all zero
init_pqr = [0; 0; 0]; % angular velocity in body frame

%% Sensor Model: Accuracy after Primary Sensor Fusion
sensor_freq = 20; % [Hz] sensor data subscriber frequency
pos_accu = 2; % [m]
vel_accu = 0.1; % [m/s]
row_pitch_accu = 0.1; % [degree]
yaw_accu = 0.5; % [degree]
acc_accu = 0.1; % [m/s^2]
angVel_accu = 0.1; % [deg/s]

% should tune white noise power in simulator for accuracy of airspeed, AOA, and AOS
lpf_f = 0.25; % [Hz] Low Pass Filter cut-off frequency of airspeed
airspeed_accu = 0.06; % [m/s] Accuracy after LPF

%% Extended Kalman Filter for States
% state X = [x, y, z, x_dot, y_dot, z_dot, row, pitch, yaw] 9*1
EKF_freq = sensor_freq; % [Hz]
state_mu0 = [init_pos_in_inertial_frame; 
             GroundSpeedCompute(init_rpy, init_uvw); 
             init_rpy]; % 9*1
state_sigma0 = blkdiag(4*eye(3), 2*eye(3), 0.4*eye(3)); % 9*9
Q = blkdiag(acc_accu^2*eye(3), (angVel_accu/180*pi)^2*eye(3)); % 6*6
R = blkdiag(pos_accu^2*eye(3), vel_accu^2*eye(3), ...
            diag([(row_pitch_accu/180*pi)^2, (row_pitch_accu/180*pi)^2, (yaw_accu/180*pi)^2])); % 9*9

%% Wind Estimator
mu0 = GetWindProfile(-init_pos_in_inertial_frame(3));
sigma0 = 0.5*eye(3); % wind variance initial guess
last_wind_pf0 = mu0; 
wind_est_dyn_var = 1.01 * (1/sensor_freq)^2 * sigma_zeta; % v ~ N(0, Q), Q matrix
wind_est_noise_var = airspeed_accu^2*eye(3); % d ~ N(0, R), R matrix, sensor noise, needs tuning
wind_err0 = zeros(4,15);

%% Guidance
psi_d = pi; % desired landing orientation

guidance_horizon_N = 200;
guidance0 = zeros(5,guidance_horizon_N);

vel_info = [95.551, 3.87807, 1.61823]; % corresponding height, Vh, Vz, without wind, delta_l,r = 0.5

psi_dot_m = 0.1906; % maximum turning angular vel without wind [rad/s]
delta_dot_m = 0.5; % [/s]
psi_ddot_m = psi_dot_m*2*delta_dot_m; % [rad/s2]

pd_controller_freq = 10; % [Hz]

%% MPCC Tracker
time_horizon_N = 100; % should not exceed 1000
% mpc_samping_T = 0.1; % [s]
control0 = zeros(3, time_horizon_N); % [h, psi, psi_dot]
% vel_info_mpcc = [4.82; 3.35; 1.52; 1.65]; % [m/s] [Vh0, Vh1, Vz0, Vz1]
% 






% %% Aerodynamic Coefficients Estimator
% aeroF_co_mu0 = [0 0 0; 
%                 0 0 0; 
%                 0 0 0;
%                 3 3 3]; % [cD; cYb; cL; cDpd], delta_s = 0; 0.5; 1;
% aeroF_co_sigma0 = 0.01*[diag([1,1,1,1]), diag([1,1,1,1]), diag([1,1,1,1])];
% aeroF_est_noise_var = 0.0005*eye(3);

% warning('off','MATLAB:polyfit:RepeatedPointsOrRescale') % mpcc