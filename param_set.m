%% Environment
gravity_acc = + 9.81; % z-down

%% Wind Profile and Wind Gust Dynamics
vel_at6m = 2 * 0.5144; % wind velocity at 6m, absolute value, [knot]->[m/s]
wind_gust_max = 2.4 * 0.5144; % maximum wind gust from wind forcast, [knot]->[m/s]
theta = 28; % [deg] constant
wind_pf_size = 3000;
height_lim = 150; % h \in (0, height_lim]

wind_h = @(h) (h>0.04572)*vel_at6m.*log(h/0.04572)/log(6.096/0.04572); % wind shear model
theta_h = theta/180*pi + pi; % forcasted wind field
GetWindProfile = @(h) [wind_h(h)*cos(theta_h);
                       wind_h(h)*sin(theta_h);
                       zeros(1,size(h,2))]; % [wx; wy; 0];

heights = linspace(1e-6, height_lim, wind_pf_size); % start from 0+ avoiding NaN
dh = height_lim/wind_pf_size;
wind_profile_hat = GetWindProfile(heights);

xi = 0.0* [randn();
           randn();
           0]; % wind profile error at 200[m]

% compute delta_w for simulation
delta_ws = zeros(3,wind_pf_size);
a_w = -0.00385;
b_w = 8 * 0.0251;
ratios = [1, 1, 0.5];
for i = 2:wind_pf_size
    delta_ws(:,i) = (1+dh*a_w)*delta_ws(:,i-1)+dh*b_w*[ratios(1)*randn(); ratios(2)*randn(); ratios(3)*randn()];
end
% tune b_w s.t. norm(delta_ws) ~ wind_gust_max - vel_at6m

mean(vecnorm(delta_ws(1:2,:)))
% hold on
% plot(heights, delta_ws(1,:));
% plot(heights, delta_ws(2,:));
% plot(heights, delta_ws(3,:));

wind_err0 = zeros(4,50); % [h, dx, dy, dz] wind error storage

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

%% Safe Zone, Vel Info and Initiation
[Axbxh, init_xy_pos] = SafeZoneCompute(0);
init_pos_in_inertial_frame = [init_xy_pos; -100]; % x-North, z-down, y-East
init_rpy = [0; 0.006; -10/180*pi]; % yaw-pitch-row; from ground to body frame; x-head, z-done, y-right
init_uvw = [3.819; -0.673; 1.62]; % velocity in body frame % shouldn't be all zero
init_pqr = [0; 0; 0]; % angular velocity in body frame

% for guidance, corresponding height, Vh, Vz, without wind, delta_l,r = 0.5
vel_info = [95.551, 3.87807, 1.61823]; % [m, m/s, m/s]
% for control, [Vh0, Vh1, Vz0, Vz1], without wind when delta_s = 0,1
vel_info_mpcc = [4.82; 3.35; 1.515; 1.653]; % [m/s]

psi_dot_m = 0.1906; % maximum turning angular vel without wind [rad/s]
delta_dot_m = 0.5; % [/s]

%% Sensor Model: Accuracy after Primary Sensor Fusion
sensor_freq = 20; % [Hz] sensor data subscriber frequency
pos_accu = 2; % [m]
vel_accu = 0.1; % [m/s]
row_pitch_accu = 0.1; % [degree]
yaw_accu = 0.5; % [degree]
acc_accu = 0.1; % [m/s^2]
angVel_accu = 0.1; % [deg/s]
% should tune white noise power in simulator for accuracy of airspeed, AOA, and AOS

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

%% Guidance
psi_d = pi; % desired landing orientation

guidance_horizon_N = 200;
guidance0 = GuidanceGuess(guidance_horizon_N, init_pos_in_inertial_frame, init_rpy, vel_info);
psi_ddot_m = psi_dot_m*2*delta_dot_m; % [rad/s2]

% pd_controller_freq = 10; % [Hz]

%% MPCC Tracker
time_horizon_N = 50; % 100 is enough
mpcc_freq = 1; % [Hz]
mpcc_Ts = 0.05; % [s]
mpcc_ctrl_freq = 10; % [Hz]
control0 = zeros(3, time_horizon_N); % [h, psi, psi_dot]
warning('off','MATLAB:polyfit:RepeatedPointsOrRescale') % suppress fit warnings in mpcc

%% Motor Model
% delta_s: 2nd order response
wd = pi/1.35; % omega_d = pi/tp
Mp = abs(2.86574-3.35184)/abs(3.35184-4.8196); % Mp = |c_peak - c_inf| / |c_start - c_inf|
zeta = sqrt((log(Mp)/pi)^2/(1+(log(Mp)/pi)^2));
wn = wd / sqrt(1-zeta^2); % omega_n = omega_d / sqrt(1-zeta^2)

% delta_a: 1st order response
Ta = 1/(0.1293/0.2/0.191); % Ta = 1/(k/psi_dot_m), k: response slope

