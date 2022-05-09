%% Environment
gravity_acc = + 9.81; % z-down

%% Wind Profile, Pos Compensation and Wind Gust Dynamics
vel_at6m = 1.5; % wind velocity at 6m, absolute value
theta = 120; % [deg] constant

wind_h = @(h) (h>0.04572)*vel_at6m.*log(h/0.04572)/log(6.096/0.04572); % wind shear model
theta_h = @(h) theta/180*pi + pi; % forcasted wind field
GetWindProfile = @(h) [wind_h(h).*cos(theta_h(h));
                       wind_h(h).*sin(theta_h(h));
                       zeros(1,size(h,2))]; % [wx; wy; 0];
wind_pf_size = 7000;
heights = linspace(1e-6, 350, wind_pf_size); % start from 0+ avoiding NaN
wind_profile_hat = GetWindProfile(heights);

Delta_s = zeros(3,wind_pf_size);
for i = 2:wind_pf_size
    Delta_s(:,i) = [trapz(heights(1:i), wind_profile_hat(1, 1:i));
                    trapz(heights(1:i), wind_profile_hat(2, 1:i));
                    trapz(heights(1:i), wind_profile_hat(3, 1:i))];
end % Remember /Vz!!

xi = 0.0* [randn();
           randn();
           0]; % wind profile error at 200[m]

a_w = -0.0385;
diag_sigma_zeta = [0.0251, 0.0251, 0.0251]; % diagonal matrix
% set delta_w sampling time = 0.2s

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

S_pd = 0.06; % payload reference area

C_BP = eye(3); % from body frame B to parafoil aero frame P;
B_r_BP = [0; 0; -1.0];

C_BW = eye(3); % from body frame B to payload frame W;
B_r_BW = [0; 0; 0.1];

[Im, Ii] = ImIiCompute(parafoil_arc_h, parafoil_ref_span, ...
                       parafoil_ref_length, parafoil_thickness);

%% Aero Coefficients
%--- from paper 12 and book ch5
c_L0 = 0.24; % lift force coefficient
c_La = 2.14;
c_Lds = 0.1670;

c_D0 = 0.12;
c_Da2 = 0.33;
c_Dds = 0.043;

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

%% Initiation
init_pos_in_inertial_frame = [200, -200, -300]; % x-North, z-down, y-East
init_rpy = [0,      0.02,  pi*2/4]; % yaw-pitch-row; from ground to body frame; x-head, z-done, y-right
init_uvw = [4.56,   0,  1.49]; % velocity in body frame
init_pqr = [0,  0,  0]; % angular velocity in body frame

%% Observer accuracy(accu)
sampling_T = 0.15;
row_pitch_accu = 0.1; % [degree]
yaw_accu = 0.5; % [degree]
pos_accu = 0.5; % [m]
vel_accu = 0.05; % [m/s]
acc_accu = 0.5; % [m/s^2]
angVel_accu = 0.02; %[deg/s]
airspeed_var = [0.001, 0.001, 0.005]; % [alpha, beta, Vb], no unit
                                      % about [2.3deg, 2.3deg, 0.05m/s]

%% Wind Estimator
% period  = sampling_T!!! [s]
mu0 = GetWindProfile(-init_pos_in_inertial_frame(3));
sigma0 = eye(3); % wind variance initial guess
w_bar_hat0 = mu0;
wind_est_dyn_var = 1.01 * sampling_T^2 * diag(diag_sigma_zeta); % v ~ N(0, Q), Q matrix
wind_est_noise_var = 3*vel_accu*eye(3); % d ~ N(0, R), R matrix, sensor noise, needs tuning
wind_err0 = zeros(4,15);

%% Guidance
guidance_horizon_N = 2000;
guidance0 = zeros(5, guidance_horizon_N); % [x, y, h, psi, psi_dot]
Vz = 1.39; % descending rate without wind [m/s]
Vh = 4.59; % horizontal vel without wind [m/s]
psi_dot_m = 0.2187; % maximum turning angular vel without wind [rad/s]
psi_d = pi;
guidance_info0 = [-init_pos_in_inertial_frame(3); Vz/Vh]; % [h, kappa]
pd_ctrler_T = 0.1; % [s]

%% MPCC Tracker
time_horizon_N = 100; % should not exceed 1000
mpc_samping_T = 0.1; % [s]
control0 = zeros(3, time_horizon_N); % [h, psi, psi_dot]

%% Aerodynamic Coefficients Estimator
aeroF_co_mu0 = [0 0 0; 
                0 0 0; 
                0 0 0;
                3 3 3]; % [cD; cYb; cL; cDpd], delta_s = 0; 0.5; 1;
aeroF_co_sigma0 = 0.01*[diag([1,1,1,1]), diag([1,1,1,1]), diag([1,1,1,1])];
aeroF_est_noise_var = 0.0005*eye(3);

warning('off','MATLAB:polyfit:RepeatedPointsOrRescale') % mpcc