% load data file
opts = detectImportOptions('system_id_data.xlsx');

% load outputs
opts.SelectedVariableNames = [2:3]; 
opts.DataRange = '2:23';
t_y_raw = readmatrix('system_id_data.xlsx',opts);

% load inputs
opts.SelectedVariableNames = [7,11]; 
opts.DataRange = '2:497';
% t_u_raw = readmatrix('system_id_data.xlsx',opts);
t_u_raw = [0,0;readmatrix('system_id_data.xlsx',opts)];

% interpolate dataset
N = 100;
t_span = linspace(0, 10, N);
Ts = (t_span(end)-t_span(1))/(N-1); % s

t_u = zeros(N,1);
t_y = zeros(N,1);
for i=1:N
    t_u(i) = interp1(t_u_raw(:,1)', t_u_raw(:,2)', t_span(i), 'linear', 'extrap');
    t_y(i) = interp1(t_y_raw(:,1)', t_y_raw(:,2)', t_span(i), 'spline', 'extrap');
end

% viz and check data
subplot(1,2,1)
hold on
grid on
plot(t_span, t_u)
scatter(t_u_raw(:,1), t_u_raw(:,2))

subplot(1,2,2)
hold on
plot(t_span, t_y,'r')
scatter(t_y_raw(:,1), t_y_raw(:,2))

% pack data and estimate
sys_id_data = iddata([zeros(10,1);t_y], [zeros(10,1);t_u], Ts);
init_sys = idtf(1,[1 1 0]);
init_sys.Structure.Denominator.Free(end) = false;
opt = tfestOptions('SearchMethod','lm', 'InitialCondition','zero');
est_tf = tfest(sys_id_data,init_sys,opt)

% verification
[pvec_step, dpvec_step] = getpvec(est_tf, 'free');
b = pvec_step(1);
a = pvec_step(2);
deley = 0;

sys = tf(b,[1, a, 0], 'InputDelay', deley)
lsim(sys,t_u',t_span)
grid on