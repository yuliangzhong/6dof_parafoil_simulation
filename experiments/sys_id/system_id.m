% load data file
filename = 'system_id_data.xlsx';

opts = detectImportOptions(filename);

% load outputs
opts.SelectedVariableNames = [2:3]; 
opts.DataRange = '2:23';
t_y_raw = readmatrix(filename,opts);

% load inputs
opts.SelectedVariableNames = [7,11]; 
opts.DataRange = '2:497';
t_u_filter = [0,0;readmatrix(filename,opts)];

opts.SelectedVariableNames = [7,10]; 
opts.DataRange = '2:497';
t_u_raw = [0,0;readmatrix(filename,opts)];

% interpolate dataset
N = 100;
t_span = linspace(0, 10, N);
Ts = (t_span(end)-t_span(1))/(N-1); % s

t_u = zeros(N,1);
t_y = zeros(N,1);
for i=1:N
    t_u(i) = interp1(t_u_filter(:,1)', t_u_filter(:,2)', t_span(i), 'linear', 'extrap');
    t_y(i) = interp1(t_y_raw(:,1)', t_y_raw(:,2)', t_span(i), 'spline', 'extrap');
end

% viz and check data
subplot(1,2,1)
hold on
grid on
plot(t_span, t_u,'b', 'linewidth',1.5)
scatter(t_u_raw(:,1), t_u_raw(:,2),25,'k','filled')
title('Control input \delta_{a}(t)')
xlabel('time [s]')
ylabel('asymmetric brake deflection \delta_a \in [-1,1]')
legend('filtered control input signal','raw control data points')
set(gca,'FontSize',20);

subplot(1,2,2)
hold on
plot(t_span, t_y,'b','linewidth',1.5)
scatter(t_y_raw(:,1), t_y_raw(:,2),25,'k','filled')

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

sys = tf(b,[1, a, 0]);
y_fit = lsim(sys,t_u',t_span);
plot(t_span, y_fit, 'r', 'linewidth',1.5)
title('Body heading output \psi(t)')
xlabel('time [s]')
ylabel('system body heading \psi [rad]')
legend('body heading output signal','raw heading data points','simulated response of identified model')
grid on
set(gca,'FontSize',20);

%% plot trajectory

opts.SelectedVariableNames = [14,15]; 
opts.DataRange = '2:27';
gps = readmatrix(filename,opts);

figure (2)
hold on
grid on
axis equal
ylim([-inf,65])

scatter(gps(:,2), gps(:,1),50, 'r','filled');

set(gca,'FontSize',20)
xlabel('\rightarrow East [m]')
ylabel('\rightarrow North [m]')
title('Real flight test trajectory in xy-plane')
legend('GNSS readings', 'Location', 'best')