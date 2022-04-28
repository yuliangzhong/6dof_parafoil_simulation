% Params
psi_dot_max = 0.2187;
xy_dot = 4.59;
z_dot = 1.39;
um = psi_dot_max;

% get current state
guidance_in_wind = out.guidance_in_wind.signals.values(:,:,end);
wind_err = out.wind_err.signals.values(:,:,end);
C_BI = out.C_BI.signals.values(:,:,end);
I_r_IB = out.I_r_IB.signals.values(end,:);

h = -I_r_IB(3);
x0 = I_r_IB(1) + (1/z_dot)*interp1(heights, Delta_s(1,:), h,'linear','extrap');
y0 = I_r_IB(2) + (1/z_dot)*interp1(heights, Delta_s(2,:), h,'linear','extrap');
C_IB = C_BI';
psi0 = atan2(C_IB(2,1), C_IB(1,1));

[~, id] = min(abs(guidance_in_wind(3,:) - h*ones(1,2000)));
xd = guidance_in_wind(1,id);
yd = guidance_in_wind(2,id);
hd = guidance_in_wind(3,id);
psid = guidance_in_wind(4,id);

disp([x0, y0, h, psi0])
disp([xd, yd, hd, psid])

% fitting --> fx, fy, fpsi
N = 100;
idn = min(id+2*N, 2000);

% thetas = guidance_in_wind(3,id:idn);
% x_s = guidance_in_wind(1,id:idn);
% y_s = guidance_in_wind(2,id:idn);
% psi_s = guidance_in_wind(4,id:idn);

px = polyfit(guidance_in_wind(3,id:idn), guidance_in_wind(1,id:idn), 7);
fx = @(x) px*[x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
py = polyfit(guidance_in_wind(3,id:idn), guidance_in_wind(2,id:idn), 7);
fy = @(x) py*[x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
ppsi = polyfit(guidance_in_wind(3,id:idn), guidance_in_wind(4,id:idn), 7);
fpsi = @(x) ppsi*[x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];

% y1 = fx(thetas);
% y2 = fy(thetas);
% y3 = fpsi(thetas);
% 
% plot(thetas,x_s)
% hold on
% plot(thetas,y_s)
% plot(thetas, psi_s)
% plot(thetas,y1)
% plot(thetas,y2)
% plot(thetas,y3)
% hold off

% MPC formulation
Ts = 0.1;


P = diag([10000,10000,10000]);
Q = diag([100, 100, 10000]);
R = [1000, 100000];

Prob = casadi.Opti();
% --- define optimization variables ---
X = Prob.variable(4, N+1);
U = Prob.variable(2, N);

% --- calculate objective --- 
objective = 0;

for i = 1:N
    objective = objective + (X(1:2,i) - [fx(X(4,i)); fy(X(4,i))])'* Q(1:2, 1:2) *(X(1:2,i) - [fx(X(4,i)); fy(X(4,i))]) ...
                          + Q(3,3) * (1 - cos(X(3,i) - fpsi(X(4,i)))) ...
                          + R(1) * U(1,i)^2 ...
                          + R(2) * (U(2,i) - z_dot)^2;
end
objective = objective + (X(1:2,N+1) - [fx(X(4,N+1)); fy(X(4,N+1))])'* P(1:2, 1:2) *(X(1:2,N+1) - [fx(X(4,N+1)); fy(X(4,N+1))]) ...
                      + P(3,3) *(1 - cos(X(3,N+1) - fpsi(X(4,N+1))));

Prob.minimize(objective)

% --- define constraints ---
Prob.subject_to(X(:,1)==[x0; y0; psi0; hd]);
for i = 1:N
    Prob.subject_to(U(1,i)<=um);
    Prob.subject_to(U(1,i)>=-um);
    Prob.subject_to(U(2,i)>=0);
    Prob.subject_to(U(2,i)<=2*z_dot);
    Prob.subject_to(X(:,i+1) == X(:,i) + Ts* [xy_dot * cos(X(3,i));
                                              xy_dot * sin(X(3,i));
                                              U(1,i);
                                              -U(2,i)]);
end

% --- define solver ---
Prob.solver('ipopt', struct('print_time', 0), struct('print_level', 0));
sol = Prob.solve();

% --- output ---
flag = sol.stats.success;
if flag
    xs = sol.value(X);
    us = sol.value(U);
else
    disp("MPC Solution not found")
    xs = zeros(3,N);
    us = zeros(1,N-1);
end
% 
% % plot
grid on;
plot(guidance_in_wind(2,id:idn),guidance_in_wind(1,id:idn), 'b')
hold on;
states = [x0; y0; psi0; hd] * ones(1,N+1);
for i = 1:N
    states(:,i+1) = states(:,i) + Ts* [xy_dot * cos(xs(3,i));
                                       xy_dot * sin(xs(3,i));
                                       us(1,i);
                                       -us(2,i)];
end

plot(states(2,:),states(1,:), 'r')
hold off
grid off
