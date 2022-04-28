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

h0 = -I_r_IB(3);
x0 = I_r_IB(1) + (1/z_dot)*interp1(heights, Delta_s(1,:), h0,'linear','extrap');
y0 = I_r_IB(2) + (1/z_dot)*interp1(heights, Delta_s(2,:), h0,'linear','extrap');
C_IB = C_BI';
psi0 = atan2(C_IB(2,1), C_IB(1,1));

[~, id] = min(abs(guidance_in_wind(3,:) - h0*ones(1,2000)));
xd = guidance_in_wind(1,id);
yd = guidance_in_wind(2,id);
hd = guidance_in_wind(3,id);
psid = guidance_in_wind(4,id);

disp([x0, y0, h0, psi0])
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

% MPC formulation
Ts = 0.1;

P = diag([10000,10000,10000, 10000]);
Q = diag([100, 100, 1000, 10000]);
R = [1000, 100000];

Prob = casadi.Opti();
% --- define optimization variables ---
X = Prob.variable(5, N+1); % [x, y, h, psi, theta]
U = Prob.variable(2, N);

% --- calculate objective --- 
objective = 0;

for i = 1:N
    objective = objective + (X(1:3,i) - [fx(X(5,i)); fy(X(5,i)); X(5,i)])'* Q(1:3, 1:3) *(X(1:3,i) - [fx(X(5,i)); fy(X(5,i)); X(5,i)]) ...
                          + Q(4,4) * (1 - cos(X(4,i) - fpsi(X(5,i)))) ...
                          + R(1) * U(1,i)^2 ...
                          + R(2) * (U(2,i) - z_dot)^2;
end
objective = objective + (X(1:3,N+1) - [fx(X(5,N+1)); fy(X(5,N+1)); X(5,N+1)])'* P(1:3, 1:3) *(X(1:3,N+1) - [fx(X(5,N+1)); fy(X(5,N+1)); X(5,N+1)]) ...
                      + P(4,4) *(1 - cos(X(4,N+1) - fpsi(X(5,N+1))));

Prob.minimize(objective)

% --- define constraints ---
Prob.subject_to(X(:,1)==[x0; y0; h0; psi0; hd]);
for i = 1:N
    Prob.subject_to(U(1,i)<=um);
    Prob.subject_to(U(1,i)>=-um);
    Prob.subject_to(U(2,i)>=0);
    Prob.subject_to(U(2,i)<=2*z_dot);
    Prob.subject_to(X(:,i+1) == X(:,i) + Ts* [xy_dot * cos(X(4,i));
                                              xy_dot * sin(X(4,i));
                                              -z_dot;
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
plot3(guidance_in_wind(2,id:idn),guidance_in_wind(1,id:idn), guidance_in_wind(3,id:idn), 'r')
hold on;
grid on;
plot3(xs(2,:),xs(1,:),xs(3,:), 'b')

hold off
grid off
