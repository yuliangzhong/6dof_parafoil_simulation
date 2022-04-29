
% warning('off','MATLAB:polyfit:RepeatedPointsOrRescale')

% Params
psi_dot_max = 0.2187;
xy_dot = 4.59;
z_dot = 1.39;
um = psi_dot_max;
theta = 120/180*pi + pi;
vel_at6m = 1.5;

% get current state
guidance = out.guidance.signals.values(:,:,end);
wind_err = out.wind_err.signals.values(:,:,end);
C_BI = out.C_BI.signals.values(:,:,end);
I_r_IB = out.I_r_IB.signals.values(end,:);
h0 = -I_r_IB(3);
x0 = I_r_IB(1);
y0 = I_r_IB(2);
C_IB = C_BI';
psi0 = atan2(C_IB(2,1), C_IB(1,1));
disp([x0, y0, h0, psi0])


[~, id] = min(vecnorm(guidance(1:2,:) - [x0; y0]*ones(1,2000)));
hd = guidance(3,id);
Dh = h0 - hd;
disp(guidance(:,id)')

% fitting --> fx, fy, fpsi
N = 100;
ids = max(id - 50, 1);
idn = min(id+2*N, 2000);

px = polyfit(guidance(3,ids:idn), guidance(1,ids:idn), 7);
fx = @(x) px*[x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
py = polyfit(guidance(3,ids:idn), guidance(2,ids:idn), 7);
fy = @(x) py*[x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
ppsi = polyfit(guidance(3,ids:idn), guidance(4,ids:idn), 7);
fpsi = @(x) ppsi*[x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];

pwx = polyfit(heights, wind_profile_hat(1,:), 7);
wx = @(x) pwx*[x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
pwy = polyfit(heights, wind_profile_hat(2,:), 7);
wy = @(x) pwy*[x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];

% MPC formulation
Ts = 0.1;

% P = diag([100,100,10000]);
Q = diag([1000, 1000]);
R = diag([1000, 1000]);
q_theta = 100;

Prob = casadi.Opti();
% --- define optimization variables ---
X = Prob.variable(5, N+1); % [x, y, psi, theta, h]
U = Prob.variable(2, N);   % [u, v]

% --- calculate objective --- 
objective = 0;

for i = 2:N
    psi = fpsi(X(4,i));
    es_l = -cos(psi)*(X(1,i) - fx(X(4,i))) - sin(psi)*(X(2,i) - fy(X(4,i)));
    es_c = sin(psi)*(X(1,i) - fx(X(4,i))) - cos(psi)*(X(2,i) - fy(X(4,i)));
    du = U(1,i) - U(1,i-1);
    dv = U(2,i) - U(2,i-1);
    objective = objective + [es_c, es_l]* Q * [es_c, es_l]' ...
                          + q_theta * X(4,i) ...
                          + [du, dv]* R *[du, dv]';
end
psi = fpsi(X(4,N+1));
es_l = -cos(psi)*(X(1,N+1) - fx(X(4,N+1))) - sin(psi)*(X(2,N+1) - fy(X(4,N+1)));
es_c = sin(psi)*(X(1,N+1) - fx(X(4,N+1))) - cos(psi)*(X(2,N+1) - fy(X(4,N+1)));
objective = objective + [es_c, es_l]* Q * [es_c, es_l]';
Prob.minimize(objective)

% --- define constraints ---
% wind_h = @(h) (h>0.04572)*vel_at6m.*log(h/0.04572)/log(6.096/0.04572);
wind_h = @(h) log(-1000+h/0.04572);
Prob.subject_to(X(:,1)==[x0; y0; psi0; hd; h0]);
for i = 1:N
    Prob.subject_to(U(1,i)<=um);
    Prob.subject_to(U(1,i)>=-um);
    Prob.subject_to(U(2,i)>=0);
    Prob.subject_to(U(2,i)<=2*z_dot);
    Prob.subject_to(X(:,i+1) == X(:,i) + Ts* [xy_dot * cos(X(3,i)) + wind_h(X(5,i));
                                              xy_dot * sin(X(3,i)) + wind_h(X(5,i));
                                              U(1,i);
                                              -U(2,i);
                                              -z_dot]);
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
    xs = zeros(5,N+1);
    us = zeros(2,N);
end

% % plot
% hold on
% grid on
% plot(guidance(2,ids:idn),guidance(1,ids:idn), 'g')
% 
% % states = [x0; y0; psi0; hd] * ones(1,N+1);
% % for i = 1:N
% %     states(:,i+1) = states(:,i) + Ts* [xy_dot * cos(xs(3,i));
% %                                        xy_dot * sin(xs(3,i));
% %                                        us(1,i);
% %                                        -us(2,i)];
% % end
% % 
scatter(fy(xs(4,:)), fx(xs(4,:)), 'r')
plot(xs(2,:),xs(1,:), 'b')
% axis equal
% % hold off


