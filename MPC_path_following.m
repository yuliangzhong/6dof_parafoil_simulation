% warning('off','MATLAB:polyfit:RepeatedPointsOrRescale')
% figure;
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
disp([x0, y0, h0, psi0])

[~, id] = min(vecnorm(guidance_in_wind(1:2,:) - [x0; y0]*ones(1,2000)));
hd = guidance_in_wind(3,id);
Dh = h0 - hd;
hold on
disp(guidance_in_wind(:,id)') 
scatter(guidance_in_wind(2,id), guidance_in_wind(1,id),'k')

% fitting --> fx, fy, fpsi
N = 100;
ids = max(id - 50, 1);
idn = min(id+2*N, 2000);

px = polyfit(guidance_in_wind(3,ids:idn), guidance_in_wind(1,ids:idn), 7);
fx = @(x) px*[x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
py = polyfit(guidance_in_wind(3,ids:idn), guidance_in_wind(2,ids:idn), 7);
fy = @(x) py*[x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
ppsi = polyfit(guidance_in_wind(3,ids:idn), guidance_in_wind(4,ids:idn), 7);
fpsi = @(x) ppsi*[x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];

% MPC formulation
Ts = 0.1;

% P = diag([100,100,10000]);
Q = diag([1000, 1000]);
R = diag([1000, 1000]);
q_theta = 100;

Prob = casadi.Opti();
% --- define optimization variables ---
X = Prob.variable(4, N+1); % [x, y, psi, theta]
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
    xs = zeros(4,N+1);
    us = zeros(2,N);
end

% plot
hold on
grid on
plot(guidance_in_wind(2,ids:idn),guidance_in_wind(1,ids:idn), 'g')

% states = [x0; y0; psi0; hd] * ones(1,N+1);
% for i = 1:N
%     states(:,i+1) = states(:,i) + Ts* [xy_dot * cos(xs(3,i));
%                                        xy_dot * sin(xs(3,i));
%                                        us(1,i);
%                                        -us(2,i)];
% end
% 
scatter(fy(xs(4,:)), fx(xs(4,:)), 'r')
plot(xs(2,:),xs(1,:), 'b')
axis equal
% hold off