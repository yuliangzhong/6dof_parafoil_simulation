function [flag, xs, us] = MPC_2d_path_following_func(x0, y0, h0, psi0, guidance_in_wind)

% Params
psi_dot_max = 0.2187;
xy_dot = 4.59;
z_dot = 1.39;
um = psi_dot_max;

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


P = diag([100,100,10000]);
Q = diag([0, 0, 10000]);
R = [1000, 1000000];

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
    xs = zeros(4,N+1);
    us = zeros(2,N);
end

end
