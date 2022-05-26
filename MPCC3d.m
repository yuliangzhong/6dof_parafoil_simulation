function [flag, control] = MPCC3d(N, Ts, vel_info, vel_info_mpcc, psi_dot_m, delta_dot_m, init_cond, init_dsda, guidance, heights, wind_profile_hat, wind_dis)

tic

%% Parameters Definition

h0 = init_cond(3);
Vz = vel_info(3);
um = psi_dot_m;
Au = [1, 0.5;
      1, -0.5;
      -1,-0.5;
      -1, 0.5];
bu = [1; 1; 0; 0];
Av = [1; -1];
bv = [2*Vz; 0];
% As = [1, 0;
%       -1,0;
%       0, 1;
%       0,-1];
As = Au;
bs = 1.05*delta_dot_m*[1; 1; 1; 1];

%% interpolate guidance & wind profile
dh = Ts*(Vz+wind_dis(3));
h_range = 1.5*N*dh;
h_max = h0 + h_range; 
h_min = max(0 - h_range, h0 - 2*h_range);
extrap_heights_num = (h_max - h_min)/dh+1;
interp_heights = linspace(h_max, h_min, extrap_heights_num);
interp_method = 'spline';
if h0 < 2*h_range
    interp_method = 'linear'; % to avoid strange fitting near the ground
end
interp_guidance = [interp1(guidance(3,:), guidance(1,:), interp_heights, interp_method, 'extrap');
                   interp1(guidance(3,:), guidance(2,:), interp_heights, interp_method, 'extrap');
                   interp_heights;
                   interp1(guidance(3,:), guidance(4,:), interp_heights, interp_method, 'extrap');
                   interp1(guidance(3,:), guidance(5,:), interp_heights, interp_method, 'extrap')];

interp_wind_pf = [interp1(heights, wind_profile_hat(1,:), interp_heights, 'linear', 'extrap');
                  interp1(heights, wind_profile_hat(2,:), interp_heights, 'linear', 'extrap')];

[~, id] = min(vecnorm(interp_guidance(1:3,:) - init_cond(1:3)*ones(1,size(interp_guidance,2))));
hr = interp_guidance(3,id);


%% Function Fit

% fx
px = polyfit(interp_guidance(3,:), interp_guidance(1,:), 10);
fx = @(x) px*[x.^10; x.^9; x.^8; x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
dfx = @(x) px*[10*x.^9; 9*x.^8; 8*x.^7; 7*x.^6; 6*x.^5; 5*x.^4; 4*x.^3; 3*x.^2; 2*x; ones(1,size(x,2)); zeros(1,size(x,2))];

% fy
py = polyfit(interp_guidance(3,:), interp_guidance(2,:), 10);
fy = @(x) py*[x.^10; x.^9; x.^8; x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
dfy = @(x) py*[10*x.^9; 9*x.^8; 8*x.^7; 7*x.^6; 6*x.^5; 5*x.^4; 4*x.^3; 3*x.^2; 2*x; ones(1,size(x,2)); zeros(1,size(x,2))];

% wx
pwx = polyfit(interp_heights, interp_wind_pf(1,:), 10);
wx = @(x) pwx*[x.^10; x.^9; x.^8; x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
% wy
pwy = polyfit(interp_heights, interp_wind_pf(2,:), 10);
wy = @(x) pwy*[x.^10; x.^9; x.^8; x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];

% wx, wy are valid only when h>0!!

% assume linear control model
Vh_f = @(x) (vel_info_mpcc(2)-vel_info_mpcc(1))*x + vel_info_mpcc(1);
Vz_f = @(x) (vel_info_mpcc(4)-vel_info_mpcc(3))*x + vel_info_mpcc(3);
dpsi_f = @(x) um*x;

%% MPC Formulation

Q = diag([1000, 100, 10]);
R = diag([10, 10, 10]);
q_eta = 50;

Prob = casadi.Opti();

% --- define optimization variables ---
X = Prob.variable(5, N); % [x, y, h, psi, eta]
U = Prob.variable(2, N); % [ds, da]
V = Prob.variable(1, N); % eta

% % --- set initial guess from control --- 
% x_guess = [init_cond; hr]*ones(1,N);
% u_guess = [0.5*ones(1,N);
%            interp_guidance(5,id:id+N-1)/um]; % ds da
% v_guess = (Vz+wind_dis(3))*ones(1,N); % 1*N
% 
% for i = 1:N-1
%     if i > ceil(h0/Ts/(Vz+wind_dis(3))) % if landed at ground, no wind disturbance
%         x_guess(:,i+1) = x_guess(:,i) + Ts* [Vh_f(u_guess(1,i)) * cos(x_guess(4,i));
%                                              Vh_f(u_guess(1,i)) * sin(x_guess(4,i));
%                                              - (Vz_f(u_guess(1,i)) + wind_dis(3));
%                                              dpsi_f(u_guess(2,i));
%                                              -v_guess(i)];
%     else
%         x_guess(:,i+1) = x_guess(:,i) + Ts* [Vh_f(u_guess(1,i)) * cos(x_guess(4,i)) + wx(x_guess(3,i)) + wind_dis(1);
%                                              Vh_f(u_guess(1,i)) * sin(x_guess(4,i)) + wy(x_guess(3,i)) + wind_dis(2);
%                                              - (Vz_f(u_guess(1,i)) + wind_dis(3));
%                                              dpsi_f(u_guess(2,i));
%                                              -v_guess(i)];        
%     end
% end
% 
% % Prob.set_initial(X, x_guess);
% Prob.set_initial(U, u_guess);
% Prob.set_initial(V, v_guess);

% --- calculate objective --- 
objective = 0;

for i = 2:N
    gamma = atan2(-dfy(X(5,i)), -dfx(X(5,i))); % since eta is decreasing!
    es_l = -cos(gamma)*(X(1,i) - fx(X(5,i))) - sin(gamma)*(X(2,i) - fy(X(5,i)));
    es_c = sin(gamma)*(X(1,i) - fx(X(5,i))) - cos(gamma)*(X(2,i) - fy(X(5,i)));
    es_h = X(3,i) - X(5,i);

    objective = objective + [es_c; es_l; es_h]'* Q * [es_c; es_l; es_h] ...
                          + q_eta * X(5,i);
end
Prob.minimize(objective)

% --- define constraints ---
Prob.subject_to(X(:,1) == [init_cond; hr]);
Prob.subject_to(U(:,1) == init_dsda);
Prob.subject_to(Au*U(:,N)<=bu);
Prob.subject_to(Av*V(N)<=bv);

for i = 1:N-1
    Prob.subject_to(Au*U(:,i)<=bu);
    Prob.subject_to(Av*V(i)<=bv);
    Prob.subject_to(As*((U(:,i+1)-U(:,i))/Ts)<=bs);

    if i > ceil(h0/Ts/(Vz+wind_dis(3))) % if landed at ground, no wind disturbance
        Prob.subject_to(X(:,i+1) == X(:,i) + Ts* [Vh_f(U(1,i)) * cos(X(4,i));
                                                  Vh_f(U(1,i)) * sin(X(4,i));
                                                  - (Vz_f(U(1,i)) + wind_dis(3));
                                                  dpsi_f(U(2,i));
                                                  -V(i)]);
    else
        Prob.subject_to(X(:,i+1) == X(:,i) + Ts* [Vh_f(U(1,i)) * cos(X(4,i)) + wx(X(3,i)) + wind_dis(1);
                                                  Vh_f(U(1,i)) * sin(X(4,i)) + wy(X(3,i)) + wind_dis(2);
                                                  - (Vz_f(U(1,i)) + wind_dis(3));
                                                  dpsi_f(U(2,i));
                                                  -V(i)]);
    end
end

% --- define solver ---
Prob.solver('ipopt', struct('print_time', 0), struct('print_level', 0));

% --- output ---
sol = Prob.solve();

xs = sol.value(X);
us = sol.value(U);
control = [xs(3,1:N);
           us(1,1:N) - us(2,1:N)/2;
           us(1,1:N) + us(2,1:N)/2]; % [h, dl, dr]

hold on
grid on
scatter3(fy(xs(5,:)), fx(xs(5,:)), xs(5,:), 5, 'g')
plot3(xs(2,:), xs(1,:), xs(3,:), 'b--','LineWidth',1)
scatter3(init_cond(2), init_cond(1), init_cond(3),'b', 'filled')

% scatter3(x_guess(2,:),x_guess(1,:),x_guess(3,:),5,'k')
% scatter3(interp_guidance(2,id), interp_guidance(1,id), interp_guidance(3,id), 'r', 'filled')
% scatter3(interp_guidance(2,:), interp_guidance(1,:), interp_guidance(3,:), 'm')
% axis equal
flag = true;


% try
%     sol = Prob.solve();
% 
%     xs = sol.value(X);
%     us = sol.value(U);
%     control = [xs(3,1:N);
%                us(1,1:N) - us(2,1:N)/2;
%                us(1,1:N) + us(2,1:N)/2]; % [h, dl, dr]
% 
%     hold on
%     grid on
%     scatter3(fy(xs(5,:)), fx(xs(5,:)), xs(5,:), 5, 'g')
%     plot3(xs(2,:), xs(1,:), xs(3,:), 'b--','LineWidth',1)
% 
%     flag = true;
% 
% catch
%     disp("WARNING! MPC Solution not found!")
%     flag = false;
%     control = zeros(3,N);
% end

%%
toc
text(init_cond(2)+10, init_cond(1), num2str(toc))
end
