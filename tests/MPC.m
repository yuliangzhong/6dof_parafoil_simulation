% TODO
psi_dot_max = 0.2187;
xy_dot = 4.59;
z_dot = 1.39;
um = psi_dot_max / z_dot;

% index = 2252;
guidance = out.guidance.signals.values(:,:,end);
wind_err = out.wind_err.signals.values(:,:,end);
C_BI = out.C_BI.signals.values(:,:,end);
I_r_IB = out.I_r_IB.signals.values(:,:,end);

xc = I_r_IB(1);
yc = I_r_IB(2);
h = -I_r_IB(3);
C_IB = C_BI';
psi_c = atan2(C_IB(2,1), C_IB(1,1));

[~, id] = min(abs(guidance(3,:) - h*ones(1,2000)));
x0 = guidance(1,id);
y0 = guidance(2,id);
dx = mean(wind_err(2,:));
dy = mean(wind_err(3,:));
dh = (guidance(3,1) - guidance(3,end))/(2000 - 1);
Ts = dh / z_dot;

init_pose = [xc - x0; yc - y0; psi_c];
N = 50;

% Compute ref trajectory
ref = zeros(3,N);
ref(3,1) = guidance(4,id);
for i = 2:N
    if id+i-1>2000
        ref(:,i) = ref(:,i-1);
    else
        ref(:,i) =  [ref(1,i-1) + Ts * xy_dot * cos(guidance(4,id+i-2));
                     ref(2,i-1) + Ts * xy_dot * sin(guidance(4,id+i-2))
                     guidance(4,id+i-1)];
    end
end

Q = diag([100, 100, 10000]);
Qn = diag([1000, 1000, 100]);
r = 1000;

Prob = casadi.Opti();
% --- define optimization variables ---
X = Prob.variable(3, N);
U = Prob.variable(1, N-1);

% --- calculate objective ---
Qpos = Q(1:2, 1:2);
Qpsi = Q(3,3);
Qnpos = Qn(1:2, 1:2);
Qnpsi = Qn(3,3);

objective = 0;

for i = 2:N
    if norm(ref(:,i) - ref(:,i-1)) < 1e-6
        objective = objective + (X(1:2,i) - ref(1:2,i))'* Qnpos *(X(1:2,i) - ref(1:2,i)) ...
                              + Qnpsi *(1 - cos(X(3,i) - ref(3,i))) ...
                              + r * U(i-1)^2;
    else
        objective = objective + (X(1:2,i) - ref(1:2,i))'* Qpos *(X(1:2,i) - ref(1:2,i)) ...
                              + Qpsi * (1 - cos(X(3,i) - ref(3,i))) ...
                              + r * U(i-1)^2;
    end
end
Prob.minimize(objective)

% --- define constraints ---
X_0 = init_pose;
Prob.subject_to(X(:,1)==X_0);
for i = 2:N
    Prob.subject_to(U(i-1)<=um);
    Prob.subject_to(U(i-1)>=-um);
    if norm(ref(:,i) - ref(:,i-1)) < 1e-6
        Prob.subject_to(X(:,i) == X(:,i-1));
    else
        Prob.subject_to(X(:,i) == X(:,i-1) + [Ts * (xy_dot * cos(X(3,i-1)) + dx);
                                              Ts * (xy_dot * sin(X(3,i-1)) + dy);
                                              Ts * U(i-1)]);
    end
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

% plot
hold on;
plot(ref(2,:),ref(1,:))

pos = init_pose * ones(1,N);
for i = 2:N
    if ref(:,i) == ref(:,i-1)
        pos(:,i) = pos(:,i-1);
    else
        pos(:,i) = pos(:,i-1) + [Ts * (xy_dot * cos(pos(3,i-1)) + dx);
                                 Ts * (xy_dot * sin(pos(3,i-1)) + dy);
                                 Ts * us(i-1)];
    end
end

plot(pos(2,:),pos(1,:))
