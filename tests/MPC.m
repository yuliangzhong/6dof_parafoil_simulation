% TODO
psi_dot_max = 0.2187;
xy_dot = 4.59;
z_dot = 1.39;
um = psi_dot_max / z_dot;

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

ref = zeros(2,N);
for i = 2:N
    if id+i-2>2000
        ref(:,i) = ref(:,i-1);
    else
        ref(:,i) = ref(:,i-1) + [Ts * xy_dot * cos(guidance(4,id+i-2));
                                 Ts * xy_dot * sin(guidance(4,id+i-2))];
    end
end


Prob = casadi.Opti();
% --- define optimization variables ---
X = Prob.variable(3, N);
U = Prob.variable(1, N-1);

% --- calculate trajectory and objective ---
objective = 0;
X_0 = init_pose;
Q = diag([100,100]);
r = 1000;

for i = 2:N
    if ref(:,i) == ref(:,i-1)
        X(:,i) = X(:,i-1);
    else
        X(:,i) = X(:,i-1) + [Ts * (xy_dot * cos(X(3,i-1)) + dx);
                             Ts * (xy_dot * sin(X(3,i-1)) + dy);
                             Ts * U(i-1)];
    end
    objective = objective + (X(1:2,i) - ref(:,i))'*Q*(X(1:2,i) - ref(:,i)) + r*U(i-1)^2;
end

Prob.minimize(objective)

% --- define constraints ---
Prob.subject_to(X(:,1)==X_0);
for i = 1:N-1
    Prob.subject_to(U(i)<=um);
    Prob.subject_to(U(i)>=-um);
end
Prob.solver('ipopt', struct('print_time', 0), struct('print_level', 0));
sol = Prob.solve();

if sol.stats.success
    us = sol.value(U);
else
    disp("MPC Solution not found")
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
