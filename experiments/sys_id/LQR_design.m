% system model
% discretize
dT = 0.5; % [seconds]
Ac = [0, 1; 0, -a];
Bc = [0; b];
Cc = [1 0];
Dc = 0;
sys_space = ss(Ac, Bc, Cc, Dc);
dsys = c2d(sys_space, dT)

% x+ = Ax + Bu
Q = eye(2);
R = 1;
K = dlqr(dsys.A, dsys.B, Q, R);

% closed-loop model
% x+ = (A-BK)x
[V, E] = eig(dsys.A - dsys.B*K);
