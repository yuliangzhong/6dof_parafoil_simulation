x = sdpvar(1);
y = sdpvar(1);

a = sqrt(2);
% Define constraints 
% Contraints = boolean(zeros(1,2));
% Constraints(1,1) = norm([x,y])<=a;
% Constraints(1,2) = norm([x,y])>=0.1;
Constraints = [norm([x,y])<=a];%, norm([x,y])>=0.1];

Objective = x+y;

% Set some options for YALMIP and solver
options = sdpsettings('verbose',0,'solver','ecos');

% Solve the problem
sol = optimize(Constraints,Objective,options);
value(Objective)

% for i = 1:1
%     disp("haha")
% end
