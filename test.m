sdpvar a x u
Constraints = [a+1 <= x, a+1<=u];
Objective = x^2 + u^2;
P = optimizer(Constraints,Objective,[],a,[x,u])