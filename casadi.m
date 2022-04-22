prob = casadi.Opti();
x = prob.variable();
y = prob.variable();

prob.minimize((1-x)^2 + (y-x^2)^2);

prob.subject_to(x^2 + y^2 == 1);
prob.subject_to(y>=x);

prob.solver('ipopt');
sol = prob.solve();