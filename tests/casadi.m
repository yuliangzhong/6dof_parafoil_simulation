prob = casadi.Opti();
x = prob.variable();
y = prob.variable();

prob.minimize((1-x)^2 + (y-x^2)^2);

prob.subject_to(x^2 + y^2 == 1);
prob.subject_to(y>=x);

prob.solver('ipopt', struct('print_time', 0), struct('print_level', 0));
sol = prob.solve();

ans_x = sol.value(x);
ans_y = sol.value(y);
disp("x = "+num2str(ans_x)+", y = "+num2str(ans_y))