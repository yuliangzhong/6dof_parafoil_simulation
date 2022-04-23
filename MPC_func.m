function [flag, us] = MPC_func(N, init_pose, Q, r, ref, Ts, xy_dot, um, dx, dy)
  
    Prob = casadi.Opti();
    % --- define optimization variables ---
    X = Prob.variable(3, N);
    U = Prob.variable(1, N-1);
    
    % --- calculate trajectory and objective ---
    objective = 0;
    X_0 = init_pose;
    
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
    flag = sol.stats.success;
    if flag
        us = sol.value(U);
    else
        disp("MPC Solution not found")
        us = zeros(1,N-1);
    end
end