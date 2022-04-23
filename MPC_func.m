function [flag, xs, us] = MPC_func(N, init_pose, Q, r, ref, Ts, xy_dot, um, dx, dy)

    Prob = casadi.Opti();
    % --- define optimization variables ---
    X = Prob.variable(3, N);
    U = Prob.variable(1, N-1);
    
    % --- calculate objective ---
    Qpos = Q(1:2, 1:2);
    Qpsi = Q(3,3);
    objective = 0;
    for i = 2:N
    objective = objective + (X(1:2,i) - ref(1:2,i))'*Qpos*(X(1:2,i) - ref(1:2,i)) ...
                          + Qpsi*(1-cos(X(3,i) - ref(3,i))) ...
                          + r*U(i-1)^2;   
    end
    Prob.minimize(objective)
    
    % --- define constraints ---
    X_0 = init_pose;
    Prob.subject_to(X(:,1)==X_0);
    for i = 2:N
        Prob.subject_to(U(i-1)<=um);
        Prob.subject_to(U(i-1)>=-um);
        if abs(ref(:,i) - ref(:,i-1)) < 1e-8
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
end