function [flag, xs, us] = MPC_func(N, init_pose, P, Q, r, Qn, ref, Ts, xy_dot, um, dx, dy)

    Prob = casadi.Opti();
    % --- define optimization variables ---
    X = Prob.variable(3, N);
    U = Prob.variable(1, N-1);
    
    % --- calculate objective --- 
    objective = 0;
    
    for i = 1:N-1
        objective = objective + (X(1:2,i) - ref(1:2,i))'* Q(1:2, 1:2) *(X(1:2,i) - ref(1:2,i)) ...
                              + Q(3,3) * (1 - cos(X(3,i) - ref(3,i))) ...
                              + r * U(i)^2;
    end
    objective = objective + (X(1:2,N) - ref(1:2,N))'* P(1:2, 1:2) *(X(1:2,N) - ref(1:2,N)) ...
                          + P(3,3) *(1 - cos(X(3,N) - ref(3,N)));
    
    Prob.minimize(objective)
    
    % --- define constraints ---
    X_0 = init_pose;
    Prob.subject_to(X(:,1)==X_0);
    for i = 1:N-1
        Prob.subject_to(U(i)<=um);
        Prob.subject_to(U(i)>=-um);
        if norm(ref(:,i+1) - ref(:,i)) < 1e-6
            Prob.subject_to(X(:,i+1) == X(:,i));
        else
            Prob.subject_to(X(:,i+1) == X(:,i) + [Ts * xy_dot * cos(X(3,i));
                                                  Ts * xy_dot * sin(X(3,i));
                                                  Ts * U(i)]);
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