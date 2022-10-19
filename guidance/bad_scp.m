% Install 'ecos' and 'Yalmip' first!

%% Parameters
lambda1 = 100;
lambda2 = 10;
psi_d = 5*pi/6;
v_ha = 2.81;
v_z = 1.23;
z0 = -100;
N = 200;
dt = -z0/v_z/N;
psi_dot_m = 0.263;

x0 = -25;
y0 = -25;
psi_0 = 0;

hs = linspace(-z0, 0, N+1);
w = [interp1(heights, wind_profile(1,:), hs, 'linear','extrap');
     interp1(heights, wind_profile(2,:), hs, 'linear','extrap')];

eps = 0.1;
MAX_ITER = 15;

hold on
grid on
box on

%% Problem formulation
yalmip('clear')
x = sdpvar(2,N+1);
u = sdpvar(2,N+1);

% x0, u0
Cons = [x(:,1) == [x0; y0], u(:,1) == [v_ha*cos(psi_0); v_ha*sin(psi_0)]];
objective = lambda1*x(:,end)'*x(:,end)+lambda2*[-cos(psi_d)/v_ha, -sin(psi_d)/v_ha]*u(:,end);

% x1~N
for i = 2:N+1
    Cons = [Cons, x(:,i)==x(:,i-1)+dt/2*(u(:,i)+u(:,i-1)), ...
                  (u(:,i)-u(:,i-1))'*(u(:,i)-u(:,i-1)) <= (psi_dot_m*v_ha*dt)^2, ...
                  u(:,i)'*u(:,i) <= (v_ha + eps)^2];
    objective = objective + 1/(v_ha^2*dt)*(u(:,i)-u(:,i-1))'*(u(:,i)-u(:,i-1));
end


% SCP
u_last = [v_ha*cos(psi_0);
          v_ha*sin(psi_0)] * ones(1,N+1);

for n = 1:MAX_ITER
    constraints = Cons;
    u_last_normalized = u_last./vecnorm(u_last);
    for i = 2:N+1
%         constraints = [constraints, u_last(:,i)'*u(:,i) >= v_ha*(v_ha-eps)];
        constraints = [constraints, u_last_normalized(:,i)'*u(:,i) >= v_ha-eps];
    end
    
    options = sdpsettings('verbose',1,'solver','ecos');
    sol = optimize(constraints,objective,options);
    
    if sol.problem ~= 1
        if value(objective)<5
            xs = value(x);
            us = value(u);
            u_last = us;

            disp("At iteration "+num2str(n)+", final cost: "+num2str(value(objective)))
%             disp([lambda1*(xs(1,end)^2 + xs(2,end)^2), lambda2 * (1-cos(psi_d)*us(1,end)/Vhs(end) - sin(psi_d)*us(2,end)/Vhs(end))])
%             plot(xs(2,:), xs(1,:),'r')
            break
        else
            current_cost = value(objective);
            disp("At iteration "+num2str(n)+", current cost: "+num2str(current_cost))
            u_last = value(u);
        end
    else
        disp("###############################################")
        disp("[ERROR] Something went wrong at iteration "+num2str(n));
        disp(sol.info)
        disp("###############################################")
    end

%     if sol.problem == 0
%         % Extract and display value
%         u_last = value(u);
%         x_last = value(x);
%         plot(x_last(2,:),x_last(1,:),'LineWidth',1.5)
%         scatter(x_last(2,:),x_last(1,:),50,'k','filled')
%     else
%         disp('Hmm, something went wrong!');
%         sol.info
%         yalmiperror(sol.problem)
%         break
%     end
end


