% TODO

xy_dot = 4.59;
z_dot = 1.39;

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

hold on;
plot(ref(2,:),ref(1,:))


% [U_star, ~, flag] = fminsearch(@(U) Loss(U, init_pose, dx, dy, N, ref, xy_dot, Ts),zeros(1,N-1),optimset('TolFun',1e-7,'TolX',1e-7,'MaxFunEvals',1e5,'MaxIter',1e5));
% disp(flag)
% if flag
% %     control(1,:) = Guide(3,id)*ones(1,N);
% %     control(2,:) = psi_c * ones(1,N);
% %     for i = 2:N
% %         control(:,i) = control(:,i-1) + [-dh;
% %                                          Ts * U_star(i-1)];
% %     end
%     pos = init_pose * ones(1,N);
%     for i = 2:N
%         if ref(:,i) == ref(:,i-1)
%             pos(:,i) = pos(:,i-1);
%         else
%             pos(:,i) = pos(:,i-1) + [Ts * (xy_dot * cos(pos(3,i-1)) + dx);
%                                      Ts * (xy_dot * sin(pos(3,i-1)) + dy);
%                                      Ts * U_star(i-1)];
%         end
%     end
% end
% 
% plot(pos(1,:), pos(2,:))


function loss = Loss(U, init_pose, dx, dy, N, ref, xy_dot, Ts)
    loss = 0;
    pos = init_pose * ones(1,N);
    Q = diag([100,100]);
    r = 1000;
    for i = 2:N
        if ref(:,i) == ref(:,i-1)
            pos(:,i) = pos(:,i-1);
        else
            pos(:,i) = pos(:,i-1) + [Ts * (xy_dot * cos(pos(3,i-1)) + dx);
                                     Ts * (xy_dot * sin(pos(3,i-1)) + dy);
                                     Ts * U(i-1)];
        end
        loss = loss + (pos(1:2,i) - ref(:,i))'*Q*(pos(1:2,i) - ref(:,i)) + r*U(i-1)^2;
    end
end