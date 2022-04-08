wind_err = out.wind_err.signals.values(:,:,1000);

X = wind_err(1,end-50:end)';
Yx = wind_err(2,end-50:end)';
Yy = wind_err(3,:)';
Yz = wind_err(4,:)'; % at h = 0, no error

sigma2_x = max(wind_err(5,:));
sigma2_y = max(wind_err(6,:));
sigma2_z = max(wind_err(7,:));

gprMdl1 = fitrgp(X, Yx, 'KernelFunction','matern32');

H = linspace(50, 270)';
[xpred1,~,xint1] = predict(gprMdl1,H);


hold on
plot(H,xpred1,'r')                  % GPR predictions
scatter(X, Yx, 'filled');
patch([H;flipud(H)],[xint1(:,1);flipud(xint1(:,2))],'k','FaceAlpha',0.1); % Prediction intervals
% Dx = H; Tx = H; 
% Dy = H;
% Dz = H;
% 
% Sx = H;
% Sy = H;
% Sz = H;
% 
% KK = K(X, X);
% 
% for a =1:3500
%     k_star = K(X, H(a));
%     Dx(a) =k_star'/(KK + sigma2_x*eye(201))*Yx;
%     Dy(a) =k_star'/(KK + sigma2_y*eye(201))*Yy;
%     Dz(a) =k_star'/(KK + sigma2_z*eye(201))*Yz;
%     Tx(a) = xi_w(1)*(log(H(a)+1));
%     Sx(a) = K(H(a),H(a)) - k_star'/(KK + sigma2_x*eye(201))*k_star;
%     Sy(a) = K(H(a),H(a)) - k_star'/(KK + sigma2_y*eye(201))*k_star;
%     Sz(a) = K(H(a),H(a)) - k_star'/(KK + sigma2_z*eye(201))*k_star;
% end
% 
% hold on
% plot(X, Yx, 'b');
% plot(H, Dx, 'r');
% plot(H, Dx+Sx, 'black');
% plot(H, Dx-Sx, 'black');
% plot(H, Tx, 'g');
% 
% 
% 
% %%
% function result = K(X1, X2)
%     n1 = size(X1,1);
%     n2 = size(X2,1);
%     result = zeros(n1, n2);
%     for i=1:n1
%         for j = 1:n2
%             result(i,j) = kernel(X1(i), X2(j));
%         end
%     end
% end
% 
% 
% function k = kernel(x1, x2)
%     sigma2 = 0.1; l = 50;
%     k = sigma2*exp(-(x1-x2)^2/(2*l^2));
% end
% 
