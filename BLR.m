wind_err = out.wind_error.signals.values(:,:,600);

sigma_n = [0.05, 0.05, 0.05]; % sigma_nx, ny, nz;
Sigma_p = {diag([0.1, 1, 0.01]), diag([0.1, 1, 0.01]), diag([1, 0.01, 0.01])}; % Sigma_px, py, pz

H = linspace(1, 350, 3500);
phi = @(h) [ones(1, size(h,2)); h/normalize_const; h.^2/normalize_const^2];
xi_x = @(h) xi_w(1) * h/100;
xi_y = @(h) xi_w(2) * h/100;
xi_z = @(h) xi_w(3);
Phi = wind_err(1:3,:);

Yx = wind_err(4,:)';
Ax = sigma_n(1)^(-2)*(Phi*Phi') + Sigma_p{1}^(-1);
mu_x = sigma_n(1)^(-2)*phi(H)'/Ax*Phi*Yx;
sigma_Ex = diag(phi(H)'/Ax*phi(H));

Yy = wind_err(5,:)';
Ay = sigma_n(2)^(-2)*(Phi*Phi') + Sigma_p{2}^(-1);
mu_y = sigma_n(2)^(-2)*phi(H)'/Ay*Phi*Yy;
sigma_Ey = diag(phi(H)'/Ay*phi(H));

Yz = wind_err(6,:)';
Az = sigma_n(3)^(-2)*(Phi*Phi') + Sigma_p{3}^(-1);
mu_z = sigma_n(3)^(-2)*phi(H)'/Az*Phi*Yz;
sigma_Ez = diag(phi(H)'/Az*phi(H));

hold on
plot(H, mu_x, '.','MarkerSize',5)
plot(H, mu_x + sigma_Ex)
plot(H, mu_x - sigma_Ex)
plot(wind_err(2,:)*normalize_const, Yx)
plot(H, xi_x(H),'g')

plot(H, mu_y, '.','MarkerSize',5)
plot(H, mu_y + sigma_Ey)
plot(H, mu_y - sigma_Ey)
plot(wind_err(2,:)*normalize_const, Yy)
plot(H, xi_y(H),'g')

% plot(H, mu_z, '.','MarkerSize',5)
% plot(wind_err(2,:)*normalize_const, Yz)
% plot(H, xi_z(H)*ones(1, size(H,2)), 'g')