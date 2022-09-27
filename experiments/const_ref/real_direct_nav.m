% read flight data of Test 4
filename = 'test4.xlsx';
opts = detectImportOptions(filename);
opts.SelectedVariableNames = [1:6]; 
opts.DataRange = '2:45';
states = readmatrix(filename,opts)';

% fit the trajectory by polynomials
px = polyfit(states(1,:), states(2,:), 10);
fx = @(x) px*[x.^10; x.^9; x.^8; x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
dfx = @(x) px*[10*x.^9; 9*x.^8; 8*x.^7; 7*x.^6; 6*x.^5; 5*x.^4; 4*x.^3; 3*x.^2; 2*x; ones(1,size(x,2)); zeros(1,size(x,2))];
ddfx = @(x) px*[90*x.^8; 72*x.^7; 56*x.^6; 42*x.^5; 30*x.^4; 20*x.^3; 12*x.^2; 6*x; 2*ones(1,size(x,2)); zeros(1,size(x,2)); zeros(1,size(x,2))];

py = polyfit(states(1,:), states(3,:), 10);
fy = @(x) py*[x.^10; x.^9; x.^8; x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
dfy = @(x) py*[10*x.^9; 9*x.^8; 8*x.^7; 7*x.^6; 6*x.^5; 5*x.^4; 4*x.^3; 3*x.^2; 2*x; ones(1,size(x,2)); zeros(1,size(x,2))];
ddfy = @(x) py*[90*x.^8; 72*x.^7; 56*x.^6; 42*x.^5; 30*x.^4; 20*x.^3; 12*x.^2; 6*x; 2*ones(1,size(x,2)); zeros(1,size(x,2)); zeros(1,size(x,2))];

%% plot GNSS data
figure (1)
subplot(1,2,1)
hold on
grid on
axis equal
xlim([-25,25])
ylim([-25,40])

scatter(states(3,:), states(2,:),50, 'r','filled');
plot(fy(states(1,:)), fx(states(1,:)),'k','LineWidth',1.5);

set(gca,'FontSize',20)
xlabel('\rightarrow East [m]')
ylabel('\rightarrow North [m]')
title('Real flight test trajectory in xy-plane')
legend('GNSS readings', 'polynomial trajectory fit', 'Location', 'best')

subplot(1,2,2)
hold on
grid on
axis equal
xlim([-25,25])
ylim([-25,40])

scatter(states(3,:), states(2,:),50, 'r','filled');
plot(fy(states(1,:)), fx(states(1,:)),'k','LineWidth',1.5);
plot(states(3,:), states(2,:),'r','LineWidth',1);

set(gca,'FontSize',20)
xlabel('\rightarrow East [m]')
ylabel('\rightarrow North [m]')
title('Real flight test trajectory in xy-plane')
legend('GNSS readings', 'polynomial trajectory fit', 'Location', 'best')

%% plot chi
figure (2)
subplot(1,2,1)
hold on
grid on

ts = linspace(min(states(1,:)), max(states(1,:)), 1000);

% real data
scatter(states(1,:), states(5,:),50,'r','filled');
plot(ts, atan2(dfy(ts),dfx(ts))/pi*180,'k--','LineWidth',1.5);

plot(states(1,:), states(5,:), 'r','LineWidth',1.5);


set(gca,'FontSize',20)
xlabel('time [s]')
ylabel('motion heading [deg]')
title('Motion heading estimated from direct navigation')
legend('estimated data','polynomial fit', 'Location', 'best')

% plot chi_dot
subplot(1,2,2)
hold on
grid on

scatter(states(1,:), states(6,:), 50, 'r','filled');

chi_dot_fit = -dfy(ts)./(dfx(ts).^2+dfy(ts).^2).*ddfx(ts)+...
              dfx(ts)./(dfx(ts).^2+dfy(ts).^2).*ddfy(ts);
plot(ts, chi_dot_fit/pi*180,'k--','LineWidth',1.5);

plot(states(1,:), states(6,:), 'r','LineWidth',1.5);

set(gca,'FontSize',20)
xlabel('time [s]')
ylabel('motion heading derivative [deg/s]')
title('Motion heading derivative estimated from direct navigation')
legend('estimated data', 'polynomial fit', 'Location', 'best')


% opts = detectImportOptions(filename);
% opts.SelectedVariableNames = [8:11]; 
% opts.DataRange = '2:1748';
% us = readmatrix(filename, opts);
