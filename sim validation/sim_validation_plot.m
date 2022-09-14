subplot(2,2,1)
times1 = load('data.mat','times').times;
da = load('data.mat','das').das;
hold on
plot(times1, da,'r','linewidth',1.5);
set(gca,'FontSize',20)
xlabel('time [s]')
ylabel('asymmetric brake deflection \delta_{a}')
title('Control Inputs')
legend('\delta_{a}')
grid on


subplot(2,2,2)
hold on
delta_ws = load('data.mat', 'delta_ws').delta_ws;
heights = load('data.mat', 'heights').heights;
wind_profile_hat = load('data.mat', 'wind_profile_hat').wind_profile_hat;
wind_truth = delta_ws + wind_profile_hat;
plot(wind_truth(1,:), heights,'b','linewidth',1.5);
plot(wind_truth(2,:), heights,'r','linewidth',1.5);
plot(wind_truth(3,:), heights,'g','linewidth',1.5);
plot(wind_profile_hat(1,:), heights,'k--','linewidth',1.5);
plot(wind_profile_hat(2,:), heights,'k--','linewidth',1.5);
plot(wind_profile_hat(3,:), heights,'k--','linewidth',1.5);
set(gca,'FontSize',20);
xlabel('wind gust [m/s]') 
ylabel('height [m]') 
legend('x-axis wind', 'y-axis wind', 'z-axis wind', 'wind profile')
title('Wind Disturbance')
grid on


subplot(2,2,3)
hold on
pos1 = load('data.mat', 'pos').pos;
times1 = load('data.mat', 'times').times;
P10_1 = find(times1>10);
p10_1 = P10_1(1);
P20_1 = find(times1>20);
p20_1 = P20_1(1);
plot(pos1(:,2), pos1(:,1),'k','linewidth',1.5);

pos2 = load('data2.mat', 'pos').pos;
times2 = load('data2.mat', 'times').times;
P10_2 = find(times2>10);
p10_2 = P10_2(1);
P20_2 = find(times2>20);
p20_2 = P20_2(1);
plot(pos2(:,2), pos2(:,1),'r','linewidth',1.5);

scatter(pos1(1,2), pos1(1,1), 100,'black')
scatter(pos1(p10_1,2), pos1(p10_1,1), 50,'blue','filled')
scatter(pos1(p20_1,2), pos1(p20_1,1), 50,'green','filled')
scatter(pos1(end,2), pos1(end,1), 100,'black','d','filled')
scatter(pos2(p10_2,2), pos2(p10_2,1), 50,'blue','filled')
scatter(pos2(p20_2,2), pos2(p20_2,1), 50,'green','filled')
scatter(pos2(end,2), pos2(end,1), 100,'red','d','filled')

set(gca,'FontSize',20)
xlabel('--> East [m]')
ylabel('--> North [m]')
title('Trajectories in xy-plane')
legend('without wind', 'with wind','start pos','input:10s','input:20s','final pos')
grid on
axis equal

subplot(2,2,4)
hold on

plot(pos1(:,1), -pos1(:,3),'k','linewidth',1.5);
plot(pos2(:,1), -pos2(:,3),'r','linewidth',1.5);

scatter(pos1(1,1), -pos1(1,3), 100,'black')
scatter(pos1(p10_1,1), -pos1(p10_1,3), 50,'blue','filled')
scatter(pos1(p20_1,1), -pos1(p20_1,3), 50,'green','filled')
scatter(pos1(end,1), -pos1(end,3), 100,'black','d','filled')
scatter(pos2(p10_2,1), -pos2(p10_2,3), 50,'blue','filled')
scatter(pos2(p20_2,1), -pos2(p20_2,3), 50,'green','filled')
scatter(pos2(end,1), -pos2(end,3), 100,'red','d','filled')

set(gca,'FontSize',20)
xlabel('--> North [m]')
ylabel('--> height [m]')
title('Trajectories in xz-plane')
legend('without wind', 'with wind','start pos','input:10s','input:20s','final pos')
grid on
axis equal
