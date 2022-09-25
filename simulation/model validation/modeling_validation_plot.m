T1 = 10;
T2 = 25;

test1 = load('no_wind.mat','out').out;
time1 = test1.time.time;
dl1 = test1.delta_l.signals.values;
dr1 = test1.delta_r.signals.values;
da1 = dr1 - dl1;

test2 = load('with_wind.mat','out').out;
time2 = test2.time.time;
dl2 = test2.delta_l.signals.values;
dr2 = test2.delta_r.signals.values;
da2 = dr2 - dl2;

%% Fig 1

figure (1)
subplot(1,2,1)
hold on
plot(time1, da1,'r','linewidth',1.5);
set(gca,'FontSize',20)
xlabel('time [s]')
ylabel('asymmetric brake deflection \delta_{a}')
title('Control Inputs \delta_{a}')
legend('\delta_{a}')
grid on

subplot(1,2,2)
hold on
wind_gust = load('wind.mat', 'wind_gust').wind_gust;
heights = load('wind.mat', 'heights').heights;
wind_profile = load('wind.mat', 'wind_profile').wind_profile;
wind_truth = wind_gust + wind_profile;
plot(wind_truth(1,:), heights,'b','linewidth',1.5);
plot(wind_truth(2,:), heights,'r','linewidth',1.5);
plot(wind_truth(3,:), heights,'g','linewidth',1.5);
plot(wind_profile(1,:), heights,'b--','linewidth',1.5);
plot(wind_profile(2,:), heights,'r--','linewidth',1.5);
plot(wind_profile(3,:), heights,'g--','linewidth',1.5);
set(gca,'FontSize',20);
title('Wind Disturbance')
xlabel('wind [m/s]') 
ylabel('height [m]') 
legend('x-axis wind','y-axis wind','z-axis wind','x-axis wind profile','y-axis wind profile','z-axis wind profile')
grid on

%% Fig 2

pos1 = test1.I_r_IB.signals.values;
P1_T1 = find(time1>T1);
p1_T1 = P1_T1(1);
P1_T2 = find(time1>T2);
p1_T2 = P1_T2(1);

pos2 = test2.I_r_IB.signals.values;
P2_T1 = find(time2>T1);
p2_T1 = P2_T1(1);
P2_T2 = find(time2>T2);
p2_T2 = P2_T2(1);

figure (2)
subplot(1,2,1)
hold on
plot3(pos1(:,2),pos1(:,1),-pos1(:,3),'k','LineWidth',1.5);
plot3(pos2(:,2),pos2(:,1),-pos2(:,3),'r','LineWidth',1.5);
scatter3(pos1(1,2), pos1(1,1), -pos1(1,3), 50,'black','filled')
scatter3(pos1(end,2), pos1(end,1), -pos1(end-2,3), 50,'black','d','filled')
scatter3(pos2(end,2), pos2(end,1), -pos2(end-2,3), 50,'red','d','filled')

set(gca,'FontSize',20)
xlabel('\rightarrow East [m]')
ylabel('\rightarrow North [m]')
zlabel('\rightarrow height [m]')
title('Trajectories in 3D')
legend('without wind', 'with wind', 'Location', 'best')
grid on
try
    axis equal
catch
end
ylim([-25,30])
xlim([-40,35])
zlim([0,inf])

subplot(1,2,2)
hold on
axis equal
ylim([-25,30])
xlim([-40,35])
plot(pos1(:,2), pos1(:,1),'k','linewidth',1.5);
plot(pos2(:,2), pos2(:,1),'r','linewidth',1.5);

scatter(pos1(1,2), pos1(1,1), 50,'black','filled')
text(pos1(1,2)+1, pos1(1,1),'t=0','HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom','FontSize', 16)

scatter(pos1(p1_T1,2), pos1(p1_T1,1), 50,'black','filled')
text(pos1(p1_T1,2)+1, pos1(p1_T1,1),'t=10','HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom','FontSize', 16)

scatter(pos1(p1_T2,2), pos1(p1_T2,1), 50,'black','filled')
text(pos1(p1_T2,2)+1, pos1(p1_T2,1),'t=25','HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom','FontSize', 16)

scatter(pos1(end,2), pos1(end,1), 50,'black','d','filled')
text(pos1(end,2)+1, pos1(end,1),'t=T','HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom','FontSize', 16)

scatter(pos2(p2_T1,2), pos2(p2_T1,1), 50,'red','filled')
text(pos2(p2_T1,2)+1, pos2(p2_T1,1),'t=10','HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom','FontSize', 16)

scatter(pos2(p2_T2,2), pos2(p2_T2,1), 50,'red','filled')
text(pos2(p2_T2,2)+1, pos2(p2_T2,1),'t=25','HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom','FontSize', 16)

scatter(pos2(end,2), pos2(end,1), 50,'red','d','filled')
text(pos2(end,2)+1, pos2(end,1),'t=T','HorizontalAlignment', 'left', 'VerticalAlignment', 'top','FontSize', 16)

set(gca,'FontSize',20)
xlabel('\rightarrow East [m]')
ylabel('\rightarrow North [m]')
title('Trajectories in xy-plane')
legend('without wind', 'with wind', 'Location', 'best')
grid on

