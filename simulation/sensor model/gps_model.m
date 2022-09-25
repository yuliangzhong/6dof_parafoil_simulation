test = load('sensor.mat','out').out;

time = test.time.time;
gps = reshape(test.gps.signals.values,3,[])';
pos = test.I_r_IB.signals.values;

subplot(1,2,1)
hold on
grid on
try
    axis equal
catch
end
xlim([-40,10])
ylim([-25,25])
zlim([0,inf])

plot3(pos(:,2), pos(:,1), -pos(:,3),'k','LineWidth',1.5);
scatter3(gps(:,2), gps(:,1), -gps(:,3), 50,'red','filled');
set(gca,'FontSize',20)
xlabel('East [m]')
ylabel('North [m]')
zlabel('height [m]')
title('GNSS sensor readings in 3D')
legend('ground truth', 'GNSS output', 'Location', 'best')

subplot(1,2,2)
hold on
grid on
axis equal
xlim([-40,10])
ylim([-25,25])
plot(pos(:,2), pos(:,1),'k','LineWidth',1.5);
scatter(gps(:,2), gps(:,1),50,'red','filled');
set(gca,'FontSize',20)
xlabel('\rightarrow East [m]')
ylabel('\rightarrow North [m]')
title('GNSS sensor readings in xy-plane')
legend('ground truth', 'GNSS output', 'Location', 'best')