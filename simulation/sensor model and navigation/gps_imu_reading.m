test = load('data.mat','out').out;

%% GPS reading plot

time = test.time.time;
gps = reshape(test.gps.signals.values,3,[])';
pos = test.I_r_IB.signals.values;

figure (1)
subplot(1,3,1)
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

subplot(1,3,2)
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

subplot(1,3,3)
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

%% IMU reading plot

acc_time = test.acc_raw.time;
acc_raw = reshape(test.acc_raw.signals.values,3,[])';
acc_truth = test.acc_truth.signals.values;

figure (2)
subplot(1,3,1)
hold on
grid on
xlim([0,max(acc_time)])

plot(acc_time, acc_raw(:,1),'b','LineWidth',1.5);
plot(acc_time, acc_raw(:,2),'r','LineWidth',1.5);
plot(acc_time, acc_raw(:,3),'g','LineWidth',1.5);
plot(acc_time, acc_truth(:,1),'k--','LineWidth',1.5);
plot(acc_time, acc_truth(:,2),'k--','LineWidth',1.5);
plot(acc_time, acc_truth(:,3),'k--','LineWidth',1.5);
set(gca,'FontSize',20)
xlabel('time [s]')
ylabel('acceleration [m/s^{2}]')
title('Accelerometer sensor readings (gravitational acc included)')
legend('x-axis', 'y-axis', 'z-axis', 'ground truth', 'Location', 'best')

subplot(1,3,2)
hold on
grid on

gyro_time = test.gyro_raw.time;
gyro_raw = reshape(test.gyro_raw.signals.values,3,[])';
gyro_truth = test.gyro_truth.signals.values;
xlim([0,max(gyro_time)])

plot(gyro_time, gyro_raw(:,1),'b','LineWidth',1.5);
plot(gyro_time, gyro_raw(:,2),'r','LineWidth',1.5);
plot(gyro_time, gyro_raw(:,3),'g','LineWidth',1.5);
plot(gyro_time, gyro_truth(:,1),'k--','LineWidth',1.5);
plot(gyro_time, gyro_truth(:,2),'k--','LineWidth',1.5);
plot(gyro_time, gyro_truth(:,3),'k--','LineWidth',1.5);

set(gca,'FontSize',20)
xlabel('time [m]')
ylabel('angular velocity [rad/s]')
title('Gyroscope sensor readings')
legend('x-axis', 'y-axis', 'z-axis','ground truth','Location', 'best')

subplot(1,3,3)
hold on
grid on

gyro_time = test.gyro_raw.time;
gyro_raw = reshape(test.gyro_raw.signals.values,3,[])';
gyro_truth = test.gyro_truth.signals.values;
xlim([0,max(gyro_time)])

plot(gyro_time, gyro_raw(:,1),'b','LineWidth',1.5);
plot(gyro_time, gyro_raw(:,2),'r','LineWidth',1.5);
plot(gyro_time, gyro_raw(:,3),'g','LineWidth',1.5);
plot(gyro_time, gyro_truth(:,1),'k--','LineWidth',1.5);
plot(gyro_time, gyro_truth(:,2),'k--','LineWidth',1.5);
plot(gyro_time, gyro_truth(:,3),'k--','LineWidth',1.5);

set(gca,'FontSize',20)
xlabel('time [m]')
ylabel('angular velocity [rad/s]')
title('Gyroscope sensor readings')
legend('x-axis', 'y-axis', 'z-axis','ground truth','Location', 'best')


