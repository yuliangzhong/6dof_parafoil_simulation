test = load('data.mat','out').out;
time1 = test.tout;
ground_speed = test.ground_speed.signals.values;

time2 = test.chi.time;
vB = reshape(test.vB.signals.values,3,[])';

test_ekf4 = load('ekf4.mat','out').out;
time3 = test_ekf4.vzs.time;
vzs = reshape(test_ekf4.vzs.signals.values,6,[])';

hold on
grid on
box on

xlim([0,max(max(time1), max(time2))]);
plot(time2, vB(:,3),'g','LineWidth',1.5)
plot(time3, vzs(:,6),'r','LineWidth',1.5)
plot(time1, ground_speed(:,3),'k--','LineWidth',1.5)

set(gca,'FontSize',20)
xlabel('time [s]')
ylabel('velocity [m/s]')
title('Vertical ground velocity noise filtering')
legend('from direct navigation', 'filtered by 4 DOF EKF','ground truth','Location', 'best')
