test = load('data.mat','out').out;
time1 = test.tout;
time2 = test.chi.time;

ground_speed = test.ground_speed.signals.values;
vB = reshape(test.vB.signals.values,3,[])';

figure (1)
hold on
grid on

xlim([0,max(max(time1), max(time2))]);
plot(time2, vB(:,1),'b','LineWidth',1.5)
plot(time2, vB(:,2),'r','LineWidth',1.5)
plot(time2, vB(:,3),'g','LineWidth',1.5)
plot(time1, ground_speed(:,1),'k--','LineWidth',1.5)
plot(time1, ground_speed(:,2),'k--','LineWidth',1.5)
plot(time1, ground_speed(:,3),'k--','LineWidth',1.5)

set(gca,'FontSize',20)
xlabel('time [s]')
ylabel('velocity [m/s]')
title('Ground velocity estimation from direct navigation')
legend('estimation of v_{B_{x}}', 'estimation of v_{B_{y}}', 'estimation of v_{B_{z}}','ground truth','Location', 'best')

figure (2)

rpy = reshape(test.rpy.signals.values,3,[])';
chi = test.chi.signals.values;

rpy_dot = reshape(test.rpy_dot.signals.values,3,[])';
chi_dot = reshape(test.chi_dot.signals.values,2,[])';

subplot(1,2,1)
hold on
grid on
xlim([0,max(max(time1), max(time2))]);

plot(time2, chi, 'r', 'LineWidth',1.5)
plot(time1, rpy(:,3),'k--','LineWidth',1.5)

set(gca,'FontSize',20)
xlabel('time [s]')
ylabel('motion heading [rad]')
title('Motion heading estimation from direct navigation')
legend('estimation of motion heading \chi', 'ground truth','Location', 'best')

subplot(1,2,2)
hold on
grid on
xlim([0,max(max(time1), max(time2))]);

plot(time2, chi_dot(:,1), 'b', 'LineWidth',1.5)
plot(time2, chi_dot(:,2), 'r', 'LineWidth',1.5)
plot(time1, rpy_dot(:,3),'k--','LineWidth',1.5)

set(gca,'FontSize',20)
xlabel('time [s]')
ylabel('motion heading derivative [rad/s]')
title('Motion heading derivative estimation from direct navigation')
legend('direct difference', '3 backward difference', 'ground truth','Location', 'best')