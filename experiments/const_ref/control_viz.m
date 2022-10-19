% load data
all_states = cell(1,5);
all_inputs = cell(1,5);

for i = 1:5
    states = load(['states',num2str(i),'.mat']).states;
    inputs = load(['inputs',num2str(i),'.mat']).inputs;
    all_states{i} = states;
    all_inputs{i} = inputs;
end
colors = colormap(turbo);

% plot heading estimation
subplot(3,1,1)
hold on
grid on
box on
index = [1,2,3,4,5];
for i = 1:size(index,2)
    color = colors(50*index(i),:);
    states = all_states{index(i)};
    plot(states(1,:), states(5,:), 'Color',color,'LineWidth',1.5);
    scatter(states(1,:),states(5,:),15,color,'filled');
end
plot([0,22],[0,0],'k--','LineWidth',1.5);
xlim([0,22])


set(gca,'FontSize',15)
xlabel('time [s]')
ylabel('motion heading [deg]')
title('Motion heading estimated from direct navigation')

% plot heading derivative estimation
subplot(3,1,2)
hold on
grid on
box on
for i = 1:size(index,2)
    color = colors(50*index(i),:);
    states = all_states{index(i)};
    plot(states(1,:), states(6,:), 'Color',color,'LineWidth',1.5);
    scatter(states(1,:),states(6,:),15,color,'filled');
end
plot([0,22],[0,0],'k--','LineWidth',1.5);
xlim([0,22])


set(gca,'FontSize',15)
xlabel('time [s]')
ylabel('motion heading derivative [deg/s]')
title('Motion heading derivative estimated from direct navigation')
% legend('test1','test2','test3','test4','test5', 'Location', 'best')

% plot control input
subplot(3,1,3)
hold on
grid on
box on

for i = 1:size(index,2)
    color = colors(50*index(i),:);
%     states = all_states{index(i)};
    inputs = all_inputs{index(i)};
    plot(inputs(1,:), inputs(2,:), 'Color',color,'LineWidth',1.5);
end
plot([0,22],[0,0],'k--','LineWidth',1.5);
xlim([0,22])

set(gca,'FontSize',15)
xlabel('time [s]')
ylabel('control input \delta_a \in [-1,1]')
title('Control input \delta_a')