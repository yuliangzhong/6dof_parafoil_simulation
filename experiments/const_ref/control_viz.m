% load data
all_inputs = cell(1,5);

for i = 1:5
    inputs = load(['inputs',num2str(i),'.mat']).inputs;
    all_inputs{i} = inputs;
end
colors = colormap(turbo);

% plot control input
hold on
grid on
for i = 4:5
    color = colors(50*i,:);
    inputs = all_inputs{i};
    plot(inputs(1,:), inputs(2,:), 'Color',color,'LineWidth',1.5);
end
plot([0,22],[0,0],'k--','LineWidth',1.5);
xlim([0,22])

set(gca,'FontSize',20)
xlabel('time [s]')
ylabel('control input \delta_a \in [-1,1]')
title('Control input \delta_a')
legend('control input of test4','control input of test5','Location','best')