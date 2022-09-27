% load data
all_states = cell(1,5);

for i = 1:5
    states = load(['states',num2str(i),'.mat']).states;
    all_states{i} = states;
end

% plot GNSS
hold on
grid on
axis equal

for i = 1:5
    plot_GNSS(all_states{i},i)
end

text(0,-5,'start point (shifted)','HorizontalAlignment', 'right', 'VerticalAlignment', 'top','FontSize', 16)
plot([40,60],[0,0],'k','LineWidth',1.5)
plot([40,40],[0,3],'k','LineWidth',1.5)
plot([60,60],[0,3],'k','LineWidth',1.5)
text(38,-3,'0','FontSize',16)
text(56,-3,'20[m]','FontSize',16)
ylim([-20,90])
set(gca,'FontSize',20,'xticklabel',[],'yticklabel',[]);
xlabel('\rightarrow East')
ylabel('\rightarrow North')
title('Real flight test trajectory in xy-plane')
legend('GNSS readings (starting point shifted)', 'Location', 'best')

function plot_GNSS(states, i)
    
    % shift them to one starting point
    states(2,:) = states(2,:) - states(2,1);
    states(3,:) = states(3,:) - states(3,1);

    % fit the trajectory by polynomials
    px = polyfit(states(1,:), states(2,:), 10);
    fx = @(x) px*[x.^10; x.^9; x.^8; x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
    dfx = @(x) px*[10*x.^9; 9*x.^8; 8*x.^7; 7*x.^6; 6*x.^5; 5*x.^4; 4*x.^3; 3*x.^2; 2*x; ones(1,size(x,2)); zeros(1,size(x,2))];
    ddfx = @(x) px*[90*x.^8; 72*x.^7; 56*x.^6; 42*x.^5; 30*x.^4; 20*x.^3; 12*x.^2; 6*x; 2*ones(1,size(x,2)); zeros(1,size(x,2)); zeros(1,size(x,2))];
    
    py = polyfit(states(1,:), states(3,:), 10);
    fy = @(x) py*[x.^10; x.^9; x.^8; x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
    dfy = @(x) py*[10*x.^9; 9*x.^8; 8*x.^7; 7*x.^6; 6*x.^5; 5*x.^4; 4*x.^3; 3*x.^2; 2*x; ones(1,size(x,2)); zeros(1,size(x,2))];
    ddfy = @(x) py*[90*x.^8; 72*x.^7; 56*x.^6; 42*x.^5; 30*x.^4; 20*x.^3; 12*x.^2; 6*x; 2*ones(1,size(x,2)); zeros(1,size(x,2)); zeros(1,size(x,2))];
   
    hold on
    colors = colormap(turbo);
    color = colors(50*i,:);
    scatter(states(3,:), states(2,:), 50, color, "filled");
    plot(fy(states(1,:)), fx(states(1,:)),'Color',color,'LineWidth',1.5);
    scatter(states(3,end),states(2,end),100, color,'filled','d');
    text(states(3,end)+1,states(2,end),['test',num2str(i)],'HorizontalAlignment', 'left', 'VerticalAlignment', 'top','FontSize', 16)
    scatter(states(3,1),states(2,1),100,'k','filled');

end
% 
% function plot_chi(states, i)
%     
%     % shift them to one starting point
%     states(2,:) = states(2,:) - states(2,1);
%     states(3,:) = states(3,:) - states(3,1);
% 
%     % fit the trajectory by polynomials
%     px = polyfit(states(1,:), states(2,:), 10);
%     fx = @(x) px*[x.^10; x.^9; x.^8; x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
%     dfx = @(x) px*[10*x.^9; 9*x.^8; 8*x.^7; 7*x.^6; 6*x.^5; 5*x.^4; 4*x.^3; 3*x.^2; 2*x; ones(1,size(x,2)); zeros(1,size(x,2))];
%     ddfx = @(x) px*[90*x.^8; 72*x.^7; 56*x.^6; 42*x.^5; 30*x.^4; 20*x.^3; 12*x.^2; 6*x; 2*ones(1,size(x,2)); zeros(1,size(x,2)); zeros(1,size(x,2))];
%     
%     py = polyfit(states(1,:), states(3,:), 10);
%     fy = @(x) py*[x.^10; x.^9; x.^8; x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
%     dfy = @(x) py*[10*x.^9; 9*x.^8; 8*x.^7; 7*x.^6; 6*x.^5; 5*x.^4; 4*x.^3; 3*x.^2; 2*x; ones(1,size(x,2)); zeros(1,size(x,2))];
%     ddfy = @(x) py*[90*x.^8; 72*x.^7; 56*x.^6; 42*x.^5; 30*x.^4; 20*x.^3; 12*x.^2; 6*x; 2*ones(1,size(x,2)); zeros(1,size(x,2)); zeros(1,size(x,2))];
%    
%     hold on
%     colors = colormap(turbo);
%     color = colors(50*i,:);
% 
%     ts = linspace(min(states(1,:)), max(states(1,:)), 1000);
%     scatter(states(1,:), states(5,:), 50, color, "filled");
% %     plot(ts, atan2(dfy(ts),dfx(ts))/pi*180,'Color',color,'LineWidth',1.5,'LineStyle','--');
%     plot(states(1,:), states(5,:),'Color',color,'LineWidth',1.5,'LineStyle','-');
% 
% end