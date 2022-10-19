% load data
all_states = cell(1,5);

for i = 1:5
    states = load(['states',num2str(i),'.mat']).states;
    all_states{i} = states;
end

% plot GNSS
hold on
grid on
box on
axis equal

for i = 1:5
    plot_GNSS(all_states{i},i)
end

text(0,-5,'start point (shifted)','HorizontalAlignment', 'right', 'VerticalAlignment', 'top','FontSize', 16)
plot([40,60],[0,0],'k','LineWidth',1.5)
plot([40,40],[0,3],'k','LineWidth',1.5)
plot([60,60],[0,3],'k','LineWidth',1.5)
text(39,-3,'0','FontSize',16)
text(56,-3,'20[m]','FontSize',16)
ylim([-20,90])
set(gca,'FontSize',20,'xticklabel',[],'yticklabel',[]);
xlabel('\rightarrow East')
ylabel('\rightarrow North')
title('Real flight test trajectory in xy-plane')
% legend('GNSS readings (starting point shifted)', 'Location', 'best')

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

%     % wind check
%     num = [11,11,8,22,13];
%     n = num(i);
%     ts = states(1,:);
%     xs = states(2,:);
%     ys = states(3,:);
% 
%     vxs = (xs(2:n) - xs(1:n-1))./(ts(2:n) - ts(1:n-1));
%     vys = (ys(2:n) - ys(1:n-1))./(ts(2:n) - ts(1:n-1));
%     Vs2 = vxs.^2+vys.^2;
%     mu_vx = mean(vxs);
%     mu_vy = mean(vys);
%     mu_Vs2 = mean(Vs2);
%     A = [vxs' - mu_vx*ones(n-1,1), vys' - mu_vy*ones(n-1,1)];
%     b = 1/2*(Vs2' - mu_Vs2*ones(n-1,1));
%     if rank(A) == 2
%         winds = (A'*A)\A'*b
%         angle = atan2(winds(2),winds(1))/pi*180
%         quiver(50,10,norm(winds)*sin(angle/180*pi),norm(winds)*cos(angle/180*pi),'k','filled','LineWidth',1.5)
%     else
%         disp('not full rank')
%     end

end
