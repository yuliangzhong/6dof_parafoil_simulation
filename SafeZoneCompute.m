% In this function, define the safezone and initial position from reality!
% The format of safezone constraints are:
% at height h, Ax * [x;y] <= bx --> [Ax, bx, h] =def= Axbxh

function [Axbxh, init_xy_pos] = SafeZoneCompute(if_plot)
% For now, assume the safe zone is a parallelogram ABCD. The origin is O
alpha1 = 80 / 180*pi;
alpha2 = -10 / 180*pi;

k1 = tan(alpha1);
k2 = tan(alpha2);

% calculate projected points at ground
T = [cos(alpha2), -sin(alpha2);
     sin(alpha2), cos(alpha2)];
PA = T*[-60; -45];
PB = T*[60; -45];
PC = T*[60; 45];
PD = T*[-60; 45];

% calculate constraints
Ax = [k1, -1;
      -k1, 1;
      k2, -1;
      -k2, 1];
bx = [k1*PB(1)-PB(2);
     -k1*PD(1)+PD(2);
      k2*PB(1)-PB(2);
     -k2*PD(1)+PD(2)];
h = [10; 10; 10; 10]; % stay in safezone when height < 10 m
Axbxh = [Ax, bx, h];

init_xy_pos = PA+[5;5];

% draw safezone constraints
if if_plot
    figure(2)
    hold on
    grid on
    xlim([-80,80])
    ylim([-80,80])
    zlim([0,inf])
    

    points = [PA, PB, PC, PD, PA];
    fill3(points(2,:), points(1,:),zeros(5,1),'g','FaceAlpha',.3)
    scatter(0, 0, 'r')
    scatter(init_xy_pos(2),init_xy_pos(1),'b','filled')
    for i = 1:4
        fill3([points(2,i), points(2,i), points(2,i+1), points(2,i+1)], ...
              [points(1,i), points(1,i), points(1,i+1), points(1,i+1)], ...
              [h(i), 0, 0, h(i)], 'b','FaceAlpha', .3);
    end 
    set(gca,'FontSize',20);
    xlabel('East --> y') 
    ylabel('North --> x') 
    zlabel('Height --> z')
end

end