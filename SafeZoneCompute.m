function [Ax, bx, init_xy_pos] = SafeZoneCompute(if_plot)
% For now, assume the safe zone is a parallelogram ABCD. The origin is O
alpha1 = 80 / 180*pi;
alpha2 = -10 / 180*pi;

k1 = tan(alpha1);
k2 = tan(alpha2);

% calculate points
T = [cos(alpha2), -sin(alpha2);
     sin(alpha2), cos(alpha2)];
PA = T*[-60; -45];
PB = T*[60; -45];
PC = T*[60; 45];
PD = T*[-60; 45];
points = [PA, PB, PC, PD];

if if_plot
    hold on
    grid on
    patch(points(2,:), points(1,:),'g','FaceAlpha',.3);
    scatter(0, 0, 'r', 'filled')
    xlabel('East --> y') 
    ylabel('North --> x') 
    axis 'equal'
    hold off
end

% calculate constraints
Ax = [k1, -1;
      -k1, 1;
      k2, -1;
      -k2, 1];
bx = [k1*PB(1)-PB(2);
     -k1*PD(1)+PD(2);
      k2*PB(1)-PB(2);
     -k2*PD(1)+PD(2)];
init_xy_pos = PA;
end