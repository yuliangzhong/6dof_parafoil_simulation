% In this function, define the safezone and initial position from reality!
% The format of safezone constraints are:
% at height h, Ax * [x;y] <= bx --> [Ax, bx, h] =def= Axbxh

function [Axbxh, init_xy_pos] = SafeZoneCompute(if_plot, origin_lat, origin_lon)

% For now, assume the safe zone is a parallelogram ABCD(clockwize)
% The origin is O. A is in the third quadrant

PA = LatLon2xy(47.35422, 8.51943, origin_lat, origin_lon);
PB = LatLon2xy(47.35544, 8.51869, origin_lat, origin_lon);
PC = -PA;
PD = -PB;

k1 = (PD(2) - PA(2))/(PD(1) - PA(1));
k2 = (PB(2) - PA(2))/(PB(1) - PA(1));

% set constraints
Ax = [k1, -1;
      -k1, 1;
      k2, -1;
      -k2, 1];
bx = [k1*PB(1)-PB(2);
     -k1*PD(1)+PD(2);
      k2*PB(1)-PB(2);
     -k2*PD(1)+PD(2)];
h = 15 * ones(4,1); % stay in safezone when height < 10 m
Axbxh = [Ax, bx, h];

init_xy_pos = [-20; 1];

% draw safezone constraints
if if_plot
    figure(2)
    hold on
    grid on
    xlim([PB(2)-5, PD(2)+5])
    ylim([PA(1)-5, PC(1)+5])
    zlim([0,inf])
    

    points = [PA, PB, PC, PD, PA];
    fill3(points(2,:), points(1,:),zeros(5,1),'g','FaceAlpha',.3)
    scatter(0, 0, 50, 'r','filled','diamond')
    scatter(init_xy_pos(2),init_xy_pos(1),'b','filled')
    for i = 1:4
        fill3([points(2,i), points(2,i), points(2,i+1), points(2,i+1)], ...
              [points(1,i), points(1,i), points(1,i+1), points(1,i+1)], ...
              [h(i), 0, 0, h(i)], 'b','FaceAlpha', .3);
    end
    legend('testing field','desired landing point: origin','starting point (projection)','obstacle constraints')
    set(gca,'FontSize',20);
    xlabel('East --> y') 
    ylabel('North --> x') 
    zlabel('Height --> z')
    try
        axis equal
    catch
    end
end

end

function P = LatLon2xy(lat, lon, origin_lat, origin_lon)
    % define the average radius of the earth
    R = 6371000; % m
    
    % calculate the position in meters
    P = [(lat - origin_lat) / 180 * pi * R;
         (lon - origin_lon) / 180 * pi * R * cos(origin_lat / 180 * pi)];
end