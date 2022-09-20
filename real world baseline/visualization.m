% load data file
opts = detectImportOptions('pos_data.xlsx');

% load outputs
opts.SelectedVariableNames = [2:4]; 
opts.DataRange = '2:73';
pos = readmatrix('pos_data.xlsx',opts);
z0 = max(pos(:,3));
pos(:,3) = pos(:,3) - max(pos(:,3));

figure (1)
hold on
grid on
set(gca,'FontSize',20);
try
    axis equal
catch
end
title('Trajectory')
xlabel('--> East [m]')
ylabel('--> North [m]')
zlabel('--> height [m]')
origin_lat = 47.35504; % degree
origin_lon = 8.51982; % degree
Origin = [origin_lat; origin_lon];
P = LatLon2xy(47.3552145, 8.5196641, Origin);
scatter3(P(2), P(1),0, 100,'green','d','filled')
plot3(pos(:,2), pos(:,1), -pos(:,3),'blue');

pause(30)
for i=1:72
    scatter3(pos(i,2), pos(i,1), -pos(i,3),'red','filled');
%     pause(0.39)
end


function P = LatLon2xy(lat, lon, origin)
    % define the average radius of the earth
    R = 6371000; % m
    
    % calculate the position in meters
    P = [(lat - origin(1)) / 180 * pi * R;
         (lon - origin(2)) / 180 * pi * R * cos(origin(1) / 180 * pi)];
end