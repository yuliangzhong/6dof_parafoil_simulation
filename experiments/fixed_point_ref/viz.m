% load data file
opts = detectImportOptions('pos_data.xlsx');

% load outputs
opts.SelectedVariableNames = [2:4]; 
opts.DataRange = '2:73';
n = 72;
pos = readmatrix('pos_data.xlsx',opts);
z0 = max(pos(:,3));

% load time
opts = detectImportOptions('pos_data.xlsx');
opts.SelectedVariableNames = [1]; 
opts.DataRange = '2:73';
time = readmatrix('pos_data.xlsx',opts);
time = time - ones(n,1)*time(1);

% load control
opts = detectImportOptions('control_data.xlsx');
opts.SelectedVariableNames = [1,4]; 
opts.DataRange = '2:2827';
m = 2826;
control_data = readmatrix('control_data.xlsx',opts);
control_time = control_data(:,1) - ones(m,1)*control_data(1,1);
da = control_data(:,2);

% compensate for baro-error
pos(:,3) = pos(:,3) - max(pos(:,3));

figure (1)
hold on
grid on
box on
set(gca,'FontSize',20);
try
    axis equal
catch
end
title('Real flight test trajectory')
xlabel('East [m]')
ylabel('North [m]')
zlabel('height [m]')
origin_lat = 47.35504; % degree
origin_lon = 8.51982; % degree
Origin = [origin_lat; origin_lon];
P = LatLon2xy(47.3552145, 8.5196641, Origin);
% scatter3(P(2), P(1),0, 100,'greed','d','filled')
scatter3(0,0,0, 100,'greed','d','filled')
plot3(pos(:,2)-P(2)*ones(n,1), pos(:,1)-P(1)*ones(n,1), -pos(:,3),'k','LineWidth',1.5);

% pause(30)
for i=1:n
    scatter3(pos(i,2)-P(2), pos(i,1)-P(1), -pos(i,3),50, 'red','filled');
%     pause(0.39)
end

figure (2)

subplot(2,1,1)
hold on
grid on
box on

% target heading
plot(time, atan2(-(pos(:,2)-P(2)*ones(n,1)), -(pos(:,1)-P(1)*ones(n,1)))/pi*180,'k--','LineWidth',1.5)
plot(time(2:end), atan2(pos(2:end,2)-pos(1:end-1,2),pos(2:end,1)-pos(1:end-1,1))/pi*180,'r','LineWidth',1.5)
scatter(time, atan2(-(pos(:,2)-P(2)*ones(n,1)), -(pos(:,1)-P(1)*ones(n,1)))/pi*180,25,'k','filled')
scatter(time(2:end), atan2(pos(2:end,2)-pos(1:end-1,2),pos(2:end,1)-pos(1:end-1,1))/pi*180,25,'r','filled')

set(gca,'FontSize',20)
xlabel('time [s]')
ylabel('heading [deg]')
title('Desired heading and estimated heading from direct navigation')
legend('desired heading', 'current heading', 'Location', 'best')

subplot(2,1,2)
hold on
grid on
box on

plot(control_time, da, 'r','LineWidth',1.5);
plot([0,29],[0,0],'k--','LineWidth',1.5);
xlim([0,29])

set(gca,'FontSize',20)
xlabel('time [s]')
ylabel('control input \delta_a \in [-1,1]')
title('Control input \delta_a')




function P = LatLon2xy(lat, lon, origin)
    % define the average radius of the earth
    R = 6371000; % m
    
    % calculate the position in meters
    P = [(lat - origin(1)) / 180 * pi * R;
         (lon - origin(2)) / 180 * pi * R * cos(origin(1) / 180 * pi)];
end