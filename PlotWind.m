function PlotWind(ifPlotWindInfo, heights, wind_profile, wind_gust)

avg_disturbance_norm = mean(vecnorm(wind_gust(1:2,:)));
max_disturbance_norm = max(vecnorm(wind_gust(1:2,:)));
disp("Horizontal Wind Gust:")
disp("Maximum: "+num2str(max_disturbance_norm) + " [m/s], Average: "+num2str(avg_disturbance_norm) + " [m/s]");
disp("======================================")

if ifPlotWindInfo
    subplot(1,3,1)
    hold on
    box on
    plot(wind_profile(1,:), heights,'b--','linewidth',1.5);
    plot(wind_profile(2,:), heights,'r--','linewidth',1.5);
    plot(wind_profile(3,:), heights,'g--','linewidth',1.5);
    set(gca,'FontSize',20);
    title("Wind profile")
    xlabel('wind profile [m/s]') 
    ylabel('height [m]') 
    legend('x-axis wind profile','y-axis wind profile','z-axis wind profile')
    grid on

    subplot(1,3,2)
    hold on
    box on
    plot(wind_gust(1,:), heights,'b','linewidth',1.5);
    plot(wind_gust(2,:), heights,'r','linewidth',1.5);
    plot(wind_gust(3,:), heights,'g','linewidth',1.5);
    set(gca,'FontSize',20);
    title("Wind gust")
    xlabel('wind gust [m/s]') 
    ylabel('height [m]') 
    legend('x-axis wind gust', 'y-axis wind gust', 'z-axis wind gust')
    grid on

    subplot(1,3,3)
    hold on
    box on
    wind_truth = wind_gust + wind_profile;
    plot(wind_truth(1,:), heights,'b','linewidth',1.5);
    plot(wind_truth(2,:), heights,'r','linewidth',1.5);
    plot(wind_truth(3,:), heights,'g','linewidth',1.5);
    plot(wind_profile(1,:), heights,'k--','linewidth',1.5);
    plot(wind_profile(2,:), heights,'k--','linewidth',1.5);
    plot(wind_profile(3,:), heights,'k--','linewidth',1.5);
    set(gca,'FontSize',20);
    title('Wind disturbance')
    xlabel('wind disturbance [m/s]') 
    ylabel('height [m]') 
    legend('x-axis wind','y-axis wind','z-axis wind','wind profile')
    grid on
end

end