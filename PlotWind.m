function PlotWind(ifPlotWindInfo, heights, wind_profile, wind_gust)

if ifPlotWindInfo
    subplot(1,2,1)
    hold on
    wind_truth = wind_gust + wind_profile;
    plot(wind_truth(1,:), heights,'b','linewidth',1.5);
    plot(wind_truth(2,:), heights,'r','linewidth',1.5);
    plot(wind_truth(3,:), heights,'g','linewidth',1.5);
    plot(wind_profile(1,:), heights,'b--','linewidth',1.5);
    plot(wind_profile(2,:), heights,'r--','linewidth',1.5);
    plot(wind_profile(3,:), heights,'g--','linewidth',1.5);
    set(gca,'FontSize',20);
    xlabel('wind [m/s]') 
    ylabel('height [m]') 
    legend('x-axis wind','y-axis wind','z-axis wind','x-axis wind profile','y-axis wind profile','z-axis wind profile')
    grid on

    subplot(1,2,2)
    hold on
    plot(wind_gust(1,:), heights,'linewidth',1.5);
    plot(wind_gust(2,:), heights,'linewidth',1.5);
    plot(wind_gust(3,:), heights,'linewidth',1.5);
    set(gca,'FontSize',20);
    xlabel('wind gust [m/s]') 
    ylabel('height [m]') 
    legend('x-axis wind gust', 'y-axis wind gust', 'z-axis wind gust')
    grid on
end

end