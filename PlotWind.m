function PlotWind(ifPlotWindInfo, heights, wind_profile_hat, delta_ws)

if ifPlotWindInfo
    subplot(1,2,1)
    hold on
    plot(delta_ws(1,:), heights,'linewidth',1.5);
    plot(delta_ws(2,:), heights,'linewidth',1.5);
    plot(delta_ws(3,:), heights,'linewidth',1.5);
    set(gca,'FontSize',20);
    xlabel('wind gust [m/s]') 
    ylabel('height [m]') 
    legend('x-axis wind gust', 'y-axis wind gust', 'z-axis wind gust')
    grid on
    subplot(1,2,2)
    hold on
    wind_truth = delta_ws + wind_profile_hat;
    plot(wind_truth(1,:), heights,'b','linewidth',1.5);
    plot(wind_truth(2,:), heights,'r','linewidth',1.5);
    plot(wind_truth(3,:), heights,'g','linewidth',1.5);
    plot(wind_profile_hat(1,:), heights,'b--','linewidth',1.5);
    plot(wind_profile_hat(2,:), heights,'r--','linewidth',1.5);
    plot(wind_profile_hat(3,:), heights,'g--','linewidth',1.5);

    set(gca,'FontSize',20);
    xlabel('wind profile [m/s]') 
    ylabel('height [m]') 
    legend('x-axis wind','y-axis wind','z-axis wind')
    grid on
end

end