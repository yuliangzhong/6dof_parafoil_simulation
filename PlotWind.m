function PlotWind(ifPlotWindInfo, heights, wind_profile_hat, delta_ws)

if ifPlotWindInfo
    subplot(1,2,1)
    hold on
    plot(delta_ws(1,:), heights,'linewidth',1.5);
    plot(delta_ws(2,:), heights,'linewidth',1.5);
    plot(delta_ws(3,:), heights,'linewidth',1.5);
    set(gca,'FontSize',25);
    xlabel('wind gust [m/s]') 
    ylabel('height [m]') 
    legend('x-axis', 'y-axis', 'z-axis')
    subplot(1,2,2)
    hold on
    plot(wind_profile_hat(1,:), heights,'linewidth',1.5);
    plot(wind_profile_hat(2,:), heights,'linewidth',1.5);
    plot(wind_profile_hat(3,:), heights,'linewidth',1.5);
    wind_truth = delta_ws + wind_profile_hat;
    plot(wind_truth(1,:), heights,'b','linewidth',1.5);
    plot(wind_truth(2,:), heights,'r','linewidth',1.5);
    plot(wind_truth(3,:), heights,'y','linewidth',1.5);
    set(gca,'FontSize',25);
    xlabel('wind profile [m/s]') 
    ylabel('height [m]') 
    legend('x-axis wind profile','y-axis wind profile','z-axis wind profile')
end

end