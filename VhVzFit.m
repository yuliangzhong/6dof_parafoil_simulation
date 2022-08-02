% fit Vh, Vz

Ts = out.position.time;

xs = reshape(out.position.signals.values(1,1,:),[],1);
ys = reshape(out.position.signals.values(2,1,:),[],1);
zs = reshape(out.position.signals.values(3,1,:),[],1);

fit_linear = 0;

if fit_linear
    N = size(xs,1);
    t_end = out.position.time(end);
    Ts = linspace(0, t_end, N)';
    
    X = polyfit(Ts,xs,1);
    Xfit = X(1)*Ts + X(2);
    Y = polyfit(Ts,ys,1);
    Yfit = Y(1)*Ts + Y(2);
    Z = polyfit(Ts,zs,1);
    Zfit = Z(1)*Ts + Z(2);
    
    hold on
    scatter(Ts, xs, 5, 'filled')
    scatter(Ts, ys, 5, 'filled')
    scatter(Ts, zs, 5, 'filled')
    plot(Ts,Xfit,'--','LineWidth',5);
    plot(Ts,Yfit,'--','LineWidth',5);
    plot(Ts,Zfit,'--','LineWidth',5);
    
    grid on
    
    disp([X(1), Y(1), sqrt(X(1)^2+Y(1)^2), Z(1)])
else
    a = [xs, ys, ones(size(xs))]\(-(xs.^2+ys.^2));
    xc = -a(1)/2;
    yc = -a(2)/2;
    R = sqrt((a(1)^2+a(2)^2)/4-a(3));
    thetas = linspace(0, 2*pi, 100);
    xt = xc+R*cos(thetas);
    yt = yc+R*sin(thetas);
    hold on
    plot(xt, yt,'--','LineWidth',5)
    scatter(xs, ys, 10, 'filled')
    axis equal
end