try

close all

hfig=figure('NumberTitle','off','name','Clock Animation Demo--matlabfan','MenuBar','none');

theta=linspace(0,6.3,1000);

x1=8*cos(theta);y1=8*sin(theta);

plot(x1,y1,'b','linewidth',1.4)%绘制外表盘

hold on

axis equal

x2=7*cos(theta);y2=7*sin(theta);

plot(x2,y2,'y','linewidth',3.5)%绘制内表盘

fill(0.4*cos(theta),0.4*sin(theta),'r');%绘制指针转轴

axis off

axis([-10 10 -10 10])

set(gca,'position',[[0.13 0.05 0.775 0.815]])

title(date,'fontsize',18)

for k=1:12

    xk=9*cos(-2*pi/12*k+pi/2);
    
    yk=9*sin(-2*pi/12*k+pi/2);
    
    plot([xk/9*8 xk/9*7],[yk/9*8
    
    yk/9*7],'color',[0.3 0.8 0.9]);
    
    text(xk,yk,num2str(k),'fontsize',16,'color',[0.9 0.3 0.8],'HorizontalAlignment','center');%表盘时刻标度

end

% % 计算时针位置
% 
% ti=clock;
% 
% th=-(ti(4)+ti(5)/60+ti(6)/3600)/12*2*pi+pi/2;
% 
% xh3=4.0*cos(th);
% 
% yh3=4.0*sin(th);
% 
% xh2=xh3/2+0.5*cos(th-pi/2);
% 
% yh2=yh3/2+0.5*sin(th-pi/2);
% 
% xh4=xh3/2-0.5*cos(th-pi/2);
% 
% yh4=yh3/2-0.5*sin(th-pi/2);
% 
% hh=fill([0 xh2 xh3 xh4 0],[0 yh2 yh3 yh4
% 
% 0],[0.6 0.5 0.3]);
% 
% % 计算分针位置
% 
% tm=-(ti(5)+ti(6)/60)/60*2*pi+pi/2;
% 
% xm3=6.0*cos(tm);
% 
% ym3=6.0*sin(tm);
% 
% xm2=xm3/2+0.5*cos(tm-pi/2);
% 
% ym2=ym3/2+0.5*sin(tm-pi/2);
% 
% xm4=xm3/2-0.5*cos(tm-pi/2);
% 
% ym4=ym3/2-0.5*sin(tm-pi/2);
% 
% hm=fill([0 xm2 xm3 xm4 0],[0 ym2 ym3 ym4
% 
% 0],[0.6 0.5 0.3]);
% 
% % 计算秒针位置
% 
% ts=-(ti(6))/60*2*pi+pi/2;
% 
% hs=plot([0 7*cos(ts)],[0
% 
% 7*sin(ts)],'color','w','linewidth',2);
% 
% set(gcf,'doublebuffer','on');

while 1

ti=clock; %每次读取系统时间，并进行运算

% 计算时针位置

th=-(ti(4)+ti(5)/60+ti(6)/3600)/12*2*pi+pi/2;

xh3=4.0*cos(th);

yh3=4.0*sin(th);

xh2=xh3/2+0.5*cos(th-pi/2);

yh2=yh3/2+0.5*sin(th-pi/2);

xh4=xh3/2-0.5*cos(th-pi/2);

yh4=yh3/2-0.5*sin(th-pi/2);

set(hh,'XData',[0 xh2 xh3 xh4 0],'YData',[0 yh2 yh3 yh4 0])

% 计算分针位置

tm=-(ti(5)+ti(6)/60)/60*2*pi+pi/2;

xm3=6.0*cos(tm);

ym3=6.0*sin(tm);

xm2=xm3/2+0.5*cos(tm-pi/2);

ym2=ym3/2+0.5*sin(tm-pi/2);

xm4=xm3/2-0.5*cos(tm-pi/2);

ym4=ym3/2-0.5*sin(tm-pi/2);

set(hm,'XData',[0 xm2 xm3 xm4 0],'YData',[0 ym2 ym3 ym4 0])

% 计算秒针位置

ts=-(ti(6))/60*2*pi+pi/2;

set(hs,'XData',[0 7*cos(ts)],'YData',[0 7*sin(ts)])

drawnow;

pause(0.09)

end

catch

return

end