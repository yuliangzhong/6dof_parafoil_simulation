%% fitting --> fx, fy, wx, wy
guidance = out.guidance.signals.values(:,:,end);
hold on
grid on
plot(guidance(2,:), guidance(1,:))

N = time_horizon_N;
h_end = guidance(3,end);
ge = [interp1(guidance(3,:),guidance(1,:),linspace(h_end,h_end-1,2*N),'linear','extrap');
      interp1(guidance(3,:),guidance(2,:),linspace(h_end,h_end-1,2*N),'linear','extrap');
      linspace(h_end,h_end-5,2*N);
      zeros(2,2*N)];
big_guidance = [guidance,ge];

id = 1700;
ids = max(id-50,1);
idn = id+2*N;

% fx
px = polyfit(big_guidance(3,ids:idn), big_guidance(1,ids:idn), 7);
fx = @(x) px*[x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
dfx = @(x) px*[7*x.^6; 6*x.^5; 5*x.^4; 4*x.^3; 3*x.^2; 2*x; ones(1,size(x,2)); zeros(1,size(x,2))];

% fy
py = polyfit(big_guidance(3,ids:idn), big_guidance(2,ids:idn), 7);
fy = @(x) py*[x.^7; x.^6; x.^5; x.^4; x.^3; x.^2; x; ones(1,size(x,2))];
dfy = @(x) py*[7*x.^6; 6*x.^5; 5*x.^4; 4*x.^3; 3*x.^2; 2*x; ones(1,size(x,2)); zeros(1,size(x,2))];


% plot(fy(linspace(0,50,50)), fx(linspace(0,50,50)))