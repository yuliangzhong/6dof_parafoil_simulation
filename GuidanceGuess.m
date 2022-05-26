% guidance guess: go straight to the origin
function guidance0 = GuidanceGuess(N, init_pos)
    h = -init_pos(3);
    hs = linspace(h,0,N);
    guidance0 = [zeros(2,N);
                 hs;
                 zeros(2,N)];
%     theta = atan2(0-init_pos(2), 0-init_pos(1));
%     Vh = vel_info(2);
%     Vz = vel_info(3);
%     dt = h/Vz/(N-1);
%     guidance0 = zeros(5,N);
%     guidance0(:,1) = [init_pos(1:2); h; init_rpy(3); 0];
%     for i = 1:N-1
%         guidance0(:,i+1) = guidance0(:,i) + dt*[Vh*cos(theta);
%                                                 Vh*sin(theta);
%                                                 -Vz;
%                                                 0;
%                                                 0];
%     end
end