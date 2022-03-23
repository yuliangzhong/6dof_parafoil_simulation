% result animation
% https://www.youtube.com/watch?v=dLyBejMmHqE
% https://medium.com/geekculture/3d-animations-made-simple-with-matlab-visualizing-flight-test-data-and-simulation-results-ed399cdcc711

fv = stlread("parafoil_v2.stl");
P = fv.Points / 1000;     % [mm] -> [m]       
C = fv.ConnectivityList;     %access the connectivity data from triangulation
fv = triangulation(C, P);

hold on;
axis('image');
view([-135 35]);

for i = 1 : 100
    x = [i*0.1; 0; 0];
    scatter3(x(1),x(2),x(3),'filled');
    P(:,1) = P(:,1) + i*0.1;
    a_fv = triangulation(C, P);
    model = trisurf(a_fv);
    pause(0.01);
    delete model;
end



% patch(parafoil_model, 'FaceColor',       [0.8 0.8 1.0], ...
%          'EdgeColor',       'none',        ...
%          'FaceLighting',    'gouraud',     ...
%          'AmbientStrength', 0.15);
% 
% % Add a camera light, and tone down the specular highlighting
% camlight('headlight');
% material('dull');

% Fix the axes scaling, and set a nice view angle
