% result animation

fv = stlread("parafoil_v2.stl");
P = fv.Points;     %access the vertex data from triangulation              
C = fv.ConnectivityList;     %access the connectivity data from triangulation
for i = 1:10
    P(:,1) = P(:,1) + 500;     %add 5 to each vertex's x value
    a_fv = triangulation(C, P);     %Combine both components back into a triangulation variable
    trisurf(a_fv);
    disp(i)
    sleep(1)
    axis('image');
    view([-135 35]);
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
