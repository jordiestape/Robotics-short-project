%% Short Project.
% After atending the Laboratory session and Theory classes you must be
% able to answer the following questions.
% Add the necessary matlab and RTB sentences to this script for reporting
% your result. I strongly recoment to use as a reference help for the RTB the file 'robot.pdf'
% http://petercorke.com/wordpress/toolboxes/robotics-toolbox
%% Sketching the enviroment of the robotics work cell.
% It is spected: main reference frames. Plot the robot Puma, draw the
% working table and the torus in working position.
% Give diferent points of view of the scenary: Top, Front, Lateral and
% isometrics view

F1 = [0 2.5 2.5 0; 0 0 0.75 0.75; 0 0 0 0; 1 1 1 1];
F1 = trotx(90)*F1;

F2 = [0 2.5 2.5 0; 0 0 0.6 0.6; 0 0 0 0; 1 1 1 1];
F2 = transl(0,0,0.75) * trotx(20) * F2;

F3 = [0 0.75 0.75 -sin(pi/9)*0.6; 0 0 cos(pi/9)*0.6 cos(pi/9)*0.6; 0 0 0 0; 1 1 1 1];
F3 = transl(0,0,0.75) * troty(90)*F3;

F4 = transl(2.5,0,0) * F3;

F5 = [0 2.5 2.5 0; 0 0 sin(pi/9)*0.6+0.75 sin(pi/9)*0.6+0.75; 0 0 0 0; 1 1 1 1];
F5 = transl(0,0.6*cos(pi/9),0) * trotx(90) * F5;

figure
xlabel('x');
ylabel('y');
zlabel('z');
axis 'equal';
axis([0 2.5 0 2.5 0 2.5]);
fill3(F1(1,:),F1(2,:),F1(3,:),'r',F2(1,:),F2(2,:),F2(3,:),'r', F3(1,:),F3(2,:),F3(3,:),'r', F4(1,:),F4(2,:),F4(3,:),'r', F5(1,:),F5(2,:),F5(3,:),'r');
alpha 0.3

fv=stlread('Torus.stl');% fv is a struct with faces and vertices
fv.vertices=fv.vertices;
ma=max(fv.vertices); mi=min(fv.vertices); dmami=ma-mi;
scale = 0.950/sqrt(dmami(1)*dmami(1) + dmami(2)*dmami(2) + dmami(3)*dmami(3));
fv.vertices = fv.vertices * scale;

fv.vertices = setMatrixTorus(fv.vertices);
fv.vertices = transl(0.85,0.3*cos(pi/9),0.75+sin(pi/9)*0.3) * trotx(-160) * fv.vertices;
fv.vertices = resetMatrixTorus(fv.vertices);

SS=patch(fv,'FaceColor',       [0.8 0.8 1.0], ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');
alpha (SS,0.2)
view(30,30)
% Fix the axes scaling, and set a nice view angle
axis('image');
axis 'equal'

mdl_puma560
p560.plot(qz);
p560
function y = setMatrixTorus(vertices)

[n,~] = size(vertices);
y = [transpose(vertices); ones(1,n)];

end

function y = resetMatrixTorus(vertices)

[m,~] = size(vertices);
vertices(m,:)=[];
y = transpose(vertices);

end

%% Working points.
% Give here your code to get the variables to locate:
% a) The reference frame for all drills holes, such that z-axis is orthogonal
% to the surface of the torus and the x-axis is in the direction of minimun
% curvature. Draw in scale the frames
% b) Repeat the obove operation for the center of the milling groove. Draw
% this frames.
% c) The reference frames for all welding points, such that z-axis of the tool 
% is orthogonal to the surface of the torus and the x-axis is in the direction of 
% spiral trajectory. Draw in scale the frames
%% Computing motor torques for the static forces.
% Give here your code to fill two tables with the motor torque at each robot pose:
% Table 1 (6x16): Rows are the motor torques (6x1). Columns (1x16) are the labeled drills
% including the initial drill before milling.
% Table 2 (6x8): Rows are the motor torques (6x1). Columns (1x8) are the
% labeled milling of the groove.
