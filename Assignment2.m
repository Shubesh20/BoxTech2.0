%% ASSIGNMENT 2
% % Environment Setup
% close all;
% clc;
% hold on;
% 
% % Axis View
% axis([-3 3 -3 3 0 2])
% view(3);                                                                    
% 
% 
% % PLACEMENT OF OBJECTS:
% % OBJECT - FLOOR:
% surf([-2.5,-2.5;2.5,2.5] ...
% ,[-2.5,2.5;-2.5,2.5] ...
% ,[0.01,0.01;0.01,0.01] ...
% ,'CData',imread('concrete.jpg') ...
% ,'FaceColor','texturemap');
% 
% % OBJECT - TABLE
% PlaceObject('tableBrown2.1x1.4x0.5m.ply', [0, 0, 0]);  
% 
% % OBJECT - PLACEMENT OF BARRIERS
% h_1 = PlaceObject('barrier1.5x0.2x1m.ply',[0,2,0]);
% verts = [get(h_1,'Vertices'), ones(size(get(h_1,'Vertices'),1),1)];
% verts(:,1) = verts(:,1) * 2.5; %X axis
% verts(:,3) = verts(:,3) * 1.5; %Z axis
% set(h_1,'Vertices',verts(:,1:3)) %Sets it were to be placed
% 
% h_2 = PlaceObject('barrier1.5x0.2x1m.ply',[0,-2,0]);
% verts = [get(h_2,'Vertices'), ones(size(get(h_2,'Vertices'),1),1)];
% verts(:,1) = verts(:,1) * 2.5;                                  
% verts(:,3) = verts(:,3) * 1.5;                                  
% set(h_2,'Vertices',verts(:,1:3))
% 
% h_3 = PlaceObject('barrier1.5x0.2x1m.ply',[0,0,0]);              
% verts = [get(h_3,'Vertices'), ones(size(get(h_3,'Vertices'),1),1)] * trotz(pi/2);
% verts(:,2) = verts(:,2) * 2.5;                                  
% verts(:,3) = verts(:,3) * 1.5;                                 
% verts(:,1) = verts(:,1) + 2;                                    
% set(h_3,'Vertices',verts(:,1:3))
% 
% h_4 = PlaceObject('barrier1.5x0.2x1m.ply',[0,0,0]);             
% verts = [get(h_4,'Vertices'), ones(size(get(h_4,'Vertices'),1),1)] * trotz(pi/2);
% verts(:,2) = verts(:,2) * 2.5;                                  
% verts(:,3) = verts(:,3) * 1.5;                                  
% verts(:,1) = verts(:,1) - 2;                                    
% set(h_4,'Vertices',verts(:,1:3))
% 
% % OBJECT - FIRE EXTINGUSIHER
% PlaceObject('fireExtinguisherElevated.ply', [2.2, -2.2, 0.5]);
% 
% % OBJECT - EMERGENCY BUTTON
% h_button = PlaceObject('emergencyStopButton.ply', [1.05, 1.00, 2.00]);  
% verts = [get(h_button,'Vertices'), ones(size(get(h_button,'Vertices'), 1), 1)];
% verts = verts * trotx(-pi/2);
% set(h_button, 'Vertices', verts(:, 1:3));
% 
% 
% % ROBOT - LINEAR UR3E SETUP :
% baseTr = transl([0.4, 0, 0.5]) * trotz(pi/2);                              % Base transform of the robot to be on top of the table 
% r = UR3e(baseTr);                                                          % This creates the robot object
% q_initial = r.model.getpos();                                              % This stores the starting joint configuration
% r.model.animate(q_initial);                                                % Animates the robot
% 
% 
% 



close all;
clc;
hold on;

% Axis View
axis([-7 0 -5.5 5.5 0 8])
view(3);      

% % PLACEMENT OF OBJECTS:
% OBJECT - FLOOR:
surf([-7,-7;7,7] ...
,[-7,7;-7,7] ...
,[0.01,0.01;0.01,0.01] ...
,'CData',imread('concrete.jpg') ...
,'FaceColor','texturemap');

% OBJECT - TABLE
% PlaceObject('ConveyerbeltCOLOUR.ply', [0, 0, 0]);  

 
c_1 = PlaceObject('CBthinnerCOLOUR.ply',[4,-5,0]);              
verts = [get(c_1,'Vertices'), ones(size(get(c_1,'Vertices'),1),1)] * trotz(pi/2);
% verts(:,2) = verts(:,2) * 2.5;                                  
% verts(:,3) = verts(:,3) * 1.5;                                 
% verts(:,1) = verts(:,1) + 2;                                    
set(c_1,'Vertices',verts(:,1:3))




PlaceObject('emergencyStopButton.ply', [0, 0, 5]); 
PlaceObject('Platform3.ply', [3, 0, 0]); 
PlaceObject('TableBrown2.ply', [-3.9, 0, 0]);  



% 
% PlaceObject('TM5900Link0.ply', [0, 0, 3]); 
% PlaceObject('TM5900Link1.ply', [0, 0, 3]); 
% PlaceObject('TM5900Link2.ply', [0, 0, 3]); 
% PlaceObject('TM5900Link3.ply', [0, 0, 3]); 
% PlaceObject('TM5900Link4.ply', [0, 0, 3]); 
% PlaceObject('TM5900Link5.ply', [0, 0, 3]); 
% PlaceObject('TM5900Link6.ply', [0, 0, 3]); 









% ROBOT - LINEAR UR10 SETUP :
baseTr = transl([-2, 0, 1.1]) * trotz(pi/2);                              % Base transform of the robot to be on top of the table 
r = LinearUR10(baseTr);                                                          % This creates the robot object
q_initial = r.model.getpos();                                              % This stores the starting joint configuration
r.model.animate(q_initial);      