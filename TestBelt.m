clf;
clc;
hold on;

% Axis View
axis([-7 0 -5.5 5.5 0 8])
view(3);    


%% ENVIRONMENT SET UP

% PLACEMENT OF OBJECTS:
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

x = -4.9;
y = 2.7;
z = 0.9;
box_1 = PlaceObject('Shoebox.ply',[x,y,z]);

%% BOX ENTERS SYSTEM

system_error = 0;
steps = 300;

while system_error == 0
    for i = 1:steps
        try delete(box_1); end
        y = y - 0.05;
        PlaceObject('Shoebox.ply',[x,y,z]);
        pause(0.01);
    end

end

 %% PlotSingleRandomStep
%         % Move each of the cows forward and rotate some rotate value around
%         % the z axis
% 
%                 self.boxCount = boxCount;
% 
% 
%             for boxIndex = 1:self.boxCount
%                 % Move Forward
%                 self.boxModel{boxIndex}.base = self.boxModel{boxIndex}.base * SE3(SE2(0.2, 0, 0));
%                 animate(self.boxModel{boxIndex},0);
% 
%                 % Turn randomly
%                 % Save base as a temp variable
%                 tempBase = self.boxModel{boxIndex}.base.T;
%                 rotBase = tempBase(1:3, 1:3);
%                 posBase = tempBase(1:3, 4);
%                 newRotBase = rotBase * troty(0.5);
%                 newBase = [newRotBase posBase ; zeros(1,3) 1];
% 
%                 % Update base pose
%                 self.boxModel{boxIndex}.base = newBase;
%                 animate(self.boxModel{boxIndex},0);                
% 
%                 % If outside workspace rotate back around
%                 % Get base as temp
%                 tempBase = self.boxModel{boxIndex}.base.T;
% 
% 
%             end
%             % Do the drawing once for each interation for speed
%             drawnow();
% 
% %% Get box
% function model = GetBox(name)
%             if nargin < 1
%                 name = 'Box';
%             end
%             [faceData,vertexData] = plyread('Shoebox.ply','tri');
%             link1 = Link('alpha',pi/2,'a',0,'d',0.3,'offset',0);
%             model = SerialLink(link1,'name',name);
% 
%             % Changing order of cell array from {faceData, []} to 
%             % {[], faceData} so that data is attributed to Link 1
%             % in plot3d rather than Link 0 (base).
%             model.faces = {[], faceData};
% 
%             % Changing order of cell array from {vertexData, []} to 
%             % {[], vertexData} so that data is attributed to Link 1
%             % in plot3d rather than Link 0 (base).
%             model.points = {[], vertexData};
%         end