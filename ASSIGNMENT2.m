%% ASSIGNMENT 2  
clf;  
clc;  
hold on;  
  
% Axis View  
axis([-1 1.5 -1 1.5 0 2])     
view(-140, 25);    

%% PLACEMENT OF OBJECTS:  
% MAIN ENVIROMENT:  
PlaceObject('Environment.ply', [0, 0, 0]);  
  
% Read the image  
img = imread('Sign.jpg');  
  
% Create a surface with the warning sign on the vertical wall  
surf([0.968,0.968;0.968,0.968], ...                                          % X coordinates (width of the wall)  
    [0.95,0.95;0.8,0.8], ...                                                 % Z coordinates (set to 0 for the base)  
    [1.6,1.4;1.6,1.4], ...                                                   % Y coordinates (height of the wall)  
    'CData', img, ...                                                        % Apply the sign on the face of the SBT
    'FaceColor', 'texturemap');  
  
% OBJECT - EMERGENCY BUTTON
h_button = PlaceObject('emergencyStopButton.ply', [3.5, 5, 3.2]);            % Creates the E stop button
verts = [get(h_button,'Vertices'), ones(size(get(h_button,'Vertices'), 1), 1)];  
verts(:,1) = verts(:,1) * 0.3;  
verts(:,2) = verts(:,2) * 0.3;                        
verts(:,3) = verts(:,3) * 0.3;                    
verts = verts * trotx(-pi/2);  
verts = verts * trotz(-pi/2);  
set(h_button, 'Vertices', verts(:, 1:3));  
  
% OBJECT - FIRE EXTINGUISHER  
PlaceObject('fireExtinguisher.ply', [-0.75, 1, 0]);  
  
% ROBOT - LINEAR UR5 SETUP:  
baseTr = transl([-0.35, 0.15, 0.6]) * trotz(-pi);                            % Base transform of the robot to be on top of the table  
r = LinearUR5(baseTr);                                                       % Creates the robot object  
q_initial = r.model.getpos();                                                % Store the starting joint configuration of the Linear UR5 
r.model.animate(q_initial);  
global models                                                                % Global variable to access robot in GUI
models = r;
  
% ROBOT - OMRON TM5 900 SETUP:  
baseTM = transl([0.4, 0.8, 0.043]) * trotz(pi/2);                            % Base transform for TM5-900  
f = TM5900(baseTM);                                                          % Creates the robot object  
q_initial_TM5 = f.model.getpos();                                            % Store the starting joint configuration of the TM5-900  
f.model.animate(q_initial_TM5);
global model                                                                 % Global variable to access robot in GUI
model = f;

Guitest                                                                      % Load in GUI


%% Integrated Loop  

% Initialize E-stop state  
global estop_engaged  
estop_engaged = 0;  

%%%%%%%%%%%%%%%%%%%%%%%%% COLLISION DETECTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Comment out to turn off

obstacle_ply = plyread('Environment.ply');                                                   % Load the obstacle ply file 
obstacle_vertices = [obstacle_ply.vertex.x, obstacle_ply.vertex.y, obstacle_ply.vertex.z];   % Get the vertices of the obstacle  

% OR
% obstacle_vertices = obstacle_ply.vertices; %This depends on the ply file


%%%%%%%%%%%%%%%%%%%%%%%%% COLLISION DETECTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i = 1:12  
   % Check if E-stop is engaged  
   global estop_engaged  
   global resume_simulation   

   % Place and position a shoe box  
   box{i} = PlaceObject('Shoebox.ply');  
   vertices{i} = get(box{i},'Vertices');  
   transformedVertices = [vertices{i},ones(size(vertices{i},1),1)] * transl(0.85,0.9,0.04)';  % Moves the shoe boxes 
   set(box{i},'Vertices',transformedVertices(:,1:3));  
  
% OMRON TM5 900 MOVEMENT

   % TM5900 picks up and places a box in designated location  
   % Compute the inverse kinematics for the box position  
   [bpos,err,exitflag] = f.model.ikcon(transl(0.72,0.9,0.11) * troty(pi));    % Position to move to  
   qMatrix = jtraj(f.model.getpos(), bpos, 45);                               % Trajectory to move the robot to the box position  
  
   % Animate robot movement  
   for j = 1:45       
      eStopStatus();                                                          % Check if E-stop is engaged 
      try  
        f.model.animate(qMatrix(j,:));  
      catch  
        % If animation fails, skip this step  
        continue;  
      end  
      drawnow();  
      pause(0.01);  
                                                                              %%%%%%%%%%%%%%%%%%%%%% COLLISION DETECTION %%%%%%%%%%%%%%%%%%%%%%%%%%
      OmronCheckCollision(f, obstacle_vertices);                              %%%%%%%%%%%%%%%%%%%%%% COLLISION DETECTION %%%%%%%%%%%%%%%%%%%%%%%%%%
   end  
  
   % Final positions for placing the box  
   boxntx = 0.7;  
   boxnty = 0.48;  
   boxntz = 0.68;  
   FinalBoxPos = f.model.ikcon(transl(boxntx,boxnty,boxntz) * troty(-pi) * trotz(pi/2));  

   qMatrix = jtraj(f.model.getpos, FinalBoxPos, 45);                        % Trajectory to move the box to the final position  
  
   % Animate placing the box  
   for j = 1:45  
      eStopStatus();                                                          % Check if E-stop is engaged   
      try  
        f.model.animate(qMatrix(j,:));  
      catch  
        % If animation fails, skip this step  
        continue;  
      end  
      clawtr = f.model.fkine(f.model.getpos);                                 % Calculates the forward kinematics of the robot arm
      transformedVertices = [vertices{i},ones(size(vertices{i},1),1)] * clawtr.T';   % Transforms the vertices of the shoe box to the current pose of the robot arm
      set(box{i},'Vertices',transformedVertices(:,1:3));                      % Updates the shoe box vertices
      drawnow();  
      pause(0.01);  
   end  
  
   % Return TM5900 to original position  
   qMatrix = jtraj(f.model.getpos, q_initial_TM5, 45);  
   for j = 1:45    
      eStopStatus();                                                          % Check if E-stop is engaged  
      try  
        f.model.animate(qMatrix(j,:));  
      catch  
        % If animation fails, skip this step  
        continue;  
      end  
      drawnow();  
      pause(0.01);  
   end  
   
   pause(1);                                                                  % short break before LinearUR5 starts moving  
  

% LINEAR UR5 MOVEMENT - Linear UR5 picks up the box from designated location and places it into the shelf
   [bpos,err,exitflag] = r.model.ikcon(transl(0.73,0.5,0.75) * troty(pi) * trotz(pi/2));  % Compute the inverse kinematics for the box position to move to  
   qMatrix = jtraj(r.model.getpos(), bpos, 45);                               % Trajectory to move the robot to the box position  
  
   % Animate UR5 movements  
   for j = 1:45     
      eStopStatus();                                                          % Check if E-stop is engaged 
      try  
        r.model.animate(qMatrix(j,:));  
      catch  
        % If animation fails, skip this step  
        continue;  
      end  
      drawnow();  
      pause(0.01);  
                                                                              %%%%%%%%%%%%%%%%%%%%%% COLLISION DETECTION %%%%%%%%%%%%%%%%%%%%%%%%%%
      UR5CheckCollision(r, obstacle_vertices);                                %%%%%%%%%%%%%%%%%%%%%% COLLISION DETECTION %%%%%%%%%%%%%%%%%%%%%%%%%% 
   end  
  
   % Final positions for placing the box  
   boxntx = -0.5;  
   boxnty = -0.25;  
   boxntz = 0.95;  
  
   % Cases are used to calculate final destinations of each shoe box on the designated shelves
   switch i  
      case 1  
        FinalBoxPos = r.model.ikcon(transl(boxntx,boxnty,boxntz) * troty(-pi) * trotz(pi/2),[0,-pi/2,-pi/4,-pi/4,0,-pi/4,0]);  
      case 2  
        FinalBoxPos = r.model.ikcon(transl((boxntx + 0.27),boxnty,boxntz) * troty(-pi) * trotz(pi/2),[0,3*pi/2,0,-pi/2,0,-3*pi/2,0]);  
      case 3  
        FinalBoxPos = r.model.ikcon(transl((boxntx + 0.54),boxnty,boxntz) * troty(-pi) * trotz(pi/2),[0,3*pi/2,0,-pi/2,0,-3*pi/2,0]);  
      case 4  
        FinalBoxPos = r.model.ikcon(transl((boxntx + 0.81),boxnty,boxntz) * troty(-pi) * trotz(pi/2),[0,3*pi/2,0,-pi/2,0,-3*pi/2,0]);  
      case 5  
        FinalBoxPos = r.model.ikcon(transl((boxntx + 1.07),boxnty,boxntz) * troty(-pi) * trotz(pi/2),[0,-pi/2,-pi/4,-pi/4,0,-pi/4,0]);  
      case 6  
        FinalBoxPos = r.model.ikcon(transl((boxntx + 1.32),boxnty,boxntz) * troty(-pi) * trotz(pi/2),[0,-pi/2,-pi/4,-pi/4,0,-pi/4,0]);  
      case 7  
        FinalBoxPos = r.model.ikcon(transl(boxntx,boxnty,(boxntz + 0.41)) * troty(-pi) * trotz(pi/2),[0,-pi/2,-pi/4,-pi/4,0,-pi/4,0]);  
      case 8  
        FinalBoxPos = r.model.ikcon(transl((boxntx + 0.27),boxnty,(boxntz + 0.41)) * troty(-pi) * trotz(pi/2),[0,-pi/2,-pi/4,-pi/4,0,-pi/4,0]);  
      case 9  
        FinalBoxPos = r.model.ikcon(transl((boxntx + 0.54),boxnty,(boxntz + 0.41)) * troty(-pi) * trotz(pi/2),[0,-pi/2,-pi/4,-pi/4,0,-pi/4,0]);  
      case 10  
        FinalBoxPos = r.model.ikcon(transl((boxntx + 0.81),boxnty,(boxntz + 0.41)) * troty(-pi) * trotz(pi/2),[0,3*pi/2,0,-pi/2,0,-3*pi/2,0]);  
      case 11  
        FinalBoxPos = r.model.ikcon(transl((boxntx + 1.07),boxnty,(boxntz + 0.41)) * troty(-pi) * trotz(pi/2),[0,3*pi/2,0,-pi/2,0,-3*pi/2,0]);  
      case 12  
        FinalBoxPos = r.model.ikcon(transl((boxntx + 1.32),boxnty,(boxntz + 0.41)) * troty(-pi) * trotz(pi/2),[0,3*pi/2,0,-pi/2,0,-3*pi/2,0]);  
   end  
  
 
   qMatrix = jtraj(r.model.getpos, FinalBoxPos, 45);                        % Generate a trajectory to move the gripper to the placement position   
  
   % Animate placing the box  
   for j = 1:45     
      eStopStatus();                                                          % Check if E-stop is engaged 
      try  
        r.model.animate(qMatrix(j,:));  
      catch  
        % If animation fails, skip this step  
        continue;  
      end  
      clawtr = r.model.fkine(r.model.getpos);                                 % Calculates the forward kinematics of the robot arm
      transformedVertices = [vertices{i},ones(size(vertices{i},1),1)] * clawtr.T';  % Transforms the vertices of the shoe box to the current pose of the robot arm
      set(box{i},'Vertices',transformedVertices(:,1:3));                      % Updates the shoe box vertices
      drawnow();  
      pause(0.01);  
                                                                              %%%%%%%%%%%%%%%%%%%%%% COLLISION DTECTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%
      UR5CheckCollision(r, obstacle_vertices);                                %%%%%%%%%%%%%%%%%%%%%% COLLISION DTECTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%
     

   end  
  
   % Returns the Linear UR5 to its original position  
   qMatrix = jtraj(r.model.getpos, q_initial, 45);  
   for j = 1:45         
      eStopStatus();                                                          % Check if E-stop is engaged 
      try  
        r.model.animate(qMatrix(j,:));  
      catch  
        % If animation fails, skip this step  
        continue;  
      end  
      drawnow();  
      pause(0.01);  

                                                                              %%%%%%%%%%%%%%%%%%%%%% COLLISION DTECTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%
      UR5CheckCollision(r, obstacle_vertices);                                %%%%%%%%%%%%%%%%%%%%%% COLLISION DTECTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%
      

   end  
end





%% E-STOP FUNCTION

function eStopStatus()
    global estop_engaged
    global resume_simulation
    while (estop_engaged ~= 0) | (resume_simulation == 0)                     % check if E-stop toggle is engaged and start button has been pressed
        pause(0.1);
    end
end


%% COLLISION DETECTION FUNCTIONS - COMMENT TO TURN OFF 

% OMRON TM5 900
function OmronCheckCollision(f, obstacle_vertices)     
  tr = f.model.fkine(f.model.getpos).T;                                       % Get the current pose of the robot   
    
  % Loop through each vertex of the obstacle   
  for i = 1:size(obstacle_vertices, 1)         
    dist = sqrt(sum((obstacle_vertices(i, :) - tr(1:3, 4)').^2));             % Calculate the distance between the end effector and the vertex 
      
    % Check if the distance is less than a certain threshold (e.g. 0.1m)   
    if dist <= 0.15  
      disp('OMRON - Collision detected!');      

      % pauseSimulation();                                                    % Pauses the simulation : UNCOMMENT TO ACTIVATE PAUSING IN A COLLISION
    
    end   
  end   
end

% Linear UR5
function UR5CheckCollision(r, obstacle_vertices)   
  % Get the current pose of the robot   
  tr = r.model.fkine(r.model.getpos).T;   
    
  % Loop through each vertex of the obstacle   
  for i = 1:size(obstacle_vertices, 1)    
    dist = sqrt(sum((obstacle_vertices(i, :) - tr(1:3, 4)').^2));             % Calculate the distance between the end effector and the vertex    
      
    % Check if the distance is less than a certain threshold (e.g. 0.1m)   
    if dist <= 0.2618203 
      disp('LINEAR UR5 - Collision detected!');  

      % pauseSimulation();                                                    % Pauses the simulation : UNCOMMENT TO ACTIVATE PAUSING IN A COLLISION

    end   
  end   
end



%% ATTEMPT FOR COLLISION AVOIDANCE - OMRON TM5 900

% function OmronCheckCollision(f, obstacle_vertices)
% tr = f.model.fkine(f.model.getpos).T;
% 
% for i = 1:size(obstacle_vertices, 1)
%     dist = sqrt(sum((obstacle_vertices(i, :) - tr(1:3, 4)').^2));             % Calculate the distance between the end effector and the vertex
% 
%     % Check if the distance is less than a certain threshold (e.g. 0.1m)
%     if dist <= 0.15
%         disp('OMRON - Collision detected!');
%         [bpos,err,exitflag] = f.model.ikcon(transl(0.72,0.9,0.11) * troty(pi));
%         qMatrix = jtraj(f.model.getpos(), bpos, 45);
% 
%         % Animate robot movement
%             for j = 1:45
%                 eStopStatus();                                                % Check if E-stop is engaged
%                 try
%                     f.model.animate(qMatrix(j,:));
%                 catch
%                     % If animation fails, skip this step
%                     continue;
%                 end
%                 drawnow();
%                 pause(0.01);
%             end
%         end
%     end
% end


% Collision detection - PAUSES AFTER FIRST COLLSION
function pauseSimulation()  
  global resume_simulation  
  resume_simulation = 0;  
  while resume_simulation == 0  
   pause(0.1);  
  end  
end