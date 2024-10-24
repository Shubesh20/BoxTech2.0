%% ASSIGNMENT 2
close all;
clc;
hold on;

% Axis View
axis([-1.5 1.5 -1.5 1.5 0 2])
% view(2);   
view(90, 60);   

%% PLACEMENT OF OBJECTS:
% MAIN ENVIROMENT:
PlaceObject('MAINoENVIRONMENTDraft.ply', [0, 0, 0]);


% Read the image
img = imread('Sign.jpg');

% Create a surface with the concrete texture as a vertical wall
surf([0.968,0.968;0.968,0.968], ...  % X coordinates (width of the wall)
     [0.95,0.95;0.8,0.8], ...            % Z coordinates (set to 0 for the base)
     [1.6,1.4;1.6,1.4], ...  % Y coordinates (height of the wall)
     'CData', img, ...         % Apply the concrete texture
     'FaceColor', 'texturemap');


  
% OBJECT - EMERGENCY BUTTON
h_button = PlaceObject('emergencyStopButton.ply', [3.5, 5, 3.2]);  
verts = [get(h_button,'Vertices'), ones(size(get(h_button,'Vertices'), 1), 1)];
verts(:,1) = verts(:,1) * 0.3; 
verts(:,2) = verts(:,2) * 0.3;                                  
verts(:,3) = verts(:,3) * 0.3;                            
verts = verts * trotx(-pi/2);
verts = verts * trotz(-pi/2);
set(h_button, 'Vertices', verts(:, 1:3));

% OBJECT - FIRE EXTINGUSIHER
PlaceObject('fireExtinguisher.ply', [-0.75, 1, 0]);



% % REFERENCE SHOE BOX
% PlaceObject('Shoebox.ply', [0.88, 0.43, 0.9]);



% ROBOT - LINEAR UR5 SETUP :
% baseTr = transl([-0.35, 0.15, 0.6]) * trotz(-pi/2);                              % Base transform of the robot to be on top of the table 
baseTr = transl([-0.35, 0.15, 0.6]) * trotz(-pi);                              % Base transform of the robot to be on top of the table 
r = LinearUR5(baseTr);                                                          % This creates the robot object
q_initial = r.model.getpos();                                              % This stores the starting joint configuration
r.model.animate(q_initial);      

% ROBOT - OMRON TM5 900 SETUP :
% baseTM = transl([0.4, 0.9, 0.03]) * trotz(pi/2);                             
% baseTM = transl([0, 1, 0.03]) * trotz(pi/2);                            %Initial pickup imrpoved 

% baseTM = transl([0.4, 0.85, 0.03]) * trotz(pi/2);                         % MK1Best version
baseTM = transl([0.4, 0.75, 0.03]) * trotz(pi/2);                         % MK1Best version


f = TM5900(baseTM);                                                          % This creates the robot object
q_initials = f.model.getpos();                                              % This stores the starting joint configuration
f.model.animate(q_initials);


for b = 1:1
    % place brick
    brick{b} = PlaceObject('Shoebox.ply'); 
    % Iterating the position of each brick alongside each other on one side of the robot
    vertices{b} = get(brick{b},'Vertices');
    transformedVertices = [vertices{b},ones(size(vertices{b},1),1)] * transl(0.85,0.9,0.04)';%change to move brick
    set(brick{b},'Vertices',transformedVertices(:,1:3));
end

% for c = 1:1
%     % place brick
%     brick{c} = PlaceObject('Shoebox.ply'); 
%     % Iterating the position of each brick alongside each other on one side of the robot
%     vertices{c} = get(brick{c},'Vertices');
%     transformedVertices = [vertices{c},ones(size(vertices{c},1),1)] * transl(0.85,0.43,0.6)';%change to move brick
%     set(brick{c},'Vertices',transformedVertices(:,1:3));
% end

%% Creating robot grippers (Lab 5 tut)
%function RobotGripper()
fprintf('loading in gripper claw \n');   %Log message

%Reference material from wk4-5 lab/tut
% Define links for gripper 1 (Right hand)
hold on
RGLink1 = Link('d',0,'a',0,'alpha',pi/2,'offset',0); %realign link to correct axis for close grip d/length z, a/length X, alpha, rotation X
RGLink2 = Link('d',0,'a',0.02,'alpha',0,'offset',0); %set link length
RGLink3 = Link('d',0,'a',0.05,'alpha',0,'offset',0);%set link length
GripperClawRight = SerialLink([RGLink1 RGLink2 RGLink3], 'name', 'gripRight'); %serial link for right finger
RHGq = zeros(1,3); %set inital q joint positiosn to 0

% Plot 
GripperClawRight.plot(RHGq); %plot the gripper

% Define links for gripper 2 (Left hand)
hold on
LGLink1 = Link('d',0,'a',0,'alpha',pi/2,'offset',0);%realign link to correct axis for close grip
LGLink2 = Link('d',0,'a',-0.02,'alpha',0,'offset',0);%set link length
LGLink3 = Link('d',0,'a',-0.05,'alpha',0,'offset',0);%set link length
GripperClawLeft = SerialLink([LGLink1 LGLink2 LGLink3], 'name', 'gripLeft');%serial link for left finger
LHGq = zeros(1,3);%set inital q joint positiosn to 0
% Plot
GripperClawLeft.plot(LHGq);%plot the gripper
%end


%% Begin brick placement procedure
%function Bricklay()
fprintf('Initialising brick wall building, robot active \n');   %Log message
steps = 45; %Number of iteration steps
% Loop to simulate the robot picking and placing bricks on the table
for j = 1:1
    % Compute the inverse kinematics for the brick position
  %ikcon - optimisation method and considers joint limits
            if j == 1 %loop for 9 bricks
            [bpos,err,exitflag] = f.model.ikcon(transl(0.72,0.9,0.11) * troty(pi)); %bandaid fix :D ,[-j/10,pi/2,pi/4,pi/4,0,-pi/4,0] 0.85,09,0.12/ mk2 0.72,0.9,0.11

            % else
            % [bpos,err,exitflag] = f.model.ikcon(transl(-j/10,0.5,0.53) * troty(pi),[-j/10,pi/2,pi/4,pi/4,0,pi/4,0]); %change to move brick

            end

    % Finding a trajectory to move the gripper to the brick - quintic polynomial method
    %generate a joint space trajectory between two sets of joint positions
    qMatrix = jtraj(f.model.getpos(),bpos,steps);

    % Gripper closed angles when picking up bricks
    RightGripperq0 = [0,pi/4,pi/6];
    LeftGripperq0 = [0,-pi/4,-pi/6];

    % Loop to animate the gripper movement (from Wk4/5 lab & tut)
    for i = 1:steps
        % animate gripper movement
        f.model.animate(qMatrix(i,:));
        % Set the base of the claw grippers to the robot end-effector
        GripperClawRight.base = f.model.fkine(f.model.getpos);
        GripperClawLeft.base = f.model.fkine(f.model.getpos);
        % animate the claw travelling with the end-effector
        GripperClawRight.animate(GripperClawRight.getpos);
        GripperClawLeft.animate(GripperClawLeft.getpos);
        drawnow();
        pause(0.01);
    end

    % Animate gripper closing
    GripperClawRight.animate(RightGripperq0);
    GripperClawLeft.animate(LeftGripperq0);

    % find the ikine/ikcon of the brick wall for the robot to follow
    %this part is for putting down the bricks
% brickintx = 0.88; %set the base x co-ordinate, changing these values will shift the wall all-together on the table
% brickinty = 0.6;%set the base y co-ordinate
% brickintz = 0.6;%set the base z co-ordinate

brickintx = 0.7; %set the base x co-ordinate, changing these values will shift the wall all-together on the table
brickinty = 0.48;%set the base y co-ordinate
brickintz = 0.68;%set the base z co-ordinate

   switch j
%finding the IKcon for the first brick placement at initial XYZ co-ordinates
    case 1
        fprintf('Brick 1 moving \n') %Log message
        FinalBrickPos = f.model.ikcon(transl(brickintx,brickinty,brickintz) * troty(-pi) * trotz(pi/2)); %,[0,-pi/2,-pi/4,-pi/4,0,-pi/4,0]
   end
    % Generate a trajectory to move the gripper to the placement position
    qMatrix = jtraj(f.model.getpos,FinalBrickPos,steps);

    % Loop to animate the robot movement - From wk4 lab/Wk5 tutorial 
    for i = 1:steps
        % Animate robot movement
        f.model.animate(qMatrix(i,:));
        % Set the base location of the gripper claw to the end effector 
        GripperClawRight.base = f.model.fkine(f.model.getpos);
        GripperClawLeft.base = f.model.fkine(f.model.getpos);
        % Find joint angle of end effector pose using fkine
        clawtr = f.model.fkine(f.model.getpos);
        % Transform and position the brick to the robot endeffector location
        transformedVertices = [vertices{j},ones(size(vertices{j},1),1)] * clawtr.T';
        set(brick{j},'Vertices',transformedVertices(:,1:3));
        drawnow();
        pause(0.01);
    end 

    % Q joints to represent open position of the claw grippers
    openGripperq0 = [0,0,0];
    % Assigning the base of the grippers to the endeffectors location
    GripperClawRight.base = f.model.fkine(f.model.getpos);
    GripperClawLeft.base = f.model.fkine(f.model.getpos);
    %animate grippers opening
    GripperClawRight.animate(openGripperq0);
    GripperClawLeft.animate(openGripperq0);

    fprintf('Joint Angles \n');   %Log message
    f.model.getpos %finding error
    fprintf('Transformation Matrix \n');   %Log message
    f.model.fkine(f.model.getpos) %finding joint angles
    fprintf('Brick Placed \n');   %Log message
end
fprintf('Wall complete, robot stopping \n');   %Log message
