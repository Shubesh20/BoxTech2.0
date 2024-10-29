%% ASSIGNMENT 2
clf;
clc;
clear;
hold on;

%% PLACEMENT OF OBJECTS:

PlaceObject('Environment.ply', [0, 0, 0]);  

% ROBOT - LINEAR UR5 SETUP :
% baseTr = transl([-0.35, 0.15, 0.6]) * trotz(-pi/2);                              % Base transform of the robot to be on top of the table 
baseTr = transl([-0.35, 0.15, 0.6]) * trotz(-pi);                              % Base transform of the robot to be on top of the table 
r = LinearUR5(baseTr);                                                          % This creates the robot object
q_initial = r.model.getpos();                                              % This stores the starting joint configuration
r.model.animate(q_initial);      

% % ROBOT - OMRON TM5 900 SETUP :
% baseTM = transl([0.4, 0.9, 0.03]) * trotz(pi/2);                              % Base transform of the robot to be on top of the table 
% f = TM5900(baseTM);                                                          % This creates the robot object
% q_initials = f.model.getpos();                                              % This stores the starting joint configuration
% f.model.animate(q_initials); 

for b = 1:1
    % place brick
    brick{b} = PlaceObject('Shoebox.ply'); 
    % Iterating the position of each brick alongside each other on one side of the robot
    vertices{b} = get(brick{b},'Vertices');
    transformedVertices = [vertices{b},ones(size(vertices{b},1),1)] * transl(0.85,0.9,0.04)';%change to move brick
    set(brick{b},'Vertices',transformedVertices(:,1:3));
end

for c = 1:12
    % place brick
    brick{c} = PlaceObject('Shoebox.ply'); 
    % Iterating the position of each brick alongside each other on one side of the robot
    vertices{c} = get(brick{c},'Vertices');
    transformedVertices = [vertices{c},ones(size(vertices{c},1),1)] * trotz(pi/2) * transl(0.7,0.35,0.6)';%change to move brick
    set(brick{c},'Vertices',transformedVertices(:,1:3));
end

 %% Creating robot grippers (Lab 5 tut)
% %function RobotGripper()
% fprintf('loading in gripper claw \n');   %Log message
% 
% %Reference material from wk4-5 lab/tut
% % Define links for gripper 1 (Right hand)
% hold on
% RGLink1 = Link('d',0,'a',0,'alpha',pi/2,'offset',0); %realign link to correct axis for close grip d/length z, a/length X, alpha, rotation X
% RGLink2 = Link('d',0,'a',0.02,'alpha',0,'offset',0); %set link length
% RGLink3 = Link('d',0,'a',0.05,'alpha',0,'offset',0);%set link length
% GripperClawRight = SerialLink([RGLink1 RGLink2 RGLink3], 'name', 'gripRight'); %serial link for right finger
% RHGq = zeros(1,3); %set inital q joint positiosn to 0
% 
% % Plot 
% GripperClawRight.plot(RHGq); %plot the gripper
% 
% % Define links for gripper 2 (Left hand)
% hold on
% LGLink1 = Link('d',0,'a',0,'alpha',pi/2,'offset',0);%realign link to correct axis for close grip
% LGLink2 = Link('d',0,'a',-0.02,'alpha',0,'offset',0);%set link length
% LGLink3 = Link('d',0,'a',-0.05,'alpha',0,'offset',0);%set link length
% GripperClawLeft = SerialLink([LGLink1 LGLink2 LGLink3], 'name', 'gripLeft');%serial link for left finger
% LHGq = zeros(1,3);%set inital q joint positiosn to 0
% % Plot
% GripperClawLeft.plot(LHGq);%plot the gripper
% %end
% 
% 
% %% Begin brick placement procedure
% %function Bricklay()
% fprintf('Initialising brick wall building, robot active \n');   %Log message
% steps = 45; %Number of iteration steps
% % Loop to simulate the robot picking and placing bricks on the table
% for j = 1:1
%     % Compute the inverse kinematics for the brick position
%   %ikcon - optimisation method and considers joint limits
%             if j == 1 %loop for 9 bricks
%             [bpos,err,exitflag] = f.model.ikcon(transl(0.85,0.9,0.04) * troty(pi)); %bandaid fix :D ,[-j/10,pi/2,pi/4,pi/4,0,-pi/4,0]
% 
%             % else
%             % [bpos,err,exitflag] = f.model.ikcon(transl(-j/10,0.5,0.53) * troty(pi),[-j/10,pi/2,pi/4,pi/4,0,pi/4,0]); %change to move brick
% 
%             end
% 
%     % Finding a trajectory to move the gripper to the brick - quintic polynomial method
%     %generate a joint space trajectory between two sets of joint positions
%     qMatrix = jtraj(f.model.getpos(),bpos,steps);
% 
%     % Gripper closed angles when picking up bricks
%     RightGripperq0 = [0,pi/4,pi/6];
%     LeftGripperq0 = [0,-pi/4,-pi/6];
% 
%     % Loop to animate the gripper movement (from Wk4/5 lab & tut)
%     for i = 1:steps
%         % animate gripper movement
%         f.model.animate(qMatrix(i,:));
%         % Set the base of the claw grippers to the robot end-effector
%         GripperClawRight.base = f.model.fkine(f.model.getpos);
%         GripperClawLeft.base = f.model.fkine(f.model.getpos);
%         % animate the claw travelling with the end-effector
%         GripperClawRight.animate(GripperClawRight.getpos);
%         GripperClawLeft.animate(GripperClawLeft.getpos);
%         drawnow();
%         pause(0.01);
%     end
% 
%     % Animate gripper closing
%     GripperClawRight.animate(RightGripperq0);
%     GripperClawLeft.animate(LeftGripperq0);
% 
%     % find the ikine/ikcon of the brick wall for the robot to follow
%     %this part is for putting down the bricks
% brickintx = 0.88; %set the base x co-ordinate, changing these values will shift the wall all-together on the table
% brickinty = 0.43;%set the base y co-ordinate
% brickintz = 0.9;%set the base z co-ordinate
% 
%    switch j
% %finding the IKcon for the first brick placement at initial XYZ co-ordinates
%     case 1
%         fprintf('Brick 1 moving \n') %Log message
%         FinalBrickPos = f.model.ikcon(transl(brickintx,brickinty,brickintz) * troty(-pi) * trotz(pi/2)); %,[0,-pi/2,-pi/4,-pi/4,0,-pi/4,0]
%    end
%     % Generate a trajectory to move the gripper to the placement position
%     qMatrix = jtraj(f.model.getpos,FinalBrickPos,steps);
% 
%     % Loop to animate the robot movement - From wk4 lab/Wk5 tutorial 
%     for i = 1:steps
%         % Animate robot movement
%         f.model.animate(qMatrix(i,:));
%         % Set the base location of the gripper claw to the end effector 
%         GripperClawRight.base = f.model.fkine(f.model.getpos);
%         GripperClawLeft.base = f.model.fkine(f.model.getpos);
%         % Find joint angle of end effector pose using fkine
%         clawtr = f.model.fkine(f.model.getpos);
%         % Transform and position the brick to the robot endeffector location
%         transformedVertices = [vertices{j},ones(size(vertices{j},1),1)] * clawtr.T';
%         set(brick{j},'Vertices',transformedVertices(:,1:3));
%         drawnow();
%         pause(0.01);
%     end 
% 
%     % Q joints to represent open position of the claw grippers
%     openGripperq0 = [0,0,0];
%     % Assigning the base of the grippers to the endeffectors location
%     GripperClawRight.base = f.model.fkine(f.model.getpos);
%     GripperClawLeft.base = f.model.fkine(f.model.getpos);
%     %animate grippers opening
%     GripperClawRight.animate(openGripperq0);
%     GripperClawLeft.animate(openGripperq0);
% 
%     fprintf('Joint Angles \n');   %Log message
%     f.model.getpos %finding error
%     fprintf('Transformation Matrix \n');   %Log message
%     f.model.fkine(f.model.getpos) %finding joint angles
%     fprintf('Brick Placed \n');   %Log message
% end
% fprintf('Wall complete, robot stopping \n');   %Log message

%% UR10 Code

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
for j = 1:12
    % Compute the inverse kinematics for the brick position
  %ikcon - optimisation method and considers joint limits
            if j == 12 %loop for 9 bricks
            [bpos,err,exitflag] = r.model.ikcon(transl(0.73,0.5,0.75) * troty(pi) * trotz(pi/2)); %bandaid fix :D ,[-j/10,pi/2,pi/4,pi/4,0,-pi/4,0], (0.7,0.47,0.75)

            else
            [bpos,err,exitflag] = r.model.ikcon(transl(0.73,0.5,0.75) * troty(pi) * trotz(pi/2)); %change to move brick ,[-j/10,pi/2,pi/4,pi/4,0,pi/4,0], (0.7,0.47,0.75)

            end

    % Finding a trajectory to move the gripper to the brick - quintic polynomial method
    %generate a joint space trajectory between two sets of joint positions
    qMatrix = jtraj(r.model.getpos(),bpos,steps);
    
    % Gripper closed angles when picking up bricks
    RightGripperq0 = [0,pi/4,pi/6];
    LeftGripperq0 = [0,-pi/4,-pi/6];
    
    % Loop to animate the gripper movement (from Wk4/5 lab & tut)
    for i = 1:steps
        % animate gripper movement
        r.model.animate(qMatrix(i,:));
        % Set the base of the claw grippers to the robot end-effector
        GripperClawRight.base = r.model.fkine(r.model.getpos);
        GripperClawLeft.base = r.model.fkine(r.model.getpos);
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
brickintx = -0.5; %set the base x co-ordinate, changing these values will shift the wall all-together on the table
brickinty = -0.25;%set the base y co-ordinate
brickintz = 0.95;%set the base z co-ordinate

   switch j
%finding the IKcon for the first brick placement at initial XYZ co-ordinates
    case 1
        fprintf('Brick 1 moving \n') %Log message
        FinalBrickPos = r.model.ikcon(transl(brickintx,brickinty,brickintz) * troty(-pi) * trotz(pi/2),[0,-pi/2,-pi/4,-pi/4,0,-pi/4,0]); 
        
%finding the IKcon for the second brick placement, and iterating XYZ co-ordinates
       case 2
        fprintf('Brick 2 moving \n')%Log message
        FinalBrickPos = r.model.ikcon(transl((brickintx + 0.27),brickinty,brickintz) * troty(-pi) * trotz(pi/2),[0,3*pi/2,0,-pi/2,0,-3*pi/2,0]); %
         
%finding the IKcon for the third brick placement, and iterating XYZ co-ordinates
       case 3
        fprintf('Brick 3 moving \n')%Log message
        FinalBrickPos = r.model.ikcon(transl((brickintx + 0.54),brickinty,brickintz) * troty(-pi) * trotz(pi/2),[0,3*pi/2,0,-pi/2,0,-3*pi/2,0]); %
        
%finding the IKcon for the fourth brick placement, and iterating XYZ co-ordinates
    case 4
        fprintf('Brick 4 moving \n')%Log message
        FinalBrickPos = r.model.ikcon(transl((brickintx + 0.81),brickinty,brickintz) * troty(-pi) * trotz(pi/2),[0,3*pi/2,0,-pi/2,0,-3*pi/2,0]);
        
%finding the IKcon for the fifth brick placement, and iterating XYZ co-ordinates
    case 5
        fprintf('Brick 5 moving \n')%Log message
        FinalBrickPos = r.model.ikcon(transl((brickintx + 1.07),brickinty,brickintz) * troty(-pi) * trotz(pi/2),[0,-pi/2,-pi/4,-pi/4,0,-pi/4,0]);
        
%finding the IKcon for the sixth brick placement, and iterating XYZ co-ordinates
    case 6
        fprintf('Brick 6 moving \n')%Log message
        FinalBrickPos = r.model.ikcon(transl((brickintx + 1.32),brickinty,brickintz) * troty(-pi) * trotz(pi/2),[0,-pi/2,-pi/4,-pi/4,0,-pi/4,0]);
        
%finding the IKcon for the seventh brick placement, and iterating XYZ co-ordinates
    case 7
        fprintf('Brick 7 moving \n')%Log message
        FinalBrickPos = r.model.ikcon(transl(brickintx,brickinty,(brickintz + 0.41)) * troty(-pi) * trotz(pi/2),[0,-pi/2,-pi/4,-pi/4,0,-pi/4,0]);
        
%finding the IKcon for the eighth brick placement, and iterating XYZ co-ordinates
    case 8
        fprintf('Brick 8 moving \n')%Log message
        FinalBrickPos = r.model.ikcon(transl((brickintx + 0.27),brickinty,(brickintz + 0.41)) * troty(-pi) * trotz(pi/2),[0,-pi/2,-pi/4,-pi/4,0,-pi/4,0]);
        
%finding the IKcon for the ninth brick placement, and iterating XYZ co-ordinates
    case 9
        fprintf('Brick 9 moving \n')%Log message
        FinalBrickPos = r.model.ikcon(transl((brickintx + 0.54),brickinty,(brickintz + 0.41)) * troty(-pi) * trotz(pi/2),[0,-pi/2,-pi/4,-pi/4,0,-pi/4,0]);

%finding the IKcon for the ninth brick placement, and iterating XYZ co-ordinates
    case 10
        fprintf('Brick 10 moving \n')%Log message
        FinalBrickPos = r.model.ikcon(transl((brickintx + 0.81),brickinty,(brickintz + 0.41)) * troty(-pi) * trotz(pi/2),[0,3*pi/2,0,-pi/2,0,-3*pi/2,0]);

%finding the IKcon for the ninth brick placement, and iterating XYZ co-ordinates
    case 11
        fprintf('Brick 11 moving \n')%Log message
        FinalBrickPos = r.model.ikcon(transl((brickintx + 1.07),brickinty,(brickintz + 0.41)) * troty(-pi) * trotz(pi/2),[0,3*pi/2,0,-pi/2,0,-3*pi/2,0]);

%finding the IKcon for the ninth brick placement, and iterating XYZ co-ordinates
    case 12
        fprintf('Brick 12 moving \n')%Log message
        FinalBrickPos = r.model.ikcon(transl((brickintx + 1.32),brickinty,(brickintz + 0.41)) * troty(-pi) * trotz(pi/2),[0,-pi/2,-pi/4,-pi/4,0,-pi/4,0]);

   end

    % Generate a trajectory to move the gripper to the placement position
    qMatrix = jtraj(r.model.getpos,FinalBrickPos,steps);

    % Loop to animate the robot movement - From wk4 lab/Wk5 tutorial 
    for i = 1:steps
        % Animate robot movement
        r.model.animate(qMatrix(i,:));
        % Set the base location of the gripper claw to the end effector 
        GripperClawRight.base = r.model.fkine(r.model.getpos);
        GripperClawLeft.base = r.model.fkine(r.model.getpos);
        % Find joint angle of end effector pose using fkine
        clawtr = r.model.fkine(r.model.getpos);
        % Transform and position the brick to the robot endeffector location
        transformedVertices = [vertices{j},ones(size(vertices{j},1),1)] * clawtr.T';
        set(brick{j},'Vertices',transformedVertices(:,1:3));
        drawnow();
        pause(0.01);
    end 

    % Q joints to represent open position of the claw grippers
    openGripperq0 = [0,0,0];
    % Assigning the base of the grippers to the endeffectors location
    GripperClawRight.base = r.model.fkine(r.model.getpos);
    GripperClawLeft.base = r.model.fkine(r.model.getpos);
    %animate grippers opening
    GripperClawRight.animate(openGripperq0);
    GripperClawLeft.animate(openGripperq0);

    fprintf('Joint Angles \n');   %Log message
    r.model.getpos %finding error
    fprintf('Transformation Matrix \n');   %Log message
    r.model.fkine(r.model.getpos) %finding joint angles
    fprintf('Brick Placed \n');   %Log message
end
fprintf('Wall complete, robot stopping \n');   %Log message
