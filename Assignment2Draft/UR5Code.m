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



            %% Begin brick placement procedure

            fprintf('Initialising brick wall building, robot active \n');   %Log message
            steps = 45; %Number of iteration steps
            % Loop to simulate the robot picking and placing bricks on the table
            for j = 1:12
              
                % Compute the inverse kinematics for the brick position
                %ikcon - optimisation method and considers joint limits
                if j == 12 %loop for 12 bricks
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
                        FinalBrickPos = r.model.ikcon(transl((brickintx + 1.32),brickinty,(brickintz + 0.41)) * troty(-pi) * trotz(pi/2),[0,3*pi/2,0,-pi/2,0,-3*pi/2,0]);

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