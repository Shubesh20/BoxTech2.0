classdef RobotA1 < RobotBaseClass
    %% UR3 Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'LinearUR3e';
    end
    
    methods
%% Constructor
        function self = RobotA1(baseTr,useTool,toolFilename)
            self.CreateModel();
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);  
                end

            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end

            hold on
			self.model.base = self.model.base.T * baseTr; % * trotx(pi/2)
            self.PlotAndColourRobot();
       end

%% CreateModel
        function CreateModel(self) 

            link(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link 
            link(2) = Link('d',0.15185,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',0,'a',-0.24355,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            link(4) = Link('d',0,'a',-0.2132,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            link(5) = Link('d',0.13105,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            link(6) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(7) = Link('d',	0.0921,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

            link(1).qlim = [-0.7 -0.02];
            link(2).qlim = [-180 180]*pi/180;
            link(3).qlim = [-180 0]*pi/180;
            link(4).qlim = [-90 90]*pi/180;
            link(5).qlim = [-360 360]*pi/180;
            link(6).qlim = [-360 360]*pi/180;
            link(7).qlim = [-360 360]*pi/180;
             
            self.model = SerialLink(link, 'name', self.name);
        end
    end
end