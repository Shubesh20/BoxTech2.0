function varargout = Guitest(varargin)
% GUITEST MATLAB code for Guitest.fig
%      GUITEST, by itself, creates a new GUITEST or raises the existing
%      singleton*.
%
%      H = GUITEST returns the handle to a new GUITEST or the handle to
%      the existing singleton*.
%
%      GUITEST('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUITEST.M with the given input arguments.
%
%      GUITEST('Property','Value',...) creates a new GUITEST or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Guitest_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Guitest_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Guitest

% Last Modified by GUIDE v2.5 28-Oct-2024 23:37:48

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Guitest_OpeningFcn, ...
                   'gui_OutputFcn',  @Guitest_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Guitest is made visible.
function Guitest_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Guitest (see VARARGIN)

% Choose default command line output for Guitest
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Guitest wait for user response (see UIRESUME)
% uiwait(handles.figure1);



% --- Outputs from this function are returned to the command line.
function varargout = Guitest_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;




%% JOINT MOVEMENTS

% OMRON TM5 900

% --- Executes on slider movement.  
function slider1_Callback(hObject, eventdata, handles)  
% hObject   handle to slider1 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   structure with handles and user data (see GUIDATA)  
  
value = get(hObject, 'Value');                                                % Get the current value of the slider 
value_rad = deg2rad(value);                                                   % Convert the value to radians 
global model                                                                  % Access the global model  
q = model.model.getpos();                                                     % Get the current joint angles 
q(1) = value_rad;                                                             % Update the first joint angle  
model.model.animate(q);                                                       % Update the robot's pose  
guidata(hObject, handles);                                                    % Update the handles structure 

% The other sliders are similar in functionality

% --- Executes during object creation, after setting all properties.  
function slider1_CreateFcn(hObject, eventdata, handles)  
% hObject   handle to slider1 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   empty - handles not created until after all CreateFcns called  
  
% Set the minimum and maximum values of the slider  
set(hObject, 'Min', -360);  
set(hObject, 'Max', 360);  
  
% Hint: slider controls usually have a light gray background.  
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))  
   set(hObject,'BackgroundColor',[.9 .9 .9]);  
end


% --- Executes on slider movement.  
function slider2_Callback(hObject, eventdata, handles)  
% hObject   handle to slider2 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   structure with handles and user data (see GUIDATA)  

value = get(hObject, 'Value');   
value_rad = deg2rad(value);   
global model  
q = model.model.getpos();  
q(2) = value_rad;   
model.model.animate(q);  
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.  
function slider2_CreateFcn(hObject, eventdata, handles)  
% hObject   handle to slider2 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   empty - handles not created until after all CreateFcns called  
  
% Set the minimum and maximum values of the slider  
set(hObject, 'Min', -360);  
set(hObject, 'Max', 360);  
  
% Hint: slider controls usually have a light gray background.  
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))  
   set(hObject,'BackgroundColor',[.9 .9 .9]);  
end


% --- Executes on slider movement.  
function slider3_Callback(hObject, eventdata, handles)  
% hObject   handle to slider3 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   structure with handles and user data (see GUIDATA)  

value = get(hObject, 'Value');    
value_rad = deg2rad(value);  
global model  
q = model.model.getpos();   
q(3) = value_rad;  
model.model.animate(q);    
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.  
function slider3_CreateFcn(hObject, eventdata, handles)  
% hObject   handle to slider3 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   empty - handles not created until after all CreateFcns called  
  
% Set the minimum and maximum values of the slider  
set(hObject, 'Min', -360);  
set(hObject, 'Max', 360);  
  
% Hint: slider controls usually have a light gray background.  
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))  
   set(hObject,'BackgroundColor',[.9 .9 .9]);  
end


% --- Executes on slider movement.  
function slider4_Callback(hObject, eventdata, handles)  
% hObject   handle to slider4 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   structure with handles and user data (see GUIDATA)  
  
value = get(hObject, 'Value');  
value_rad = deg2rad(value);  
global model    
q = model.model.getpos();  
q(4) = value_rad;   
model.model.animate(q);   
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.  
function slider4_CreateFcn(hObject, eventdata, handles)  
% hObject   handle to slider4 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   empty - handles not created until after all CreateFcns called  
  
% Set the minimum and maximum values of the slider  
set(hObject, 'Min', -360);  
set(hObject, 'Max', 360);  
  
% Hint: slider controls usually have a light gray background.  
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))  
   set(hObject,'BackgroundColor',[.9 .9 .9]);  
end


% --- Executes on slider movement.  
function slider5_Callback(hObject, eventdata, handles)  
 
value = get(hObject, 'Value');  
value_rad = deg2rad(value);  
global model   
q = model.model.getpos();  
q(5) = value_rad;  
model.model.animate(q);  
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.  
function slider5_CreateFcn(hObject, eventdata, handles)  
% hObject   handle to slider5 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   empty - handles not created until after all CreateFcns called  
  
% Set the minimum and maximum values of the slider  
set(hObject, 'Min', -360);  
set(hObject, 'Max', 360);  
  
% Hint: slider controls usually have a light gray background.  
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))  
   set(hObject,'BackgroundColor',[.9 .9 .9]);  
end


% --- Executes on slider movement.  
function slider6_Callback(hObject, eventdata, handles)  
% hObject   handle to slider6 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   structure with handles and user data (see GUIDATA)  
    
value = get(hObject, 'Value');    
value_rad = deg2rad(value);   
global model  
q = model.model.getpos();  
q(6) = value_rad;  
model.model.animate(q);   
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.  
function slider6_CreateFcn(hObject, eventdata, handles)  
% hObject   handle to slider6 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   empty - handles not created until after all CreateFcns called  
  
% Set the minimum and maximum values of the slider  
set(hObject, 'Min', -360);  
set(hObject, 'Max', 360);  
  
% Hint: slider controls usually have a light gray background.  
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))  
   set(hObject,'BackgroundColor',[.9 .9 .9]);  
end


% Linear UR5

% --- Executes on slider movement.  
function slider10_Callback(hObject, eventdata, handles)  
% hObject   handle to slider10 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   structure with handles and user data (see GUIDATA)  
  

value = get(hObject, 'Value');   
global models   
q = models.model.getpos();   
q(1) = value;  
models.model.animate(q);  
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.  
function slider10_CreateFcn(hObject, eventdata, handles)  
% hObject   handle to slider10 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   empty - handles not created until after all CreateFcns called  
  
% Set the minimum and maximum values of the slider  
set(hObject, 'Min', -0.8);  
set(hObject, 'Max', -0.01);  
set(hObject, 'Value', -0.8);
  
% Hint: slider controls usually have a light gray background.  
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))  
   set(hObject,'BackgroundColor',[.9 .9 .9]);  
end


% --- Executes on slider movement.  
function slider11_Callback(hObject, eventdata, handles)  
% hObject   handle to slider11 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   structure with handles and user data (see GUIDATA)  
  
value = get(hObject, 'Value');  
value_rad = deg2rad(value);    
global models    
q = models.model.getpos();  
q(2) = value_rad;   
models.model.animate(q);  
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.  
function slider11_CreateFcn(hObject, eventdata, handles)  
% hObject   handle to slider11 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   empty - handles not created until after all CreateFcns called  
  
% Set the minimum and maximum values of the slider  
set(hObject, 'Min', -360);  
set(hObject, 'Max', 360);  
 
  
% Hint: slider controls usually have a light gray background.  
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))  
   set(hObject,'BackgroundColor',[.9 .9 .9]);  
end


% --- Executes on slider movement.  
function slider12_Callback(hObject, eventdata, handles)  
% hObject   handle to slider11 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   structure with handles and user data (see GUIDATA)  
   
value = get(hObject, 'Value');   
value_rad = deg2rad(value);  
global models    
q = models.model.getpos();   
q(3) = value_rad;  
models.model.animate(q);  
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.  
function slider12_CreateFcn(hObject, eventdata, handles)  
% hObject   handle to slider11 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   empty - handles not created until after all CreateFcns called  
  
% Set the minimum and maximum values of the slider  
set(hObject, 'Min', -360);  
set(hObject, 'Max', 360);  
 
  
% Hint: slider controls usually have a light gray background.  
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))  
   set(hObject,'BackgroundColor',[.9 .9 .9]);  
end


% --- Executes on slider movement.  
function slider13_Callback(hObject, eventdata, handles)  
% hObject   handle to slider11 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   structure with handles and user data (see GUIDATA)  
  
 
value = get(hObject, 'Value');   
value_rad = deg2rad(value);  
global models   
q = models.model.getpos();  
q(4) = value_rad;  
models.model.animate(q);    
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.  
function slider13_CreateFcn(hObject, eventdata, handles)  
% hObject   handle to slider11 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   empty - handles not created until after all CreateFcns called  
  
% Set the minimum and maximum values of the slider  
set(hObject, 'Min', -360);  
set(hObject, 'Max', 360);  
   
% Hint: slider controls usually have a light gray background.  
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))  
   set(hObject,'BackgroundColor',[.9 .9 .9]);  
end


% --- Executes on slider movement.  
function slider14_Callback(hObject, eventdata, handles)  
% hObject   handle to slider11 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   structure with handles and user data (see GUIDATA)  
  
 
value = get(hObject, 'Value');   
value_rad = deg2rad(value);  
global models   
q = models.model.getpos();   
q(5) = value_rad;   
models.model.animate(q);   
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.  
function slider14_CreateFcn(hObject, eventdata, handles)  
% hObject   handle to slider11 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   empty - handles not created until after all CreateFcns called  
  
% Set the minimum and maximum values of the slider  
set(hObject, 'Min', -360);  
set(hObject, 'Max', 360);  
 
  
% Hint: slider controls usually have a light gray background.  
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))  
   set(hObject,'BackgroundColor',[.9 .9 .9]);  
end


% --- Executes on slider movement.  
function slider15_Callback(hObject, eventdata, handles)  
% hObject   handle to slider11 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   structure with handles and user data (see GUIDATA)  
   
value = get(hObject, 'Value');   
value_rad = deg2rad(value);  
global models  
q = models.model.getpos();  
q(6) = value_rad;   
models.model.animate(q);  
  
% Update the handles structure  
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.  
function slider15_CreateFcn(hObject, eventdata, handles)  
% hObject   handle to slider11 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   empty - handles not created until after all CreateFcns called  
  
% Set the minimum and maximum values of the slider  
set(hObject, 'Min', -360);  
set(hObject, 'Max', 360);  
 
  
% Hint: slider controls usually have a light gray background.  
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))  
   set(hObject,'BackgroundColor',[.9 .9 .9]);  
end


% --- Executes on slider movement.  
function slider16_Callback(hObject, eventdata, handles)  
% hObject   handle to slider11 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   structure with handles and user data (see GUIDATA)  
   
value = get(hObject, 'Value');   
value_rad = deg2rad(value);  
global models  
q = models.model.getpos();  
q(7) = value_rad;   
models.model.animate(q);  
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.  
function slider16_CreateFcn(hObject, eventdata, handles)  
% hObject   handle to slider11 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   empty - handles not created until after all CreateFcns called  
  
% Set the minimum and maximum values of the slider  
set(hObject, 'Min', -360);  
set(hObject, 'Max', 360);  
   
% Hint: slider controls usually have a light gray background.  
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))  
   set(hObject,'BackgroundColor',[.9 .9 .9]);  
end


%% Cartesian Movements


%%%%%%%%%%%%%%%%%%%%%%%%%%%% OMRON TM5 900 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% X-Global Frame
% --- Executes on button press in minusX_pushbutton.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to minusX_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global model  
q = model.model.getpos;
tr = model.model.fkine(q);
tr = double(tr);
tr(1,4) = tr(1,4) - 0.1;
newQ = model.model.ikcon(tr,q);
model.model.animate(newQ);

% --- Executes on button press in plusX_pushbutton.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to plusX_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global model  
q = model.model.getpos;
size(q);
tr = model.model.fkine(q);
tr = double(tr);
tr(1,4) = tr(1,4) + 0.1;
newQ = model.model.ikcon(tr,q);
model.model.animate(newQ);


% Y-Global Frame
% --- Executes on button press in minusY_pushbutton.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to minusY_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global model 
q = model.model.getpos;
tr = model.model.fkine(q);
tr = double(tr);
tr(2,4) = tr(2,4) - 0.01;
newQ = model.model.ikcon(tr,q);
model.model.animate(newQ);

% --- Executes on button press in plusY_pushbutton.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to plusY_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global model 
q = model.model.getpos;
size(q);
tr = model.model.fkine(q);
tr = double(tr);
tr(2,4) = tr(2,4) + 0.01;
newQ = model.model.ikcon(tr,q);
model.model.animate(newQ);


% Z-Global Frame
% --- Executes on button press in minusZ_pushbutton.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to minusZ_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global model
q = model.model.getpos;
tr = model.model.fkine(q);
tr = double(tr);
tr(3,4) = tr(3,4) - 0.01;
newQ = model.model.ikcon(tr,q);
model.model.animate(newQ); 

% --- Executes on button press in plusZ_pushbutton.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to plusZ_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global model
q = model.model.getpos;
size(q);
tr = model.model.fkine(q);
tr = double(tr);
tr(3,4) = tr(3,4) + 0.01;
newQ = model.model.ikcon(tr,q);
model.model.animate(newQ);






%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Linear UR5 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% X-Global Frame
% --- Executes on button press in minusX_pushbutton.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to minusX_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global models  
q = models.model.getpos;
tr = models.model.fkine(q);
tr = double(tr);
tr(1,4) = tr(1,4) - 0.1;
newQ = models.model.ikcon(tr,q);
models.model.animate(newQ);

% --- Executes on button press in plusX_pushbutton.
function pushbutton8_Callback(hObject, eventdata, handles)
% hObject    handle to plusX_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global models  
q = models.model.getpos;
size(q);
tr = models.model.fkine(q);
tr = double(tr);
tr(1,4) = tr(1,4) + 0.1;
newQ = models.model.ikcon(tr,q);
models.model.animate(newQ);



% Y-Global Frame
% --- Executes on button press in minusY_pushbutton.
function pushbutton9_Callback(hObject, eventdata, handles)
% hObject    handle to minusY_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global models 
q = models.model.getpos;
tr = models.model.fkine(q);
tr = double(tr);
tr(2,4) = tr(2,4) - 0.01;
newQ = models.model.ikcon(tr,q);
models.model.animate(newQ);

% --- Executes on button press in plusY_pushbutton.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to plusY_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global models 
q = models.model.getpos;
size(q);
tr = models.model.fkine(q);
tr = double(tr);
tr(2,4) = tr(2,4) + 0.01;
newQ = models.model.ikcon(tr,q);
models.model.animate(newQ);


% Z-Global Frame
% --- Executes on button press in minusZ_pushbutton.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to minusZ_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global models
q = models.model.getpos;
tr = models.model.fkine(q);
tr = double(tr);
tr(3,4) = tr(3,4) - 0.01;
newQ = models.model.ikcon(tr,q);
models.model.animate(newQ); 

% --- Executes on button press in plusZ_pushbutton.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to plusZ_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global models
q = models.model.getpos;
size(q);
tr = models.model.fkine(q);
tr = double(tr);
tr(3,4) = tr(3,4) + 0.01;
newQ = models.model.ikcon(tr,q);
models.model.animate(newQ);


%% E-STOP

% E-STOP toggle button
function togglebutton1_Callback(hObject, eventdata, handles)  
% hObject   handle to togglebutton1 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   structure with handles and user data (see GUIDATA)  

% Get the current state of the toggle button  
estop_state = get(hObject, 'Value');  
  
% Update the E-stop state  
global estop_engaged  
global resume_simulation  
if estop_state == 1  
   estop_engaged = 1;  
   resume_simulation = 0;  
   % Display message in command window  
   fprintf(' !!! E-STOP ENGAGED: SBT 2.0 PAUSING !!! \n');  
else if estop_state == 0 || resume_simulation == 0   
   estop_engaged = 0;
   % Display message in command window  
   fprintf(' !!! E-STOP DISENGAGED: SBT 2.0  BACK TO NEAUTRAL, CLICK *START* TO CONTINUE !!! \n');  
end
end



% RESUME/START BUTTON  
% --- Executes on button press in pushbutton2.  
function pushbutton13_Callback(hObject, eventdata, handles)  
% hObject   handle to pushbutton2 (see GCBO)  
% eventdata  reserved - to be defined in a future version of MATLAB  
% handles   structure with handles and user data (see GUIDATA)  

% Check if E-stop is disengaged  
global estop_engaged  
global resume_simulation  
if estop_engaged == 0  
   resume_simulation = 1;  
   % Display message in command window  
   fprintf(' !!! SBT 2.0 WILL CONTINUE OPERATION !!! \n');  
end









