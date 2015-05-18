function varargout = MainGUI(varargin)
% MAINGUI MATLAB code for MainGUI.fig
%      MAINGUI, by itself, creates a new MAINGUI or raises the existing
%      singleton*.
%
%      H = MAINGUI returns the handle to a new MAINGUI or the handle to
%      the existing singleton*.
%
%      MAINGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MAINGUI.M with the given input arguments.
%
%      MAINGUI('Property','Value',...) creates a new MAINGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MainGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MainGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MainGUI

% Last Modified by GUIDE v2.5 10-May-2015 18:17:03

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @MainGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @MainGUI_OutputFcn, ...
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

end
% --- Executes just before MainGUI is made visible.
function MainGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to MainGUI (see VARARGIN)

% Choose default command line output for MainGUI
handles.output = hObject;
handles.t= timer(...
    'ExecutionMode', 'fixedRate', ...   % Run timer repeatedly
    'Period', 0.06, ...                % Initial period is 1 sec.
    'TimerFcn', @timer_Callback); % Specify callback

global robot_IP_address; global robot_port; global socket; global buffer;
robot_IP_address = '127.0.0.1';
robot_port = 7027;
% socket = tcpip(robot_IP_address, robot_port);
buffer = [];
set(socket, 'ReadAsyncMode', 'continuous'); 
% 
% fopen(socket); 

% Update handles structure
guidata(hObject, handles);
start(handles.t);




end





function timer_Callback(obj,event)
global robot_IP_address; global robot_port; global socket;global commandFlag; global buffer;
global databuffer;global tableUpdate;
    if isempty(buffer)
        buffer = '500050005000500050005000010000';
        commandFlag.m = buffer;
    end
    %   Check if the connection is valid.
    if(~isequal(get(socket, 'Status'), 'open'))
%         warning(['Could not open TCP connection to ', robot_IP_address, ' on port ', robot_port]);
    else
    
        % Read a line from the socket. Note the line feed appended to the message in the RADID sample code.
        data = fgetl(socket); %disp(data);
        databuffer = data; lastBits = str2double(databuffer(29:30));
        dataNum = str2num(data);
%         lastBits = rem(dataNum,100);    
%         disp(lastBits);
        if lastBits > 90     

            dataNum = (dataNum - lastBits) / 100;
            Xpos = rem(dataNum, 10000)-5000;

            dataNum = (dataNum - Xpos)/10000;
            Ypos = rem(dataNum, 10000)-5000;

            dataNum = (dataNum - Ypos)/10000;
            Zpos = rem(dataNum, 10000)-5000;

            % Send feedback to the server on the robot.
%             disp(commandFlag.stat);
            if commandFlag.stat;                
                command = commandFlag.m;
            else
                command = buffer; 
            end
                fwrite(socket, command);  
                

        else
             fwrite(socket, data);
        end
%         buffer = commandFlag.m;
%         pause(0.1);
    end
    if ~isempty(databuffer)
        loBuf_X = str2num(databuffer((21+1):24));
        loBuf_Y = str2num(databuffer((17+1):20));loBuf_Z = str2num(databuffer((13+1):16));
        loBuf_J1 = str2num(databuffer(11:12));loBuf_J2 = str2num(databuffer(9:10));
        loBuf_J3 = str2num(databuffer(7:8));loBuf_J4 = str2num(databuffer(5:6));
        loBuf_J5 = str2num(databuffer(3:4));loBuf_J6 = str2num(databuffer(1:2));
        tableUpdate = [4*(loBuf_J6-50),4*(loBuf_J5-50),4*(loBuf_J4-50),4*(loBuf_J3-50),4*(loBuf_J2-50),4*(loBuf_J1-50),loBuf_Z,loBuf_Y,loBuf_X];
    else
        tableUpdate = [0,0,0,0,0,0,0,0,0,0,0,0;0,0,0,0,0,0,0,0,0,0,0,0];
    end
    
    %     disp(tableUpdate);
        disp(commandFlag.m);
% commandFlag.CD,commandFlag.Vac,commandFlag.speed,

end





% UIWAIT makes MainGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = MainGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
global buffer;global databuffer;
data = [0,0,0,0,0,0,0,0,0,0,0,0;0,0,0,0,0,0,0,0,0,0,0,0];
set(handles.uitable3,'data',data);
data = [0,0,100,0,0,0,0,0,0,0,0,0;0,0,0,50,50,50,50,50,50,0,0,0];
set(handles.uitable3,'data',data);
if ~isempty(buffer)&&~isempty(databuffer)
    loBuf_X = str2num(databuffer((21+1):24));
    loBuf_Y = str2num(databuffer((17+1):20));loBuf_Z = str2num(databuffer((13+1):16));
    loBuf_J1 = str2num(databuffer(11:12));loBuf_J2 = str2num(databuffer(9:10));
    loBuf_J3 = str2num(databuffer(7:8));loBuf_J4 = str2num(databuffer(5:6));
    loBuf_J5 = str2num(databuffer(3:4));loBuf_J6 = str2num(databuffer(1:2));
    data = [0,0,100,4*(loBuf_J6-50),4*(loBuf_J5-50),4*(loBuf_J4-50),4*(loBuf_J3-50),4*(loBuf_J2-50),4*(loBuf_J1-50),loBuf_Z,loBuf_Y,loBuf_X;0,0,0,0,0,0,0,0,0,0,0,0];
    set(handles.uitable3,'data',data);   
end
end

% --- Executes on button press in previewCamera.
function previewCamera_Callback(hObject, eventdata, handles)
% hObject    handle to previewCamera (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vid; global vid2;
vid = videoinput('winvideo',1,'RGB24_640x480');
% vid2 = videoinput('winvideo',2,'RGB24_1600x1200');
hImage = image(zeros(640,480,3),'Parent',handles.axes1);
hImage2 = image(zeros(1600,1200,3),'Parent',handles.axes2);
preview(vid,hImage); 
% preview(vid2,hImage2);

% [cdata,map] = imread('red-estop-Junior-s-Tees.jpg'); 
%  h=msgbox('Check emergency button',...
%         'Warning','custom',cdata,map);
end



% --- Executes on button press in SaveSnap.
function SaveSnap_Callback(hObject, eventdata, handles)
% hObject    handle to SaveSnap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vid;
%---change data in table DEMO in a pushbutton callback----
% data = get(handles.uitable3, 'data');
% data(1,:) = 1;
% set(handles.uitable3,'data',data);
%---------------------------------------------
img = getsnapshot(vid);%show the taken snapshot
prompt = 'please type in file name with image type  ';
str = input(prompt,'s');
figure(1);clf;
imshow(img, 'InitialMagnification', 'fit');
%save the snapshot, check documentation of imwrite
imwrite(img, str);
end


% --- Executes on button press in Ginput.
function Ginput_Callback(hObject, eventdata, handles)
% hObject    handle to Ginput (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vid; global commandFLag; 
data = get(handles.uitable3,'data');
speed = format(int2str(data(1,3)),4);
stoppreview(vid);
[x,y] = ginput(1);
%output xd,yd
% disp([x,y]);
% [xd,yd] = Pixel2Pose(x,y);
[xd,yd] = Pixel2Pose(500,400);
preview(vid);
% Hint: get(hObject,'Value') returns toggle state of Up
xd = int16(xd); yd = int16(yd);
% disp([xd,yd])
% xd = uint64(xd); yd = uint64(yd);
% disp([xd,yd])
commandStr = ['5000' '5000' '5000' '5000' int2str(yd+5000) int2str(xd+5000) speed '06'];
for s = 1:0.5:3
    commandFLag.m = commandStr;
    pause(0.2);
end
end


function [a, b] = Pixel2Pose(x, y)
if x >= 794 
    b = (x-794) * 520 / (1589 - 794);
elseif x < 794
    b = (x-794) * 520/ (794 - 10.5);
end
a = (y-292) * 373.6 / (862 - 292) + 175;
end
    
% --- Executes on button press in EF.
function EF_Callback(hObject, eventdata, handles)
% hObject    handle to EF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of EF

end





% --- Executes on button press in TrackChocolate.
function TrackChocolate_Callback(hObject, eventdata, handles)
% hObject    handle to TrackChocolate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vid;
stat = get(hObject,'Value');
stoppreview(vid);
while stat == 1
% %      image_choco = getsnapshot(vid);
%     image_choco = imread('IMG_0014.jpg');
%     output = DetectChocolate3(image_choco);

end

preview(vid);
% Hint: get(hObject,'Value') returns toggle state of TrackChocolate

end

%Chocolate image processing from PSE1-----------------------------------------------


% --- Executes on button press in Up.
function Up_Callback(hObject, eventdata, handles)
% hObject    handle to Up (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of Up
global commandFlag;global tableUpdate;
commandFlag.stat = get(hObject,'Value');
stat = get(hObject,'Value');
data = get(handles.uitable3,'data');
speed = format(int2str(data(1,3)),4);
    while stat
        stat = get(hObject,'Value'); 
        drawnow; 
%         if ~stat, break; end
        if ~get(handles.LT,'Value')
%             drawnow;
            buf = (['5000' '5000' '5200' speed '05']);
        else
%             drawnow;
            %do torsion 
            MoveJoint = str2double(get(handles.JointNo,'String'));
            switch MoveJoint
                case 1
                    buf = (['5000' '5000' '5000' '5000' '5000' '5200' speed '07']);       
                case 2
                    buf = (['5000' '5000' '5000' '5000' '5200' '5000' speed '07']);
                case 3
                    buf = (['5000' '5000' '5000' '5200' '5000' '5000' speed '07']);
                case 4
                    buf = (['5000' '5000' '5200' '5000' '5000' '5000' speed '07']);
                case 5
                    buf = (['5000' '5200' '5000' '5000' '5000' '5000' speed '07']);
                case 6
                    buf = (['5200' '5000' '5000' '5000' '5000' '5000' speed '07']);
            end                      
        end
            commandStr = format(buf,30);
            commandFlag.m = commandStr; 
            data = get(handles.uitable3,'data');
            data(1,4:12) = tableUpdate;
            set(handles.uitable3,'data',data);  
    end
            buf = ['5000' '5000' '5000' '5000' '5000' '5000' speed '00'];
            commandStr = format(buf,30);
            commandFlag.m = commandStr;  
end

% --- Executes on button press in Down.
function Down_Callback(hObject, eventdata, handles)
% hObject    handle to Down (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Down
% Hint: get(hObject,'Value') returns toggle state of Up
global commandFlag; global tableUpdate;
commandFlag.stat = get(hObject,'Value');
stat = get(hObject,'Value');
data = get(handles.uitable3,'data');
speed = format(int2str(data(1,3)),4);
    while stat   
        stat = get(hObject,'Value'); 
        drawnow; 
%         if ~stat, break; end
        if ~get(handles.LT,'Value')
%             drawnow;
            buf = (['5000' '5000' '4800' speed '05']);
        else
%             drawnow;
            %do torsion 
            MoveJoint = str2double(get(handles.JointNo,'String'));
            switch MoveJoint
                case 1
                    buf = (['5000' '5000' '5000' '5000' '5000' '4800' speed '07']);       
                case 2
                    buf = (['5000' '5000' '5000' '5000' '4800' '5000' speed '07']);
                case 3
                    buf = (['5000' '5000' '5000' '4800' '5000' '5000' speed '07']);
                case 4
                    buf = (['5000' '5000' '4800' '5000' '5000' '5000' speed '07']);
                case 5
                    buf = (['5000' '4800' '5000' '5000' '5000' '5000' speed '07']);
                case 6
                    buf = (['4800' '5000' '5000' '5000' '5000' '5000' speed '07']);
            end                      
        end
            commandStr = format(buf,30);
            commandFlag.m = commandStr; 
            data = get(handles.uitable3,'data');
            data(1,4:12) = tableUpdate;
            set(handles.uitable3,'data',data);   
    end
            buf = ['5000' '5000' '5000' '5000' '5000' '5000' speed '00'];
            commandStr = format(buf,30);
            commandFlag.m = commandStr;  
end

% --- Executes on button press in Left.
function Left_Callback(hObject, eventdata, handles)
% hObject    handle to Left (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of Left
% Hint: get(hObject,'Value') returns toggle state of Up
global commandFlag;global tableUpdate;
commandFlag.stat = get(hObject,'Value');
stat = get(hObject,'Value');
data = get(handles.uitable3,'data');
speed = format(int2str(data(1,3)),4);
buf = (['5000' '5200' '5000' speed '05']);
commandStr = format(buf,30); 
    while stat   
        drawnow;
        commandFlag.m = commandStr;
        data = get(handles.uitable3,'data');
        data(1,4:12) = tableUpdate;
        set(handles.uitable3,'data',data);
    end
    buf = ['5000' '5000' '5000' '5000' '5000' '5000' speed '00'];
    commandStr = format(buf,30);
    commandFlag.m = commandStr;  
end

% --- Executes on button press in Right.
function Right_Callback(hObject, eventdata, handles)
% hObject    handle to Right (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of Right
% Hint: get(hObject,'Value') returns toggle state of Up
global commandFlag;global tableUpdate;
commandFlag.stat = get(hObject,'Value');
stat = get(hObject,'Value');
data = get(handles.uitable3,'data');
speed = format(int2str(data(1,3)),4);
buf = (['5000' '4800' '5000' speed '05']);
commandStr = format(buf,30); 
    while stat   
        drawnow;
        commandFlag.m = commandStr;
        data = get(handles.uitable3,'data');
        data(1,4:12) = tableUpdate;
        set(handles.uitable3,'data',data);
    end
    buf = ['5000' '5000' '5000' '5000' '5000' '5000' speed '00'];
    commandStr = format(buf,30);
    commandFlag.m = commandStr;  
end

% --- Executes on button press in Zdown.
function Zdown_Callback(hObject, eventdata, handles)
% hObject    handle to Zdown (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of Zdown
% Hint: get(hObject,'Value') returns toggle state of Up
global commandFlag; global tableUpdate;
commandFlag.stat = get(hObject,'Value');
stat = get(hObject,'Value');
data = get(handles.uitable3,'data');
speed = format(int2str(data(1,3)),4);
buf = (['5200' '5000' '5000' speed '05']);
commandStr = format(buf,30); 
    while stat   
        drawnow;
        commandFlag.m = commandStr;
        data = get(handles.uitable3,'data');
        data(1,4:12) = tableUpdate;
        set(handles.uitable3,'data',data);
    end
    buf = ['5000' '5000' '5000' '5000' '5000' '5000' speed '00'];
    commandStr = format(buf,30);
    commandFlag.m = commandStr;  
end

% --- Executes on button press in Zup.
function Zup_Callback(hObject, eventdata, handles)
% hObject    handle to Zup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of Zup
global commandFlag; global tableUpdate;
commandFlag.stat = get(hObject,'Value');
stat = get(hObject,'Value');
data = get(handles.uitable3,'data');
speed = format(int2str(data(1,3)),4);
buf = (['4800' '5000' '5000' speed '05']);
commandStr = format(buf,30); 
    while stat   
        drawnow;
        commandFlag.m = commandStr;
        data = get(handles.uitable3,'data');
        data(1,4:12) = tableUpdate;
        set(handles.uitable3,'data',data);
    end
    buf = ['5000' '5000' '5000' '5000' '5000' '5000' speed '00'];
    commandStr = format(buf,30);
    commandFlag.m = commandStr; 
end


function replaceString = format(str,n)
    if length(str)<n
        zeroString = mat2str(zeros(1,(n-length(str))));
        if length(zeroString) == 1
            buffer = zeroString;
        else
            buffer = zeroString(2:length(zeroString)-1);
        end
        replaceString = regexprep(buffer,'[^\w'']','');
        replaceString = strcat(replaceString,str);        
    else
        replaceString = str;
    end
    return;
end

function result = negative2positive(a)
    if a <0
        buffer = int2str(a);
%         disp(length(buffer));
        result = str2num(strcat('9', buffer(2:end)));   
    elseif a>=0
        result = a;
    end
    return;
end
% --- Executes on button press in LT.
function LT_Callback(hObject, eventdata, handles)
% hObject    handle to LT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of LT

P_stat = 1 -  get(handles.LT,'Value');

if P_stat == 0
    set(handles.Up,'String','clockwise');
    set(handles.Down,'String','count-clockwise');
    set(handles.Left,'enable','off');
    set(handles.Right,'enable','off');
    
else
    set(handles.Up,'String','Up');
    set(handles.Down,'String','Down');
    set(handles.Left,'enable','on');
    set(handles.Right,'enable','on');
end

end


% --- Executes on button press in StopTimer.
function StopTimer_Callback(hObject, eventdata, handles)
global socket;
% hObject    handle to StopTimer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
stop(handles.t);
% fclose(socket);

end



function JointNo_Callback(hObject, eventdata, handles)
% hObject    handle to JointNo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of JointNo as text
%        str2double(get(hObject,'String')) returns contents of JointNo as a double

end
% --- Executes during object creation, after setting all properties.
function JointNo_CreateFcn(hObject, eventdata, handles)
% hObject    handle to JointNo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end




% function DetectChocolate
function output= DetectChocolate3(im)
sceneImage = im;
% imshow(sceneImage);
sceneGray = rgb2gray(sceneImage);
sceneGray2 = sceneGray;
scenePoints = detectSURFFeatures(sceneGray,'MetricThreshold',75);
[sceneFeatures, scenePoints] = extractFeatures(sceneGray, scenePoints);
c1 = [];
count3 = [];
% image1 = imread(strcat(program_folder, 'chocolate smaples/front1.jpg'));


chocPolygon = [1, 1;...                           % top-left
        177, 1;...                 % top-right
        177, 81;... % bottom-right)
        1, 81;...                 % bottom-left
        1, 1];                   % top-left again to close the poly
n = 0;
times = 1;
count = 0;
count2 = 1;
figure(1);
% imshow(sceneImage);
hold on;
% load('chocolateData/chocData.mat');
% load(strcat(program_folder, 'chocolateData/chocData.mat'));
while (times <= 600)
    times = times+1;
	count = count+1;

if count == 32 ||count <= 0
    count = 1;
end
    
        choc = sprintf('%s%d.mat', 'chocolateData/chocData', count);
%         choc = sprintf('%s%d.mat', 'chocData', count);    
        load(choc);

%     [chocFeatures, chocPoints] = extractFeatures(chocGray, chocPoints);
%     
%     [sceneFeatures, scenePoints] = extractFeatures(sceneGray, scenePoints);


    chocPairs = matchFeatures(chocFeatures, sceneFeatures, 'MatchThreshold',3);

    matchedChocPoints = chocPoints(chocPairs(:, 1), :);
    matchedScenePoints = scenePoints(chocPairs(:, 2), :);

    [tform, inlierChocPoints, inlierScenePoints, status] = ...
        estimateGeometricTransform(matchedChocPoints, matchedScenePoints, 'affine');
    
    if status ~= 0
        if count2 <= 50
        count2 = count2 + 1;
            continue;
        else
%            count = count+1;
%            continue;
            break;
        end
    end
%     figure(6);
%     showMatchedFeatures(chocGray, sceneGray, inlierChocPoints, ...
%         inlierScenePoints, 'montage');
%     title('Matched Points (Inliers Only)');



    newChocPolygon = transformPointsForward(tform, chocPolygon);

    N = size(sceneImage);
    c = [newChocPolygon(1,1);newChocPolygon(2,1);newChocPolygon(3,1);newChocPolygon(4,1);newChocPolygon(5,1)];
    r = [newChocPolygon(1,2);newChocPolygon(2,2);newChocPolygon(3,2);newChocPolygon(4,2);newChocPolygon(5,2)];
    BW = roipoly(sceneGray2,c,r);
            
    bwWhite = find(BW~=0);
    if numel(bwWhite) <= 13037 || numel(bwWhite) >= 15637 
        if count2 <= 50
            count2 = count2 + 1;
            continue;
        else
%            count = count+1;
%            continue;
            break;
        end
    end
    stat = regionprops(BW, 'Perimeter', 'Centroid', 'Orientation');
                
    
%     m = matchedScenePoints.Count;

        B = stat.Perimeter;
        C = stat.Centroid;
        ori = stat.Orientation;
    if B >= 486 && B <= 546;
        count3 = [count3, count];
        ref = (newChocPolygon(1,1) + newChocPolygon(4,1))/2;
%         ref(2) = mean(newChocPolygon(2,2), newChocPolygon(3,2));
        if count<=20;
            [Flavour, Angle, Reachable, Pickable] = detectDetails(BW, sceneImage, sceneGray2, C, ori, ref);
%             disp(flavour); disp(C(1)); disp(C(2)); disp(Angle); disp(Reachable);
            ForB = 1;
%             c = [C(1), C(2), Angle, 177, 81, Flavour, ForB, Reachable, Pickable];
        else
            [Flavour, Angle, Reachable, Pickable] = detectDetailsBack(C, ori, ref);
            ForB = 0;
%             disp(flavour); disp(C); disp(Angle); disp(Reachable);
%             c = [C(1), C(2), Angle, 177, 81, Flavour, ForB, Reachable, Pickable];
        end
        c2 = [1600 - C(1), C(2), Angle, 177, 81, Flavour, ForB, Reachable, Pickable];

        n = n+1;
        x2 = C(1) + 150 * cos(Angle);
        y2 = C(2) - 150 * sin(Angle); 
        x2 = x2 - 2 * (x2 -C(1));
        y2 = y2 - 2 * (y2 - C(2));
%         position = []
%         
%         plot(100,100,'*r',handles.axes1);
%         
%         plot([x2 1600-C(1)], [y2 C(2)], 'r');
%         plot([C(1)-150, C(1)], [C(2) C(2)], 'r');
%         plot(C(1), C(2),'r')
%         centre_str = sprintf('%s\n%d\n%s\n[%d, %d]\n%s\n%d','n =',n ,'Center =', int32(C(1)), int32(C(2)), 'Orientation =', Angle);
%         text(C(1),C(2), centre_str, 'Color','y',...
%        'FontSize',7,'FontWeight','bold');    
        count = count-3;
        BW1 = uint8(BW);
        Gray = BW1 .* sceneGray;
        for i = 1:scenePoints.Count
            x1 = int32(scenePoints.Location(i,1));
            y1 = int32(scenePoints.Location(i,2));
            if Gray(y1,x1)~=0
                sceneFeatures(i,:) = 0;
            end
        end

%         
%         
        for i = 1:N(1)
            for j = 1:N(2)
                if BW(i,j) == 1
                    sceneGray2(i,j) = 0;                        
                end   
            end
        end

%         a = size(chocPairs);
%         for i = 1:a(1)
%             sceneFeatures(chocPairs(i,2),:) = 0;
%         end
        
    else
        if count2 <= 50
            count2 = count2 + 1;
            continue;
        else
%            count = count+1;
%            continue;
            break;
        end

    end
    count2 = 0;
%     count2 = 0;
    
%     stat = struct([]);
end
value = []
positon = [c2(:,1), c2(:,2)];    
for k = 1:numel(c2(:,1))
    value(k) = sprintf('%s\n%d\n%s\n[%d, %d]\n%s\n%d','n =',k ,'Center =', int32(c2(:,1)), int32(c2(:,2)), 'Orientation =', Angle(k));
end

% figure;
% imshow(sceneImage);
% hold on;
% for i = 1:n
%     line(allChocPolygons(5*i-4:5*i, 1), allChocPolygons(5*i-4:5*i, 2), 'Color', 'y');
% end
% disp(count3);
hold off;
% disp(times);
end

function [flavour, Angle, Reachable, Pickable] = detectDetails(BW, sceneImage, sceneGray, C, ori, ref)
    ori = ori*pi/180;
    BW1 = uint8(BW);
    Gray = BW1 .* sceneGray;
    RGB = sceneImage;
    s = size(sceneImage);
    for i = 1:s(1)
        for j = 1:s(2)
            if Gray(i,j) == 0
                BW1(i,j) = 0;
            end
        end
    end
    BW2 = icolor(BW1);
    RGB = BW2.*RGB;
    [x1, y1] = find(RGB(:,:,1)<=72 & RGB(:,:,1)>=3 & RGB(:,:,2)>=26 & RGB(:,:,2)<=120 & RGB(:,:,3)>=69 & RGB(:,:,3)<=133);%blue
    if numel(x1) >= 50;
        flavour = 1;
    else
        [x2, y2] = find(RGB(:,:,1)>=122 & RGB(:,:,1)<=161 & RGB(:,:,2)>=75 & RGB(:,:,2)<=126 & RGB(:,:,3)>=4 & RGB(:,:,3)<=60);%orange
        if numel(x2) >= 10;
            flavour = 3;                      
        else
            [x3, y3] = find(RGB(:,:,1)>=33 & RGB(:,:,1)<=85 & RGB(:,:,2)>=69 & RGB(:,:,2)<=117 & RGB(:,:,3)>=1 & RGB(:,:,3)<=34);%green
            if numel(x3) >= 10;
                flavour = 4;
            else
                [x4, y4] = find(RGB(:,:,1)~=0);
                if numel(x4) >= 11520;
                    flavour = 2;
                else
                        flavour = 0;
                end
            end
        end
    end
    if ori >= 0
        if ref >= C(1);
            Angle = ori;
        else
            Angle = ori - pi;
        end
    else
        if ref >= C(1);
            Angle = ori;
        else
            Angle = ori + pi;
        end
    end
%         if Angle <= 0
%             Angle = Angle+pi;
%         else
%             Angle = pi - Angle;
%         end
    x = (C(1) - 794)/1580*1040;
    y = (C(2) - 142)/570*373.6;
    if sqrt(x^2+(y+175)^2)<=548.6
        Reachable = 1;
        Pickable = 1;

    else
        Reachable = 0;
        Pickable = 0;
    end
end

function [flavour, Angle, Reachable, Pickable] = detectDetailsBack(C, ori, ref)
    ori = ori*pi/180;
        flavour = 0;
    if ori >= 0
        if ref >= C(1);
            Angle = ori;
        else
            Angle = ori - pi;
        end
    else
        if ref >= C(1);
            Angle = ori;
        else
            Angle = ori + pi;
        end
    end
%         if Angle <= 0
%             Angle = Angle+pi;
%         else
%             Angle = Angle -pi;
%         end
    x = (C(1) - 794)/1580*1040;
    y = (C(2) - 142)/570*373.6;
    if sqrt(x^2+(y+175)^2)<=548.6
        Reachable = 1;
        Pickable = 1;

    else
        Reachable = 0;
        Pickable = 0;
    end
end





% --- Executes when entered data in editable cell(s) in uitable3.
function uitable3_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to uitable3 (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)
global commandFlag;
tableData = get(hObject,'data');
commandFlag.speed =tableData(1,3);  commandFlag.Vac = tableData(1,2); commandFlag.CD = tableData(1,3);
end


% --- Executes on button press in MT.
function MT_Callback(hObject, eventdata, handles)
% hObject    handle to MT (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of MT

global commandFlag; global tableUpdate;
tableData = get(handles.uitable3,'data');
MoveInstruction = tableData(2,3:12);
    speed = int2str(tableData(1,3));loBuf_X = int2str(MoveInstruction(10)++5000);
    loBuf_Y = int2str(MoveInstruction(9)++5000);loBuf_Z = int2str(MoveInstruction(8)++5000);
    loBuf_J1 = int2str(MoveInstruction(7)+5000);loBuf_J2 = int2str(MoveInstruction(6)+5000);
    loBuf_J3 = int2str(MoveInstruction(5)+5000);loBuf_J4 = int2str(MoveInstruction(4)+5000);
    loBuf_J5 = int2str(MoveInstruction(3)+5000);loBuf_J6 = int2str(MoveInstruction(2)+5000);    
    modeStat = get(handles.LT,'Value');
    commandFlag.stat = get(hObject,'Value');
    stat = get(hObject,'Value');
    speed = format(speed,4);
    while stat
        stat = get(hObject,'Value');
        drawnow;
        if ~modeStat
            buf = (['5000' '5000' '5000' loBuf_Z loBuf_Y loBuf_X speed '03']);        
        else
            buf = ([loBuf_J6 loBuf_J5 loBuf_J4 loBuf_J3 loBuf_J2 loBuf_J1 speed '04']);
        end
        commandStr = format(buf,30);
        commandFlag.m = commandStr;
        data = get(handles.uitable3,'data');
        data(1,4:12) = tableUpdate;
        set(handles.uitable3,'data',data);  
%         pause(0.2);
% disp('MT');
    end
    buf = ['5000' '5000' '5000' '5000' '5000' '5000' speed '00'];
end 


% --- Executes on button press in Ginput_C.
function Ginput_C_Callback(hObject, eventdata, handles)
% hObject    handle to Ginput_C (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global vid; global commandFLag; 
data = get(handles.uitable3,'data');
speed = format(int2str(data(1,3)),4);
stoppreview(vid);
[x,y] = ginput(1);
%output xd,yd
% disp([x,y]);
% [xd,yd] = Pixel2Pose(x,y);
[xd,yd] = Pixel2Pose(500,400);
preview(vid);
% Hint: get(hObject,'Value') returns toggle state of Up
xd = int16(xd); yd = int16(yd);
% disp([xd,yd])
% xd = uint64(xd); yd = uint64(yd);
% disp([xd,yd])
commandStr = ['5000' '5000' '5000' '5000' int2str(yd+5000) int2str(xd+5000) speed '06'];
for s = 1:0.5:3
    commandFLag.m = commandStr;
    pause(0.2);
end
end


% --- Executes on button press in CD.
function CD_Callback(hObject, eventdata, handles)
% hObject    handle to CD (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of CD
% global commandFlag;
% data = get(handles.uitable3,'data');
% commandFlag.stat = get(hObject,'value');
% stat = get(hObject,'value');

global commandFlag;global tableUpdate;
commandFlag.stat = get(hObject,'Value');
stat = get(hObject,'value');
data = get(handles.uitable3,'data');
speed = format(int2str(data(1,3)),4);

        while stat
            stat = get(hObject,'Value'); 
            drawnow; 
            
        if ~get(handles.CD,'Value')
            buf = (['5000' '5000' '5000' '5000' '5000' '5000' '0000' '14']);
            data(1,1) = 0;
            set(handles.uitable3,'data',data);
        else
            buf = (['5000' '5000' '5000' '5000' '5000' '5000' '0001' '14']);
            data(1,1) = 1;
            set(handles.uitable3,'data',data);        
        
        end          
            commandStr = format(buf,30);
            commandFlag.m = commandStr; 

            commandFlag.m = commandStr;
        end

% for s = 1:0.5:5
%     commandFlag.m = commandStr;
% end
end


% --- Executes on button press in VP.
function VP_Callback(hObject, eventdata, handles)
% hObject    handle to VP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of VP
global commandFlag;global tableUpdate;
commandFlag.stat = get(hObject,'Value');
stat = get(hObject,'value');
data = get(handles.uitable3,'data');
speed = format(int2str(data(1,3)),4);
        while stat
            stat = get(hObject,'Value'); 
            drawnow; 
            
        if ~get(handles.VP,'Value')
            buf = (['5000' '5000' '5000' '5000' '5000' '5000' '0000' '11']);
            data(1,2) = 0;
            set(handles.uitable3,'data',data);
        else
            buf = (['5000' '5000' '5000' '5000' '5000' '5000' '0001' '11']);
            data(1,2) = 1;
            set(handles.uitable3,'data',data);        
        
        end          
            commandStr = format(buf,30);
            commandFlag.m = commandStr; 

    commandFlag.m = commandStr;
        end

end
%%%%%%%%%%%%command for conveyor is completed and pls check it%%%%%%%%%%%%%%%%
