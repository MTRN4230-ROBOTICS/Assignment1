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

% Last Modified by GUIDE v2.5 13-Apr-2015 21:24:46

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
    'Period', 0.1, ...                % Initial period is 1 sec.
    'TimerFcn', @timer_Callback); % Specify callback
% t = timer('TimerFcn',@timer_Callback,'StartDelay',5);
% Update handles structure
guidata(hObject, handles);
% start(timer);
start(handles.t);

end

function timer_Callback(obj,event)
disp('coming to tyree,Ben and Raymond he ti la');

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
data = [0,0,0,0,0];
set(handles.uitable3,'data',data);

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
% Hi, Arthur, i have an idea that I guess the camera might lose signal 
% if the e-stop button is pressed.this function will popup an emergency 
% warning.uncoment previewCamera then u see what'll happen.
% 
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
global vid;
stoppreview(vid);
[x,y] = ginput(1);
%output xd,yd
disp([x,y]);
[xd,yd] = Pixel2Pose(x,y);
preview(vid);
data = get(handles.uitable3,'data');
% Hint: get(hObject,'Value') returns toggle state of Up
speed = uint64(data(5));
xd = int32(xd); yd = int32(yd);
disp([xd,yd])
xd = uint64(negative2positive(xd)); yd = uint64(negative2positive(yd));
disp([xd,yd])
M = int2str(yd*10^10+xd*10^6+speed*100+3);

M = format(M);

disp(M);
% MTRN4230_Client_Sample(M);
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
% --- Executes on button press in JA.
function JA_Callback(hObject, eventdata, handles)
% hObject    handle to JA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of JA

end
% --- Executes on button press in CD.
function CD_Callback(hObject, eventdata, handles)
% hObject    handle to CD (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of CD
stat = get(hObject,'Value');
data = get(handles.uitable3,'data');
if stat == 1
    data(1,3) = ~data(1,3);
    set(handles.uitable3,'data',data);
else
    data(1,3) = ~data(1,3);
    set(handles.uitable3,'data',data);
    
end
end
% --- Executes on button press in VP.
function VP_Callback(hObject, eventdata, handles)
% hObject    handle to VP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of VP
stat = get(hObject,'Value');
data = get(handles.uitable3,'data');
if stat == 1
    data(1,4) = ~data(1,4);
    set(handles.uitable3,'data',data);
else
    data(1,4) = ~data(1,4);
    set(handles.uitable3,'data',data);
end
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
    image_choco = getsnapshot(vid);
    chocolate1 = imrotate(image_choco,180);
    [line1 line2] = part_two(chocolate1);
%     Line1  = line1;
%      line1 = line([0 10],[0 50],'color','r','LineWidth',10);
    set(line1,'Parent',handles.axes1);
%     p=stem(1:10);
%     set(p,'Parent', handles.axes1);    
end

preview(vid);
% Hint: get(hObject,'Value') returns toggle state of TrackChocolate

end

%Chocolate image processing from PSE1-----------------------------------------------
function [line1 line2] = part_two(image)
% chocolate1 = imread('IMG_0014.jpg');
% chocolate2 = imread('IMG_0017.jpg');
% chocolate1 = imrotate(chocolate1,180);
% chocolate2 = imrotate(chocolate2,180);

%isolate bench
image_ = isolate(image);
% chocolate2_ = isolate(chocolate2);
Chocolate = image_;%input Image

[BWchoco1,maskedChoco1]  = createMask(Chocolate);
BWchoco1 = bwareaopen(BWchoco1, 120);%clear noise on the picture
CH = bwconvhull(BWchoco1); %create a convel hull
% figure;imshow(BWchoco1);
centroidChoco1 = regionprops(CH, 'Centroid');
orientation = regionprops(CH,'Orientation');
%get chocolate key points
[p1 p2 p3 p4 p5 p6] = getGlobalPoints(orientation.Orientation,centroidChoco1.Centroid,[80 170]);
p1 = int16(p1); p2 = int16(p2);p3 = int16(p3); p4 = int16(p4);p5 = int16(p5); p6 = int16(p6);
BWregion1 = roipoly(BWchoco1,[p1(1),p2(1),p5(1),p6(1)],[p1(2),p2(2),p5(2),p6(2)]); 
BWregion2 = roipoly(BWchoco1,[p3(1),p4(1),p5(1),p6(1)],[p3(2),p4(2),p5(2),p6(2)]); 
%cut chocolate into 2 regions
region1 = BWregion1.*(BWchoco1);
region2 = BWregion2.*(BWchoco1);
%figure out the heading
if size(find(region1==1)) > size(find(region2==1))
    heading = 1;
else
    heading = 2;
end
%choose right pointing line segments distance
for increment = 0:1:200
lineLength = sqrt((increment*tan(orientation.Orientation*pi/180))^2+(increment)^2);
    if lineLength > 100
    break
    end
end
if heading == 1
    line1 = line([centroidChoco1.Centroid(1) centroidChoco1.Centroid(1)+increment],[centroidChoco1.Centroid(2) centroidChoco1.Centroid(2)-increment*tan(orientation.Orientation*pi/180)],'color','r','LineWidth',4);
else
    line1 = line([centroidChoco1.Centroid(1) centroidChoco1.Centroid(1)-increment],[centroidChoco1.Centroid(2) centroidChoco1.Centroid(2)+increment*tan(orientation.Orientation*pi/180)],'color','r','LineWidth',4);
end
line2 = line([centroidChoco1.Centroid(1) centroidChoco1.Centroid(1)+100],[centroidChoco1.Centroid(2) centroidChoco1.Centroid(2)],'color','r','LineWidth',4);
return;
end




function [BW,maskedRGBImage] = createMask(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder App. The colorspace and
%  minimum/maximum values for each channel of the colorspace were set in the
%  App and result in a binary mask BW and a composite image maskedRGBImage,
%  which shows the original RGB image values under the mask BW.

% Auto-generated by colorThresholder app on 28-Mar-2015
%------------------------------------------------------


% Convert RGB image to chosen color space
I = RGB;

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.000;
channel1Max = 158.000;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.000;
channel2Max = 206.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 57.000;
channel3Max = 255.000;

% Create mask based on chosen histogram thresholds
BW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
end

function result = isolate(bench)
    xx = [1 1600 1600 1]; yy = [963 963 1200 1200];
    result = icolor(uint8(imcomplement(roipoly(bench,xx,yy)))).*bench;
    return
end

function [point1Global,point2Global,point3Global,point4Global,point5Global,point6Global] = getGlobalPoints(orientation,centriod,objectDimension)
    theta = (orientation)*pi/180;
    L = objectDimension(2)/2; W = objectDimension(1)/2;
    point1 = [L -W];
    point2 = [L W];
    point3 = [-L -W];
    point4 = [-L W];
    point5 = [0 W];
    point6 = [0 -W];
    point1Global = [centriod(1)-point1(1)*cos(theta)-sin(theta)*point1(2) centriod(2)+point1(1)*sin(theta)-cos(theta)*point1(2)];
    point2Global = [centriod(1)-point2(1)*cos(theta)-sin(theta)*point2(2) centriod(2)+point2(1)*sin(theta)-cos(theta)*point2(2)];
    point3Global = [centriod(1)-point3(1)*cos(theta)-sin(theta)*point3(2) centriod(2)+point3(1)*sin(theta)-cos(theta)*point3(2)];
    point4Global = [centriod(1)-point4(1)*cos(theta)-sin(theta)*point4(2) centriod(2)+point4(1)*sin(theta)-cos(theta)*point4(2)];
    point5Global = [centriod(1)-point5(1)*cos(theta)-sin(theta)*point5(2) centriod(2)+point5(1)*sin(theta)-cos(theta)*point5(2)];
    point6Global = [centriod(1)-point6(1)*cos(theta)-sin(theta)*point6(2) centriod(2)+point6(1)*sin(theta)-cos(theta)*point6(2)];
    return
end


% --- Executes on button press in Up.
function Up_Callback(hObject, eventdata, handles)
% hObject    handle to Up (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
data = get(handles.uitable3,'data');
% Hint: get(hObject,'Value') returns toggle state of Up
speed = data(5);
stat = get(hObject,'Value');
speed_ = speed*100;
% if (~get(handles.LT,'Value'))
    
M = 1000005+speed_; %0000.0000.0001.0000.05
M = int2str(M);
M = format(M);
% disp(M);
    while stat  
        
        drawnow
        disp(M);
        stat = get(hObject,'Value');
        disp(stat);
%     disp(stat);
    %     MTRN4230_Client_Sample(M);

    end

end

% --- Executes on button press in Down.
function Down_Callback(hObject, eventdata, handles)
% hObject    handle to Down (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Down
data = get(handles.uitable3,'data');
% Hint: get(hObject,'Value') returns toggle state of Up
speed = data(5);
stat = get(hObject,'Value');
speed_ = speed*100;
% if (~get(handles.LT,'Value'))
    
M = 2000005+speed_; %0000.0000.0001.0000.05
M = int2str(M);
M = format(M);
disp(M);
    while stat   
        drawnow;
        disp(M);
        stat = get(hObject,'Value');
        disp(stat);
        %     MTRN4230_Client_Sample(M);

    end
% else
%     %do torsion mode
% end
end

% --- Executes on button press in Left.
function Left_Callback(hObject, eventdata, handles)
% hObject    handle to Left (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Left
data = get(handles.uitable3,'data');
% Hint: get(hObject,'Value') returns toggle state of Up
speed = data(5);
stat = get(hObject,'Value');
speed_ = speed*100;
% if (~get(handles.LT,'Value'))
    
M = 10000000005+speed_; %0000.0000.0001.0000.05
M = int2str(M);
M = format(M);

    while stat   
        drawnow;
        disp(M);
        stat = get(hObject,'Value');
        disp(stat);
    %     MTRN4230_Client_Sample(M);

    end
end

% --- Executes on button press in Right.
function Right_Callback(hObject, eventdata, handles)
% hObject    handle to Right (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Right
data = get(handles.uitable3,'data');
% Hint: get(hObject,'Value') returns toggle state of Up
speed = data(5);
stat = get(hObject,'Value');
speed_ = speed*100;
% if (~get(handles.LT,'Value'))
    
M = 20000000005+speed_; %0000.0000.0001.0000.05
M = int2str(M);
M = format(M);

    while stat   
        drawnow;
        disp(M);
        stat = get(hObject,'Value');
        disp(stat);
    %     MTRN4230_Client_Sample(M);

    end
end

% --- Executes on button press in Zup.
function Zup_Callback(hObject, eventdata, handles)
% hObject    handle to Zup (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Zup
data = get(handles.uitable3,'data');
% Hint: get(hObject,'Value') returns toggle state of Up
speed = data(5);
stat = get(hObject,'Value');
speed_ = speed*100;
M = 100000000000005+speed_; %0000.0000.0001.0000.05
M = int2str(M);
M = format(M);


    while stat   
        drawnow;
        disp(M);
        stat = get(hObject,'Value');
        disp(stat);
    %     MTRN4230_Client_Sample(M);

    end
end
% --- Executes on button press in Zdown.
function Zdown_Callback(hObject, eventdata, handles)
% hObject    handle to Zdown (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of Zdown
data = get(handles.uitable3,'data');
% Hint: get(hObject,'Value') returns toggle state of Up
speed = data(5);
stat = get(hObject,'Value');
speed_ = speed*100; 
M = 200000000000005+speed_; %0000.0000.0001.0000.05
M = int2str(M);
M = format(M);


    while stat   
        drawnow;
        disp(M);
        stat = get(hObject,'Value');
        disp(stat);
    %     MTRN4230_Client_Sample(M);

    end
end

function replaceString = format(str)
if length(str)<30
    zeroString = mat2str(zeros(1,(30-length(str))));
    
    buffer = zeroString(2:length(zeroString)-1);
    replaceString = regexprep(buffer,'[^\w'']','');
    replaceString = strcat(replaceString,str);
    return;
end

end

function result = negative2positive(a)
    if a <0
        buffer = int2str(a);
        disp(length(buffer));
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
% hObject    handle to StopTimer (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
stop(handles.t);

end