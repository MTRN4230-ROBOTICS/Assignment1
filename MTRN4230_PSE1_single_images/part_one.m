function part_one
board = imread('board.tif');
hand = imread('hands1.jpg');
football = imread('football.jpg');
onion = imread('onion.png');
%call color thresholder
[BW,maskedimage] = createMask1(board);
%creat a buffer for orignial image
boardGray = rgb2gray(board);
%find background pixels
% turn to 3 pannels
boardGrayIn3panel = icolor(boardGray);
% 50% intensity
boardGrayIn3panel = boardGrayIn3panel.*(0.5);
figure(1)
subplot(4,2,1);imshow(board)
subplot(4,2,2);imshow(draw_n(boardGrayIn3panel,maskedimage,0))

%-------------------------------------------------------------
hand_gray = rgb2gray(hand);
drawOutLine = edge(hand_gray,'sobel');
outLineIn3panel = icolor(drawOutLine);
[x_1,y_1] = find(drawOutLine(:,:) == 1);

subplot(4,2,3);imshow(drawOutLine);
subplot(4,2,4);imshow(draw(hand,outLineIn3panel,1));

%---------------------------------------------------------------
football_gray = rgb2gray(football);
%cut object
x_2 = [47 157 285 255 142 22];
y_2 = [120 42 59 178 243 219];
bw = roipoly(football_gray,x_2,y_2);
bw = uint8(bw);
cutFootball = football_gray.*bw;

%detectSURFFeatures
footballPoints = detectSURFFeatures(cutFootball);
footballPoints_scene = detectSURFFeatures(football_gray);
subplot(4,2,5);
imshow(cutFootball);hold on;
plot(selectStrongest(footballPoints,200));hold off;

subplot(4,2,6);
imshow(football);hold on
plot(selectStrongest(footballPoints_scene,200));hold off;
%extract features descriptors
[footballFeatures,footballPoints] = extractFeatures(cutFootball,footballPoints);
[football_sceneFeatures,footballPoints_scene] = extractFeatures(football_gray,footballPoints_scene);
%match points
footballPairs = matchFeatures(footballFeatures,football_sceneFeatures);
matchedFootballPoints = footballPoints(footballPairs(:,1),:);
matchedFootballPoints_scene = footballPoints_scene(footballPairs(:,2),:);
figure(4);
showMatchedFeatures(cutFootball,football_gray,matchedFootballPoints,matchedFootballPoints_scene,'montage');
[BWonion,MaskedOnionImage] = createMask2(onion);

BW2 = bwareaopen(BWonion, 50);
CH = bwconvhull(BW2);
edgeOfConvhull = edge(CH);
edgeOfOnion = edge(BW2);
drawEdgeOfConvhull = imcomplement(edgeOfConvhull);

drawEdgeOfOnion = icolor(edgeOfOnion);
drawEdgeOfConvhull = icolor(drawEdgeOfConvhull);
drawEdgeOfOnion(:,:,1) = drawEdgeOfOnion(:,:,1).*255;
drawEdgeOfOnion(:,:,2) = drawEdgeOfOnion(:,:,2).*165;
drawEdgeOfOnion(:,:,3) = drawEdgeOfOnion(:,:,3).*0;
figure(1);
subplot(4,2,7);imshow(onion);
result = draw(onion,drawEdgeOfConvhull,0);
result = draw(result,drawEdgeOfOnion,255);
subplot(4,2,8);imshow(result);hold on;
centroid = regionprops(CH, 'Centroid');
plot(centroid.Centroid(1),centroid.Centroid(2),'r*');
hold off;
end
function [BW,maskedRGBImage] = createMask2(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder App. The colorspace and
%  minimum/maximum values for each channel of the colorspace were set in the
%  App and result in a binary mask BW and a composite image maskedRGBImage,
%  which shows the original RGB image values under the mask BW.

% Auto-generated by colorThresholder app on 28-Mar-2015
%------------------------------------------------------


% Convert RGB image to chosen color space
I = rgb2ycbcr(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 96.000;
channel1Max = 228.000;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 97.000;
channel2Max = 122.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 135.000;
channel3Max = 156.000;

% Create mask based on chosen histogram thresholds
BW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
end

function [BW,maskedRGBImage] = createMask1(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder App. The colorspace and
%  minimum/maximum values for each channel of the colorspace were set in the
%  App and result in a binary mask BW and a composite image maskedRGBImage,
%  which shows the original RGB image values under the mask BW.

% Auto-generated by colorThresholder app on 25-Mar-2015
%------------------------------------------------------


% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.566;
channel1Max = 0.774;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.305;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.493;
channel3Max = 0.882;

% Create mask based on chosen histogram thresholds
BW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
end

function result = draw_n(original,replacement,color_ex)
    [xx,yy] = find(replacement(:,:,1) ~= color_ex(1));
%     figure;subplot(1,2,1);imshow(original);
%     subplot(1,2,2);imshow(replacement);
    for index = 1:numel(xx)
        original(xx(index),yy(index),1) = replacement(xx(index),yy(index),1);
        original(xx(index),yy(index),2) = replacement(xx(index),yy(index),2);
        original(xx(index),yy(index),3) = replacement(xx(index),yy(index),3);
    end
    result = original;
    return;
end
function result = draw(original,replacement,color)
    [xx,yy] = find(replacement(:,:,1) == color(1));
    for index = 1:numel(xx)
        original(xx(index),yy(index),1) = replacement(xx(index),yy(index),1);
        original(xx(index),yy(index),2) = replacement(xx(index),yy(index),2);
        original(xx(index),yy(index),3) = replacement(xx(index),yy(index),3);
    end
    result = original;
    return;
end