
%%% Kyle Ramsey and Matthew Van Houten Final Project
%%% Lego Color Sorter
%%% ECE5330/ECE6311



%clear('cam'); % clear the cam object so you can make a new one
%cam = webcam('FaceTime HD Camera (Built-in)'); %open the camera
cam = webcam('HD Webcam C615'); %open the camera

 clear s dcmElbow sm shield a;         % Clears all joint objects
 clear s dcmWaist sm shield a;
 clear s dcmShoulder sm shield a;
 clear s dcmGripper sm shield a;
   a = arduino("COM4",'Mega2560','Libraries','Adafruit\MotorShieldV2'); % Creates arduino object to control arduino board
   shield = addon(a,'Adafruit\MotorShieldV2');      % addon to the arduino object for the motor shield

   dcmWaist = dcmotor(shield,1);        % Object for the waist joint
   dcmShoulder = dcmotor(shield,2);     % object for the shoulder joint
   dcmElbow = dcmotor(shield,3);        % Object for the elbow joint
   dcmGripper = dcmotor(shield,4);      % Object for the gripper joint

   %flags
   inposFlag=0;     % flag for when gripper is in postition to pick up a lego
   goFlag=1;        % flag for the main loop, used to stop program

color = 0;          % variable to determine which mask to use to pick up specific color
fieldtime = 5.2;    % time for how long waist joint should turn after dropping off lego and getting to checkpoint

%%% this is all the color mask for the sticky note, which is used as a
%%% checkpoint to tell when the robot is close to the drop off zones
% Define thresholds for channel 1 based on histogram settings
channel1greenmin = 0.151;
channel1greenmax = 0.202;

% Define thresholds for channel 2 based on histogram settings
channel2greenmin = 0.283;
channel2greenmax = 0.406;

% Define thresholds for channel 3 based on histogram settings
channel3greenmin = 0.727;
channel3greenmax = 0.975;


while(goFlag)
RGB = snapshot(cam);    
%figure(1)
%imshow(RGB) %original
%%%%%%%%%%%%%% FROM colorThresholder %%%%%%%%%%%%%%%%%% 
% I used the Matlab app colorThresholder to find the points for my mask, which is red 
% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

if(color == 0)      % use of color variable to set specific mask
%%%%%Red
blockArea=13000;    % the area of the connected object in the mask to tell when the grippers are in range to grab the lego

% mask for red legos
% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.987;
channel1Max = 0.033;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.744;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.650;
channel3Max = 1.000;

buckettime = 2; % time after reaching checkpoint to get to red drop off zone

elseif(color == 1)  % color check for green msk
%%% Green %%%

blockArea = 8500;      % area for connected object for green lego to pickup
% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.447;
channel1Max = 0.505;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.789;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.528;
channel3Max = 0.692;

buckettime = 4; % time after checkpoint to get to green drop off

elseif(color == 2)  % color check for blue
%%% Blue
blockArea = 12000;  % area of blue object to pickup
% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.540;
channel1Max = 0.614;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.784;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.667;
channel3Max = 1.000;

buckettime = 6; % time after checkpoint to get to blue dropoff

end



% Create mask based on chosen histogram thresholds
sliderBW = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ...
(I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
(I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;
% Initialize output masked image based on input image.
maskedRGBImage = RGB;
% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
%%%%%%%%%%%%% END FROM colorThresholder %%%%%%%%%%%%%%%% 
%figure(2);clf;
%imshow(maskedRGBImage) 

%https://www.mathworks.com/help/images/ref/bwconncomp.html is easier.
CC = bwconncomp(sliderBW);
s = regionprops(CC,'Centroid','Area');
centroids1 = cat(1,s.Centroid);     % gets centroid of object
areas = cat(1,s.Area); % gets area of connected object
[m,ind] = max(areas); % find the largest connected component


while(max(areas) >= 1000) % only tries to pickup legos if it detects one in frame of specific color, otherwise it moves to the next mask and checks again

RGB = snapshot(cam);

I = rgb2hsv(RGB);
% Create mask based on chosen histogram thresholds
sliderBW = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ...
(I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
(I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;
% Initialize output masked image based on input image.
maskedRGBImage = RGB;
% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
%%%%%%%%%%%%% END FROM colorThresholder %%%%%%%%%%%%%%%% 
%figure(2);clf;
%imshow(maskedRGBImage) 

%https://www.mathworks.com/help/images/ref/bwconncomp.html is easier.
CC = bwconncomp(sliderBW);
s = regionprops(CC,'Centroid','Area');
centroids1 = cat(1,s.Centroid);
areas = cat(1,s.Area);
[m,ind] = max(areas); % find the largest connected component
disp("Inside Loop");
figure(3);clf;
imshow(double(sliderBW)) % display masked image
hold on
%plot(centroids(:,1),centroids(:,2),'b*')
% for i = 1:numel(areas) % this shows all the spots
% text(centroids(i,1),centroids(i,2),num2str(areas(i) ),'color','r')
% end
plot(centroids1(ind,1),centroids1(ind,2),'m*','markersize',32) % plots the centroid of the object
% put a star on the largest connected component.
% This is where you should aim your robot.

[rows,columns,layers] = size(sliderBW); % size elements of image
xCenter = columns/2;    % these determine the center x and y value of frame, and puts a cross hair in the middle of screen
yCenter = rows/2;
xline(xCenter, 'LineWidth', 2, 'Color', 'r');
yline(yCenter, 'LineWidth', 2, 'Color', 'r');

xline(centroids1(ind,1), 'LineWidth', 2, 'Color', 'r'); % these plot a cross hair on the centroid of the object, allows visualization of what the mask sees and where to move to center it
yline(centroids1(ind,2), 'LineWidth', 2, 'Color', 'r');

disp(max(areas)); % displays the area of connected object, useful for debugging
if(inposFlag==0) % checks if the gripper is in position to pick up lego, if not it works to get into position
if((centroids1(ind,1)-xCenter)<-30) % this part of if statement checks if object is left of center and moves the arm to left if it is
   dcmWaist.Speed = 0.3;
   start(dcmWaist);
   pause(0.2);
   stop(dcmWaist);
elseif((centroids1(ind,1)-xCenter)>30) % this part checks if the object is to the right, and moves right if so
   dcmWaist.Speed = -0.3;
   start(dcmWaist);
   pause(0.2);
   stop(dcmWaist);
end 

if((centroids1(ind,2)-yCenter)<-30) % checks if object is above center, and moves robot to center with object
    dcmElbow.Speed = 0.35;
    start(dcmElbow);
    pause(0.2);
    stop(dcmElbow);
elseif((centroids1(ind,2)-yCenter)>30) % checks if object is below center, and moves down to center with object
    dcmElbow.Speed = -0.25;
    start(dcmElbow);
    pause(0.2);
    stop(dcmElbow);
end

% Detects if grip is centered and far from block, moves arm closer
if(((centroids1(ind,1)-xCenter)>-50) && ((centroids1(ind,1)-xCenter)<50) ...
        && ((centroids1(ind,2)-yCenter)>-50) && ((centroids1(ind,2)-yCenter)<50))
    
    disp(max(areas)); % we do some debugging
    if(max(areas)>2000 & max(areas)<blockArea) %#ok<AND2> % checks if the area of the block is >2000, this eliminates noise // also has to see that the object is 
        % too far away by checking if the area of objct is less than the
        % minimum area for pickup

        disp("Forward");
        dcmElbow.Speed=0.60;
        dcmShoulder.Speed=0.40; % sets shoulder and elbow joint so that the arm translates forward through a combination of joint movement
        start(dcmElbow);
        start(dcmShoulder);
        pause(0.4);
        stop(dcmElbow);
        stop(dcmShoulder);
    
    end
end
end


% Detects if block is centered and close to gripper, activates pickup
% sequence
if(((centroids1(ind,1)-xCenter)>-80) && ((centroids1(ind,1)-xCenter)<80) ...
        && ((centroids1(ind,2)-yCenter)>-80) && ((centroids1(ind,2)-yCenter)<80)&&(max(areas>blockArea)))

    inposFlag=1; % setting this flag activates the sequence to pickup object and place it in drop off zone
end


if(inposFlag) % beginning of pickup sequence
    disp("Close Grip");
    dcmGripper.Speed = 0.5;
    start(dcmGripper);  % closes the gripper on the lego
    pause(1.2);
    stop(dcmGripper);


    dcmShoulder.Speed=-0.50;
    start(dcmShoulder);         % lift lego up, away from table
    pause(1.5);
    stop(dcmShoulder);


    dcmElbow.Speed=-0.50;
    dcmShoulder.Speed=-0.60;
    start(dcmElbow);            % again uses two joints to move linearly backward to original position
    start(dcmShoulder);
    pause(1.3);
    stop(dcmElbow);
    stop(dcmShoulder);

RGB = snapshot(cam);    % staart taking photos and masking to look for green sticky note, which is used as a checkpoint to tell when robot arm is close to drop off zones
    %figure(1)
    %imshow(RGB)

    I = rgb2hsv(RGB);

    sliderBW = ((I(:,:,1) >= channel1greenmin) | (I(:,:,1) <= channel1greenmax)) & ...      % mask for checkpoint
    (I(:,:,2) >= channel2greenmin) & (I(:,:,2) <= channel2greenmax) & ...
    (I(:,:,3) >= channel3greenmin) & (I(:,:,3) <= channel3greenmax);

    BW = sliderBW;

    maskedRGBImage = RGB;

    maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

    CC = bwconncomp(sliderBW);

    s = regionprops(CC,'Centroid','Area');

    centroids3 = cat(1,s.Centroid);

    areas = cat(1,s.Area);



while(~(((centroids3(ind,1)-xCenter)>-80) && ((centroids3(ind,1)-xCenter)<80)))     % if it does not detect the center of the checkpoint, 
                                                                                    % it keeps moving right to the checkpoint and masking 
                                                                                    % to look for the checkpoint

    disp("Looking For Green");
    disp(max(areas))
    RGB = snapshot(cam);
    %figure(1)
    %imshow(RGB)

    I = rgb2hsv(RGB);

    sliderBW = ((I(:,:,1) >= channel1greenmin) | (I(:,:,1) <= channel1greenmax)) & ...
    (I(:,:,2) >= channel2greenmin) & (I(:,:,2) <= channel2greenmax) & ...
    (I(:,:,3) >= channel3greenmin) & (I(:,:,3) <= channel3greenmax);

    BW = sliderBW;

    maskedRGBImage = RGB;

    maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

    figure(3);clf;

    imshow(maskedRGBImage)

    CC = bwconncomp(sliderBW);

    s = regionprops(CC,'Centroid','Area');

    centroids3 = cat(1,s.Centroid);

    areas = cat(1,s.Area);

    dcmWaist.Speed=-0.45;
    start(dcmWaist);        % moving waist joint to look for checkpoint
    pause(0.2);
    stop(dcmWaist);

end

disp("Green Found");
dcmWaist.Speed = -0.45; % after checkpoint is found, moves to the right for duration specified in the mask selection at top
start(dcmWaist);        % these different times will drop each color in their respective drop off
pause(buckettime);
stop(dcmWaist);

    disp("Open Grip");
    dcmGripper.Speed = -0.5;
    start(dcmGripper);      % drops the lego in its zone
    pause(0.8);
    stop(dcmGripper);

    RGB = snapshot(cam);        % start looking for checkpoint again
    %figure(1)
    %imshow(RGB)

    I = rgb2hsv(RGB);

    sliderBW = ((I(:,:,1) >= channel1greenmin) | (I(:,:,1) <= channel1greenmax)) & ...
    (I(:,:,2) >= channel2greenmin) & (I(:,:,2) <= channel2greenmax) & ...
    (I(:,:,3) >= channel3greenmin) & (I(:,:,3) <= channel3greenmax);

    BW = sliderBW;

    maskedRGBImage = RGB;

    maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

    CC = bwconncomp(sliderBW);

    s = regionprops(CC,'Centroid','Area');

    centroids3 = cat(1,s.Centroid);

    areas = cat(1,s.Area);

while(~(((centroids3(ind,1)-xCenter)>-80) && ((centroids3(ind,1)-xCenter)<80)))     % scans left to find checkpoint again

    disp("Looking For Green GOing back");
    disp(max(areas));
    RGB = snapshot(cam);
    %figure(1)
    %imshow(RGB)

    I = rgb2hsv(RGB);

    sliderBW = ((I(:,:,1) >= channel1greenmin) | (I(:,:,1) <= channel1greenmax)) & ...
    (I(:,:,2) >= channel2greenmin) & (I(:,:,2) <= channel2greenmax) & ...
    (I(:,:,3) >= channel3greenmin) & (I(:,:,3) <= channel3greenmax);

    BW = sliderBW;

    maskedRGBImage = RGB;

    maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

    figure(3);clf;

imshow(maskedRGBImage)

    CC = bwconncomp(sliderBW);

    s = regionprops(CC,'Centroid','Area');

    centroids3 = cat(1,s.Centroid);

    areas = cat(1,s.Area);

    dcmWaist.Speed=0.40;
    start(dcmWaist);
    pause(0.2);     
    stop(dcmWaist);

end

disp("Found Green Again");      % after chceckpoint is found again, arms rotates left for set amount of time to get to pickup zone
dcmWaist.Speed = 0.45;
start(dcmWaist);
pause(fieldtime);
stop(dcmWaist);

    inposFlag=0;        % sets flag that robot needs to try to move into poistion for next lego
    %goFlag=0;
end

RGB = snapshot(cam);        % start looking for next lego again, have to do it here to prevent out of bounds index for plotting centroid

I = rgb2hsv(RGB);
% Create mask based on chosen histogram thresholds
sliderBW = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ...
(I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
(I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;
% Initialize output masked image based on input image.
maskedRGBImage = RGB;
% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;
%%%%%%%%%%%%% END FROM colorThresholder %%%%%%%%%%%%%%%% 
%figure(2);clf;
%imshow(maskedRGBImage) 

%https://www.mathworks.com/help/images/ref/bwconncomp.html is easier.
CC = bwconncomp(sliderBW);
s = regionprops(CC,'Centroid','Area');
centroids2 = cat(1,s.Centroid);
areas = cat(1,s.Area);
[m,ind] = max(areas); % find the largest connected component

disp("Final max:"+max(areas));


end
color = color + 1;      % if a specific color is not detected, it will switch to the next mask and look for a new color

if(color==3)    % if all colors have been run through, it will switch to the first mask to look for new colors again
    color=0;
end


end