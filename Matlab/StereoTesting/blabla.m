
stringLeft = 'img_cam1_';
stringRight = 'img_cam2_';
workingDir = 'C:\Users\GRVC\Desktop\Calibration D\LargeRandom_highFPS';

leftNames = dir(fullfile(workingDir,[stringLeft,'*']));
leftNames = {leftNames.name}';

rightNames = dir(fullfile(workingDir,[stringRight,'*']));
rightNames = {rightNames.name}';

% if length(leftNames) ~= length(rightNames)
%     return;
% end

videoPlayer = vision.VideoPlayer('Name','Disparity Map','Position',[30 200 640 480]);
videoPlayer2 = vision.VideoPlayer('Name','LeftImage','Position',[700 200 640 480]);

rightVal2=zeros(length(leftNames),1);
leftVal2=zeros(length(leftNames),1);

for i=1:length(leftNames)

    leftImage = imread(fullfile(workingDir,[stringLeft,num2str(i),'.jpg']));
    rightImage = imread(fullfile(workingDir,[stringRight,num2str(i),'.jpg']));
    %leftVal(i)=CPBD_compute(rgb2gray(leftImage));
    %rightVal(i)=CPBD_compute(rgb2gray(rightImage));
    leftVal2(i)=imageBlurIndex(rgb2gray(leftImage));
    rightVal2(i)=imageBlurIndex(rgb2gray(rightImage));
    %step(videoPlayer, rightImage)
    %step(videoPlayer2, leftImage);
 i
   
end