load calibrationparametersC.mat

stringLeft = 'cam1_img';
stringRight = 'cam2_img';
workingDir = 'C:\Users\GRVC\Desktop\OnBoardStereoMultiTools_1';

leftNames = dir(fullfile(workingDir,[stringLeft,'*']));
leftNames = {leftNames.name}';

rightNames = dir(fullfile(workingDir,[stringRight,'*']));
rightNames = {rightNames.name}';

% if length(leftNames) ~= length(rightNames)
%     return;
% end

videoPlayer = vision.VideoPlayer('Name','Disparity Map','Position',[30 200 640 480]);
videoPlayer2 = vision.VideoPlayer('Name','LeftImage','Position',[700 200 640 480]);
   
for i=1:length(leftNames)
    %leftImage = imread(fullfile(workingDir,leftNames{i}));
    %rightImage = imread(fullfile(workingDir,rightNames{i}));
    leftImage = imread(fullfile(workingDir,[stringLeft,num2str(i),'.jpg']));
    rightImage = imread(fullfile(workingDir,[stringRight,num2str(i),'.jpg']));
    [leftRectifiedImage, rightRectifiedImage] = rectifyStereoImages(leftImage, rightImage, stereoParams);
    
    disparityMap = disparityCalculationFun(rgb2gray(leftRectifiedImage),rgb2gray(rightRectifiedImage), [-16*12 0]);
    
    step(videoPlayer, mat2gray(disparityMap))
    step(videoPlayer2, leftImage);
 
    
%     figure('name', 'Disparity map')
% imshow(mat2gray(disparityMap),'Border','tight')
% colormap jet
end
    