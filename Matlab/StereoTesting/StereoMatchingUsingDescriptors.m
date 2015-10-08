%% Stereo feature matching using descriptor matches from undistorted images

close all
load calibrationparameters.mat

I1 = imread('testImages/img_cam1_4.jpg');
I2 = imread('testImages/img_cam2_4.jpg');
figure(1)
imshow([I1,I2])

maxReprojectionError = 1;
harrisMinQuality = 0.001;
harrisFilterSize = 3;
mserThresholdDelta = 1;
mserRegionAreaRange = [5 5000];
mserMaxAreaVariation = 0.25;
matchingThreshold = 30; %more gives more matches but worse
matchingMaxRatio = 0.9; % [0 1] less give fewer matches but better
minDistanceFromCam = 0;
maxDistanceFromCam = 3000; % in mm



grayI1 =  rgb2gray(undistortImage(I1,stereoParams.CameraParameters1));
grayI2 =  rgb2gray(undistortImage(I2,stereoParams.CameraParameters2));

%%
% featrues in left image
%features1 = detectSURFFeatures(grayI1, 'MetricThreshold', 200);
featuresHarrisLeft = detectHarrisFeatures(grayI1, 'MinQuality', harrisMinQuality, 'FilterSize', harrisFilterSize);
featuresMSERLeft = detectMSERFeatures(grayI1, 'ThresholdDelta', mserThresholdDelta, 'RegionAreaRange', mserRegionAreaRange, 'MaxAreaVariation', mserMaxAreaVariation);
%features1 = detectFASTFeatures(grayI1);
%features1 = detectBRISKFeatures(grayI1);

%features in right image
featuresHarrisRight = detectHarrisFeatures(grayI2, 'MinQuality', harrisMinQuality, 'FilterSize', harrisFilterSize);
featuresMSERRight = detectMSERFeatures(grayI2, 'ThresholdDelta', mserThresholdDelta, 'RegionAreaRange', mserRegionAreaRange, 'MaxAreaVariation', mserMaxAreaVariation);

fprintf('We found %d and %d Harris features (left and right). \n', length(featuresHarrisLeft),length(featuresHarrisRight));
fprintf('We found %d and %d MSER features (left and right). \n', length(featuresMSERLeft),length(featuresMSERRight));


% imshow(grayI1)
% hold on
% plot(featuresMSER)

[ft1,validPts1]  = extractFeatures(grayI1,featuresHarrisLeft);
[ft2,validPts2] = extractFeatures(grayI2,featuresHarrisRight);

indexPairs = matchFeatures(ft1,ft2,'MatchThreshold', matchingThreshold, 'MaxRatio',matchingMaxRatio);


matchedHarris1  = validPts1(indexPairs(:,1));
matchedHarris2 = validPts2(indexPairs(:,2));

fprintf('We found %d Harris feature matches. \n', (length(matchedHarris1)+length(matchedHarris2))/2);

figure(20)
showMatchedFeatures(grayI1, grayI2, matchedHarris1, matchedHarris2, 'montage');
title('Putatively Matched Points (Including Outliers)');


[ft1,validPts1]  = extractFeatures(grayI1,featuresMSERLeft);
[ft2,validPts2] = extractFeatures(grayI2,featuresMSERRight);

indexPairs = matchFeatures(ft1,ft2,'MatchThreshold', matchingThreshold, 'MaxRatio', matchingMaxRatio);


matchedMSER1  = validPts1(indexPairs(:,1));
matchedMSER2 = validPts2(indexPairs(:,2));

fprintf('We found %d MSER feature matches. \n', (length(matchedMSER1)+length(matchedMSER2))/2);

figure(21)
showMatchedFeatures(grayI1, grayI2, matchedMSER1, matchedMSER2, 'montage');
title('Putatively Matched Points (Including Outliers)')


points3D = [];
%% problem: triangulation is not done with rectified images! 
[worldPoins, reprojectionErrors] = triangulate(matchedHarris1, matchedHarris2, stereoParams);
% Eliminate noisy points
errorDists = max(sqrt(sum(reprojectionErrors .^ 2, 2)), [], 3);
validIdx = errorDists < maxReprojectionError;

points3D = [points3D; worldPoins(validIdx, :)];
validPoints1 = matchedHarris1(validIdx, :);
validPoints2 = matchedHarris2(validIdx, :);

figure;
showMatchedFeatures(I1, I2, validPoints1,validPoints2);
title('Matched Features After Removing Noisy Matches');

fprintf('After filtering %d Harris features are left. \n', (length(validPoints1)+length(validPoints2))/2);

figure
plot(reprojectionErrors)

[worldPoins, reprojectionErrors] = triangulate(matchedMSER1, matchedMSER2, stereoParams);
% Eliminate noisy points
errorDists = max(sqrt(sum(reprojectionErrors .^ 2, 2)), [], 3);
validIdx = errorDists < maxReprojectionError;

points3D = [points3D; worldPoins(validIdx, :)];
validPoints1 = matchedMSER1.Location(validIdx, :);
validPoints2 = matchedMSER2(validIdx, :);

figure;
showMatchedFeatures(I1, I2, validPoints1,validPoints2);
title('Matched Features After Removing Noisy Matches');

fprintf('After filtering %d MSER features are left. \n', (length(validPoints1)+length(validPoints2))/2);

figure
plot(reprojectionErrors)

fprintf('We have a total of %d 3D points, ',size(points3D,1));

%filter 3D points
z = points3D(:,3);
z(z < minDistanceFromCam | z > maxDistanceFromCam) = NaN;
points3D(:,3) = z;
fprintf('%d are in the range [%d, %d] mm from the camera. \n\n',sum(~isnan(z)),minDistanceFromCam,maxDistanceFromCam);

figure
plotCamera('Location',[0 0 0],'Orientation',eye(3),'Opacity',0, 'Size', 50, ...
    'Color', 'r', 'Label', '1')
hold on
grid on
plotCamera('Location',stereoParams.TranslationOfCamera2, 'Orientation',...
    stereoParams.RotationOfCamera2, 'Opacity',0, 'Size', 50, ...
    'Color', 'b', 'Label', '2')
showPointCloud(points3D, 'VerticalAxis', 'Y','VerticalAxisDir', 'down', 'MarkerSize', 45);
xlabel('x-axis (mm)');
ylabel('y-axis (mm)');
zlabel('z-axis (mm)')
title('Reconstructed Point Cloud');

hold off;

% figure
% plot3(worldPoins(:,1),worldPoins(:,2),worldPoins(:,3),'*');
% xlim([-1000 1000]);
% ylim([-1000 1000]);
% zlim([0 2000]);