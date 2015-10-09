%% Stereo feature matching using descriptor matches from undistorted images

close all
load calibrationparameters.mat

I1 = imread('testImages/img_cam1_4.jpg');
I2 = imread('testImages/img_cam2_4.jpg');
figure(1)
imshow([I1,I2])

maxReprojectionError = 1; %1 is good for harris
harrisMinQuality = 0.000001; %going lower gives many more features, they are usefull
harrisFilterSize = 3; %3 is minimum and gives the most features
mserThresholdDelta = 1; %(0 100] intensity step size between regions in %. Less gives more regions
mserRegionAreaRange = [5 5000];
mserMaxAreaVariation = 0.4; %(0 1] maximum variation at different intensities. Hiher number gives more regions, but more unstable
minimumSimilarity = 0.9;
NCCWindowSize = 5; %must be odd
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

%we join features into one variable
features1 = [featuresHarrisLeft.Location; featuresMSERLeft.Location];
fprintf('We found %d Harris features (left). \n', length(featuresHarrisLeft));
fprintf('We found %d MSER features (left). \n', length(featuresMSERLeft));


% imshow(grayI1)
% hold on
% plot(featuresMSER)

[matchedLeft, matchedRight, similarity] = findAllFeatureMatches(cornerPoints(features1), grayI1, grayI2, stereoParams.FundamentalMatrix);

fprintf('We found %d feature matches out of total %d. \n', length(matchedLeft),length(features1));

figure(20)
showMatchedFeatures(grayI1, grayI2, matchedLeft, matchedRight, 'montage');
title('Putatively Matched All Points (Including Outliers)');

figure(21)
title('Stereo matching similarity')
plot(similarity)


points3D = [];
%% problem: triangulation is not done with rectified images! 
[worldPoins, reprojectionErrors] = triangulate(matchedLeft.Location, single(matchedRight), stereoParams);
% Eliminate noisy points
errorDists = max(sqrt(sum(reprojectionErrors .^ 2, 2)), [], 3);
validIdx = errorDists < maxReprojectionError;

points3D = [points3D; worldPoins(validIdx, :)];
validPoints1 = matchedLeft(validIdx, :);
validPoints2 = matchedRight(validIdx, :);

figure;
showMatchedFeatures(I1, I2, validPoints1,validPoints2);
title('Matched Features After Removing Noisy Matches');

fprintf('After filtering %d  features are left. \n', length(validPoints1));

figure
title('Reprojection errors')
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