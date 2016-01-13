close all
load calibrationparameters.mat

I1 = imread('testImages/img_cam1_5.jpg');
I2 = imread('testImages/img_cam2_5.jpg');
figure(1)
imshow([I1,I2])


grayI1 =  rgb2gray(undistortImage(I1,stereoParams.CameraParameters1));
grayI2 =  rgb2gray(undistortImage(I2,stereoParams.CameraParameters2));

%%

%features1 = detectSURFFeatures(grayI1, 'MetricThreshold', 200);
featuresHarris = detectHarrisFeatures(grayI1, 'MinQuality', 0.1, 'FilterSize', 3);
featuresMSER = detectMSERFeatures(grayI1, 'ThresholdDelta', 1, 'RegionAreaRange', [5 5000], 'MaxAreaVariation', 0.25);
%features1 = detectFASTFeatures(grayI1);
%features1 = detectBRISKFeatures(grayI1);
features1 = [featuresHarris.Location];% featuresMSER.Location];

fprintf('We found %d features. \n', length(features1));

imshow(grayI1)
hold on
plot(featuresHarris)
plot(featuresMSER)
[featuresP1, featuresP2, similarity] = findAllFeatureMatches(cornerPoints(features1), grayI1, grayI2, stereoParams.FundamentalMatrix);

fprintf('%d stereo matched were found\n', length(featuresP1));

figure
showMatchedFeatures(grayI1,grayI2,featuresP1,featuresP2,'montage')

%% problem: triangulation is not done with rectified images! 
[worldPoins, reprojectionErrors] = triangulate(featuresP1.Location, single(featuresP2), stereoParams);
% Eliminate noisy points
errorDists = max(sqrt(sum(reprojectionErrors .^ 2, 2)), [], 3);
validIdx = errorDists < 2;

points3D = worldPoins(validIdx, :);
validPoints1 = featuresP1.Location(validIdx, :);
validPoints2 = featuresP2(validIdx, :);

figure;
showMatchedFeatures(I1, I2, validPoints1,validPoints2);
title('Matched Features After Removing Noisy Matches');

figure
plot(reprojectionErrors)

figure
plotCamera('Location',[0 0 0],'Orientation',eye(3),'Opacity',0, 'Size', 50, ...
    'Color', 'r', 'Label', '1')
hold on
grid on
plotCamera('Location',stereoParams.TranslationOfCamera2, 'Orientation',...
    stereoParams.RotationOfCamera2, 'Opacity',0, 'Size', 50, ...
    'Color', 'b', 'Label', '2')
showPointCloud(points3D, 'VerticalAxisDir', 'down', 'MarkerSize', 45);
xlabel('x-axis (mm)');
ylabel('y-axis (mm)');
zlabel('z-axis (mm)')
title('Reconstructed Point Cloud');

hold off;

figure
plot3(worldPoins(:,1),worldPoins(:,2),worldPoins(:,3),'*');
xlim([-1000 1000]);
ylim([-1000 1000]);
zlim([0 2000]);