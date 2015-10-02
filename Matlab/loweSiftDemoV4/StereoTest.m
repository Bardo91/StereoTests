load calibrationparameters.mat

I1 =  imread('C:/Users/Pablo RS/ownCloud/Datasets/StereoTesting/testImages/img_cam1_0.jpg');
I2 = imread('C:/Users/Pablo RS/ownCloud/Datasets/StereoTesting/testImages/img_cam2_0.jpg');

% Rectify the images.
[J1, J2] = rectifyStereoImages(I1, I2, stereoParams);

grayI1 =  rgb2gray(J1);
grayI2 =  rgb2gray(J2);


[matches, loc1, loc2] = match(grayI1,grayI2);

% Draw features
dis1 = grayI1;
dis2 = grayI2;

figure;
imshow(dis1);
hold on;
plot(loc1(:,2),loc1(:,1),'*');

figure;
imshow(dis2);
hold on;
plot(loc2(:,2),loc2(:,1),'*');



matched1 = [];
matched2 = [];

for i=1:length(matches(1,:))
    if(matches(i) ~= 0)
       matched1 = [matched1 ; fliplr(loc1(i,1:2))];
       matched2 = [matched2 ; fliplr(loc2(matches(i),1:2))];
    end
end


[tform, inlierBoxPoints, inlierScenePoints] = ...
    estimateGeometricTransform(matched1, matched2,'similarity','confidence',10);

figure
showMatchedFeatures(I1,I2,matched1,matched2)
title('Candidate matched points (including outliers)')


figure;
showMatchedFeatures(I1, I2, inlierBoxPoints, inlierScenePoints, 'montage');
title('Matched Points (Inliers Only)');

worldPoints = triangulate(inlierBoxPoints, inlierScenePoints, stereoParams);

figure;
plot3(worldPoints(:,1),worldPoints(:,2),worldPoints(:,3),'*');
axis([-1000 1000 -1000 1000 -5000 5000]);
title('Unfiltered cloud');

ptCloud = pointCloud(worldPoints);
filteredCloud = pcdenoise(ptCloud);

figure;
plot3(filteredCloud.Location(:,1),filteredCloud.Location(:,2),filteredCloud.Location(:,3),'*');
title('Filtered cloud');
axis([-1000 1000 -1000 1000 -5000 5000]);
% 
% figure
% plot3(newMatrix(:,1),newMatrix(:,2),newMatrix(:,3),'*')