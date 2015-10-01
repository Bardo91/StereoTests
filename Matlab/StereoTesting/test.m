load calibrationparameters.mat

I1 = undistortImage( imread('testImages/img_cam1_5.jpg'),stereoParams.CameraParameters1);
I2 = undistortImage( imread('testImages/img_cam2_5.jpg'),stereoParams.CameraParameters2);

% Rectify the images.
%[J1, J2] = rectifyStereoImages(I1, I2, stereoParams);

grayI1 =  rgb2gray(I1);
grayI2 =  rgb2gray(I2);

features1 = detectSURFFeatures(grayI1, 'MetricThreshold', 200);
features2 = detectSURFFeatures(grayI2, 'MetricThreshold', 200);

[ft1,validPts1]  = extractFeatures(grayI1,features1);
[ft2,validPts2] = extractFeatures(grayI2,features2);

indexPairs = matchFeatures(ft1,ft2,'MatchThreshold', 10);


matched1  = validPts1(indexPairs(:,1));
matched2 = validPts2(indexPairs(:,2));

showMatchedFeatures(I1, I2, matched1, matched2, 'montage');
title('Putatively Matched Points (Including Outliers)');

[tform, inlierBoxPoints, inlierScenePoints] = estimateGeometricTransform(matched1, matched2, 'affine', 'Confidence', 80);

figure;
showMatchedFeatures(I1, I2, inlierBoxPoints, inlierScenePoints, 'montage');
title('Matched Points (Inliers Only)');


% figure
% showMatchedFeatures(I1,I2,matched1,matched2)
% title('Candidate matched points (including outliers)')

worldPoins = triangulate(inlierBoxPoints, inlierScenePoints, stereoParams);
% index =1;
% 
% for i=1:size(worldPoins,1)
%     if (abs(worldPoins(i,1)) < 1000)
%         if(abs(worldPoins(i,2)) < 1000)
%             if(abs(worldPoins(i,3)-1800) < 300)
%                 newMatrix(index,:) = worldPoins(i,:);
%                 index = index +1;
%             end
%         end
%     end
% end

figure
plot3(worldPoins(:,1),worldPoins(:,2),worldPoins(:,3),'*');
% 
% figure
% plot3(newMatrix(:,1),newMatrix(:,2),newMatrix(:,3),'*')