load testPointClouds

pointCloud1 = pointCloud(pointCloud1);
pointCloud2 = pointCloud(pointCloud2);
pointCloud3 = pointCloud(pointCloud3);
pointCloud4 = pointCloud(pointCloud4);

pointCloud1 = removeInvalidPoints(pointCloud1);
pointCloud2 = removeInvalidPoints(pointCloud2);
pointCloud3 = removeInvalidPoints(pointCloud3);
pointCloud4 = removeInvalidPoints(pointCloud4);

pointCloud1 = pcdenoise(pointCloud1, 'NumNeighbors',3,'Threshold',1);
pointCloud2 = pcdenoise(pointCloud2, 'NumNeighbors',3,'Threshold',1);
pointCloud3 = pcdenoise(pointCloud3, 'NumNeighbors',3,'Threshold',1);
point = pcdenoise(pointCloud4, 'NumNeighbors',3,'Threshold',0.2);
% figure(10)
% showPointCloud(pointC)


figure
showPointCloud(pointCloud1)
title('around 500 points')

figure
showPointCloud(pointCloud2)
title('around 1000 points')

figure
showPointCloud(pointCloud3)
title('around 2000 points')

figure(1)
showPointCloud(pointCloud4)
title('around 3000 points')

tform = pcregrigid((pointCloud1),(pointCloud4))

testPointCloud = pcdownsample(pointCloud4,'random', 0.2);
testPointCloud = pcdownsample(pointCloud4,'gridAverage',50);

figure(2)
showPointCloud(testPointCloud)
title('Just downsampled')

point = pcdenoise(testPointCloud, 'NumNeighbors',50,'Threshold',0.0001);
figure(3)
showPointCloud(point)
title('First downsampled, then denoised')


point2 = pcdenoise(pointCloud4, 'NumNeighbors',100,'Threshold',0.001);
figure(4)
showPointCloud(point2)
title('Just denoised')

figure(5)
showPointCloud(pcdownsample(point2,'gridAverage',50))
title('First denoised, then downsampled')



