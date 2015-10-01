load calibrationparameters.mat

% Read in the stereo pair of images.
I1 = imread('testImages/img_cam1_6.jpg');
I2 = imread('testImages/img_cam2_6.jpg');

% Rectify the images.
[J1, J2] = rectifyStereoImages(I1, I2, stereoParams);

% Display the images before rectification.
figure; imshow(cat(3, J1(:,:,1), J2(:,:,2:3)), 'InitialMagnification', 50);

disparityMap = disparity(rgb2gray(J1), rgb2gray(J2));
figure; imshow(disparityMap, [0, 64], 'InitialMagnification', 50);

pointCloud = reconstructScene(disparityMap, stereoParams);

z = pointCloud(:,:,3);
z(z < -8000) = NaN;
x = pointCloud(:,:,1);
x(x < -3000 | x > 3000) = NaN;
pointCloud(:,:,3) = z;
pointCloud(:,:,1) = x;

figure
showPointCloud(pointCloud);
title('Point Cloud Visualization')
xlabel('x axis');
ylabel('y axis');
zlabel('z axis');
% set axis properties
%axis([0 4000 -4000 4000 -4000 4000]);
view([-79.5000   50.0000]);
grid on;