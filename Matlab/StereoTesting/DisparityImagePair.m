close all
%load calibrationparametersC.mat

% Read in the stereo pair of images.
I1 = imread('C:\Users\GRVC\Desktop\OnBoardStereoMultiTools_1/cam1_img760.jpg');
I2 = imread('C:\Users\GRVC\Desktop\OnBoardStereoMultiTools_1/cam2_img760.jpg');

% Rectify the images.
%[J1, J2] = rectifyStereoImages(I1, I2, stereoParams);
[J1, J2] = rectifyStereoImages(I1, I2, stereoParams, 'OutputView','full');
J = stereoAnaglyph(J1,J2);

% Display the images before and after rectification.
figure('name' ,'Overlayed stereo images before rectification')
imshow(cat(3, I1(:,:,1), I2(:,:,2:3)), 'InitialMagnification', 50, 'Border','tight');

figure('name', 'Overlayed stereo images after rectification')
imshow(J, 'InitialMagnification', 50, 'Border','tight');

% Calculate disparity map
disparityMap = disparity(rgb2gray(J1),rgb2gray(J2),...
    'BlockSize', 15,'DisparityRange', [-16*12 16*0], 'Method','SemiGlobal');

% change undefined values in map
marker_idx = (disparityMap == -realmax('single'));
      disparityMap(marker_idx) = max(disparityMap(~marker_idx));

% Show map and point cloud
figure('name', 'Disparity map')
imshow(mat2gray(disparityMap),'Border','tight')
colormap jet

point3D = reconstructScene(disparityMap, stereoParams);
% Limit the Z-range for display in mm.
z = point3D(:,:,3);
z(z < 0 | z > 3000) = NaN;
point3D(:,:,3) = z;

figure('name','Point cloud');
showPointCloud(point3D, J1, 'VerticalAxis', 'Y', 'VerticalAxisDir', 'Down');
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
%axis([0 4000 -4000 4000 -4000 4000]);
grid on;