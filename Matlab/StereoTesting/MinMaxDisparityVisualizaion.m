% This code shows how the disparity changes over an area in the world 
% coordinate system. You have to provide the camera matrices for the stereo
% pair, shown in the example from stereoParams returned by the stereo
% calibration. The returned min and max disparity can be used for
% optimisation of stereo matches, but ONLY in the case of approximatelty
% parallel cameras! The area grid is parallel to the left camera and its
% center in the middle of the cameras. 

% define minimum and maximum Z distance from camera
Zmin = 400;
Zmax = 700;

% define the grid half square size (should be different for Zmin and Zmax,
% because of the perspective change)
halfSizeZmin = 500;
halfSizeZmax = 750;

%step size of the grid
step = floor(min([halfSizeZmin, halfSizeZmax])/20);


cameraMatrix1 = cameraMatrix(stereoParams.CameraParameters1, eye(3), [0 0 0]);
cameraMatrix2 = cameraMatrix(stereoParams.CameraParameters2, ...
    stereoParams.RotationOfCamera2, stereoParams.TranslationOfCamera2);

%construction of matrix
gridMin = [];
for i=-stereoParams.TranslationOfCamera2(1)/2-halfSizeZmin:step:-stereoParams.TranslationOfCamera2(1)/2+halfSizeZmin
    for j = -stereoParams.TranslationOfCamera2(2)/2-halfSizeZmin:step:-stereoParams.TranslationOfCamera2(2)/2+halfSizeZmin
        gridMin=[gridMin;i,j,Zmin];
    end
end

gridMax = [];
for i=-stereoParams.TranslationOfCamera2(1)/2-halfSizeZmax:step:-stereoParams.TranslationOfCamera2(1)/2+halfSizeZmax
    for j = -stereoParams.TranslationOfCamera2(2)/2-halfSizeZmax:step:-stereoParams.TranslationOfCamera2(2)/2+halfSizeZmax
        gridMax=[gridMax;i,j,Zmax];
    end
end


gridProjected1=projectW(gridMin,cameraMatrix1);
gridProjected2=projectW(gridMin,cameraMatrix2);

sq=(gridProjected1-gridProjected2).^2;
diff=sqrt(sq(:,1)+sq(:,2));
DispMax = max(diff)
figure
surf(reshape(diff,[size(-stereoParams.TranslationOfCamera2(1)/2-halfSizeZmin:step:-stereoParams.TranslationOfCamera2(1)/2+halfSizeZmin,2),size(-stereoParams.TranslationOfCamera2(2)/2-halfSizeZmin:step:-stereoParams.TranslationOfCamera2(2)/2+halfSizeZmin,2)]))
title('Disparity at minimum distance from camera');

gridProjected1=projectW(gridMax,cameraMatrix1);
gridProjected2=projectW(gridMax,cameraMatrix2);

sq=(gridProjected1-gridProjected2).^2;
diff=sqrt(sq(:,1)+sq(:,2));
DispMin = min(diff)
figure
surf(reshape(diff,[size(-stereoParams.TranslationOfCamera2(1)/2-halfSizeZmax:step:-stereoParams.TranslationOfCamera2(1)/2+halfSizeZmax,2),size(-stereoParams.TranslationOfCamera2(2)/2-halfSizeZmax:step:-stereoParams.TranslationOfCamera2(2)/2+halfSizeZmax,2)]))
title('Disparity at maximum distance from camera');
