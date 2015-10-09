function [ otherImageFeatureLocation, similarity ] = findEpipolarMatchNCC( feature, featureImage, otherImage, fundamentalMatrix )
%findMatchInRowNCC Summary of this function goes here
%   Detailed explanation goes here
windowHalfSize = 5;
minimumSimilarity = 0.5;

column = round(feature(1));
row = round(feature(2));

imageWidth = size(otherImage,2);
imageHeight = size(otherImage,1);


line = epipolarLine(fundamentalMatrix, feature);
%I have to create a new matrix that has the width and height difference
%depending on the linexx = [1,imageWidth];
xx = [1,imageWidth];
yy = [1,imageHeight];
y = (line(1).*xx+line(3))./-line(2);
x = (line(2).*yy+line(3))./-line(1);

if min(x)< (windowHalfSize+1) 
    xMin = 1;
else
    xMin = min(x) - windowHalfSize;
end

if max(x) >= (imageWidth - windowHalfSize)
    xMax = imageWidth;
else
    xMax = max(x) + windowHalfSize;
end

if min(y)< (windowHalfSize+1) 
    yMin = 1;
else
    yMin = min(y) - windowHalfSize;
end

if max(y) >= (imageHeight - windowHalfSize)
    yMax = imageHeight;
else
    yMax = max(y) + windowHalfSize;
end

yMin = floor(yMin);
xMin = floor(xMin);
yMax = ceil(yMax);
xMax = ceil(xMax);

%check if the feature in featureImage is to close to the edge to calculate corelation
if(row - windowHalfSize < 1 || row + windowHalfSize > size(otherImage,1) ||...
        column - windowHalfSize < 1 || column + windowHalfSize > size(otherImage,2))
    disp('out of bounds');
    otherImageFeatureLocation = [0, 0];
    similarity = 0;
    return;
end

%check if the epipolar line in otherImage is to close to the edge
if( ((yMax == imageHeight) && (yMax-yMin < 10)) || ((yMin == 1) && (yMax-yMin < 10)))
    disp('epipolar line to close to edge');
    otherImageFeatureLocation = [0, 0];
    similarity = 0;
    return;
end
    
%here I should create a matrix that only has the image values in the
%window around the epipolar line. For now I will assume the lines are
%almost horizontal and take the whole window. More has to change in order
%to compensate for a true general scenario (how to define the
%otherImageFeatureLocation)
%partOfImage = zeros(yMax-yMin,xMax-xMin);
%partOfImage = otherImage(yMin:yMax,xMin:xMax);
%becaue we don't solve second scenario yet
xMin = 1;
xMax = imageWidth;
partOfImage = otherImage(yMin:yMax,xMin:xMax);
if size(partOfImage) == [480 640]
        disp('vertical epipolar line');
    otherImageFeatureLocation = [0, 0];
    similarity = 0;
    return;
end

template = featureImage(row-windowHalfSize:row+windowHalfSize,column-windowHalfSize:column+windowHalfSize);
C = normxcorr2(template, partOfImage);

similarity = max(C(:));
[ypeak, xpeak] = find(C==similarity);
if similarity > minimumSimilarity
    if ((xpeak(1)-windowHalfSize > 0) && (ypeak(1)-windowHalfSize > 0))
        otherImageFeatureLocation = [xMin-1+xpeak(1)-windowHalfSize, yMin-1+ypeak(1)-windowHalfSize];
    else
        otherImageFeatureLocation = [0,0];
        similarity = -1;
        disp('weird unexpected problem, not critical');
    end
        
else
    otherImageFeatureLocation = [0,0];
end
end

