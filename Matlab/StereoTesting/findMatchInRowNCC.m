function [ otherImageFeatureLocation, similarity ] = findMatchInRowNCC( feature, featureImage, otherImage )
%findMatchInRowNCC Summary of this function goes here
%   Detailed explanation goes here
column = round(feature(1));
row = round(feature(2));

windowHalfSize = 5;
minimumSimilarity = 0.5;

if(row - windowHalfSize < 1 || row + windowHalfSize > size(otherImage,1) ||...
        column - windowHalfSize < 1 || column + windowHalfSize > size(otherImage,2))
    disp('out of bounds');
    otherImageFeatureLocation = [0, 0];
    similarity = 0;
    return;
end

partOfImage = otherImage(row-windowHalfSize:row+windowHalfSize,:);
template = featureImage(row-windowHalfSize:row+windowHalfSize,column-windowHalfSize:column+windowHalfSize);
C = normxcorr2(template, partOfImage);

similarity = max(C(2*windowHalfSize+1,:));
[ypeak, xpeak] = find(C==similarity);
if similarity > minimumSimilarity
    if xpeak-windowHalfSize > 0
        otherImageFeatureLocation = [xpeak-windowHalfSize, row];
    else
        otherImageFeatureLocation = [1, row];
    end
        
else
    otherImageFeatureLocation = [0,0];
end
end

