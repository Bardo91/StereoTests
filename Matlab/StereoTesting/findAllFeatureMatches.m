function [ pairedfirstImageFeatures, pairedSecondImageFeatures, similarity ] = findAllFeatureMatches( firstImageFeatures, firstUndistortedGrayImage, secondUndistortedGrayImage, fundamentalMatrix )
%findAllFeatureMatches Summary of this function goes here
%   Detailed explanation goes here
index = 1;
for i=1:length(firstImageFeatures)
    [secondImageFeatures(index,:), similarity(index)] = findEpipolarMatchNCC(firstImageFeatures.Location(index,:),firstUndistortedGrayImage, secondUndistortedGrayImage, fundamentalMatrix);
index = index + 1;
if index == 670
    bla =4;
end
end


pairedSecondImageFeatures = secondImageFeatures(find(secondImageFeatures(:,1)~=0),:);
pairedfirstImageFeatures = firstImageFeatures(find(secondImageFeatures(:,1)~=0),:);