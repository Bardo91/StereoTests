function [ blur ] = imageClurIndex( C )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
MM = edge(C,'Sobel',[],'vertical','nothinning');

edgesWidth = sum(MM(:));
ZZ = edge(MM,'zerocross',[0]);

edgesNum = sum(ZZ(:));
blur = edgesWidth/edgesNum;
figure
% imshow(C)
% figure
% imshow(MM)
% figure
% imshow(ZZ)
end

