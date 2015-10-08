function [ disparityMap ] = disparityCalculationFun( leftRectifiedGrayImage, rightRectifiedGrayImage, disparityRange)
%calculateDisparity

disparityMap = disparity(leftRectifiedGrayImage,rightRectifiedGrayImage,...
    'BlockSize', 15,'DisparityRange', disparityRange, 'Method','SemiGlobal');

% change undefined values in map
marker_idx = (disparityMap == -realmax('single'));
      disparityMap(marker_idx) = max(disparityMap(~marker_idx));


end

