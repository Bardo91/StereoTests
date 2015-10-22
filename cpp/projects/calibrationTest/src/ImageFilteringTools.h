//
//
//
//
//


#ifndef IMAGEFILTERINGTOOLS_H_
#define IMAGEFILTERINGTOOLS_H_

#include <opencv2/opencv.hpp>

/// Check if the image is blurry or not.
/// \param _image: image to be checked
///	\param _threshold: threshold of internal blur metric. Must be higher than 0.5 (Large value accept more blur on image).
bool isBlurry(const cv::Mat &_image, double _threshold);



#endif	//	IMAGEFILTERINGTOOLS_H_