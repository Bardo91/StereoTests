//
//
//
//
//

#include "ImageFilteringTools.h"


using namespace cv;
using namespace std;

//---------------------------------------------------------------------------------------------------------------------
bool isBlurry(const Mat &_image, double _threshold) {
	Mat dx, ddx;
	// Calculate first derivative (edge detector)
	Sobel(_image, dx, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT);
	convertScaleAbs( dx, dx );
	threshold( dx, dx, 50, 255, THRESH_BINARY);

	// Calculate second derivative (zero cross detector)
	Laplacian(dx, ddx, CV_16S, 1, 1, 0, cv::BORDER_DEFAULT);
	convertScaleAbs( ddx, ddx );

	return double(sum(dx)[0]) / sum(ddx)[0] < _threshold;
}

