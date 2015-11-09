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
	// Calculate horizontal blur
	Mat dx, ddx;
	// Calculate first derivative (edge detector)
	Sobel(_image, dx, CV_16S, 1, 0, 1, 1, 0, cv::BORDER_DEFAULT);
	convertScaleAbs( dx, dx );
	threshold( dx, dx, 20, 255, THRESH_BINARY);

	// Calculate second derivative (zero cross detector)
	//Laplacian(dx, ddx, CV_16S, 1, 1, 0, cv::BORDER_DEFAULT);
	Sobel(dx, ddx, CV_16S, 1, 0, 1, 1, 0, cv::BORDER_DEFAULT);
	convertScaleAbs( ddx, ddx );

	double horiBlurMetric = double(sum(dx)[0]) / sum(ddx)[0];

	// Calculate vertical blur
	Mat dy, ddy, test;
	// Calculate first derivative (edge detector)
	Sobel(_image, dy, CV_16S, 0, 1, 1, 1, 0, cv::BORDER_DEFAULT);
	convertScaleAbs( dy, dy );
	threshold( dy, dy, 20, 255, THRESH_BINARY);
	imshow("first Derivative", dy);
	// Calculate second derivative (zero cross detector)
	//Laplacian(dy, ddy, CV_16S, 1, 1, 0, cv::BORDER_DEFAULT);
	Sobel(dy, ddy, CV_16S, 0, 1, 1, 1, 0, cv::BORDER_DEFAULT);

	convertScaleAbs(ddy, ddy);
	imshow("Second derivative, current implementation", ddy);

	double vertBlurMetric = double(sum(dy)[0]) / sum(ddy)[0];

	double blurriness = horiBlurMetric > vertBlurMetric ? horiBlurMetric : vertBlurMetric;
	cout << blurriness << " ";

	return horiBlurMetric > _threshold || vertBlurMetric > _threshold? true : false;
}

