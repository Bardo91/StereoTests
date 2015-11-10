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
	zeroCrossX(dx, ddx);

	double horiBlurMetric = double(sum(dx)[0]) / sum(ddx)[0];

	// Calculate vertical blur
	Mat dy, ddy, test;
	// Calculate first derivative (edge detector)
	Sobel(_image, dy, CV_16S, 0, 1, 1, 1, 0, cv::BORDER_DEFAULT);
	convertScaleAbs( dy, dy );
	threshold( dy, dy, 20, 255, THRESH_BINARY);
	// Calculate second derivative (zero cross detector)
	zeroCrossY(dy, ddy);

	double vertBlurMetric = double(sum(dy)[0]) / sum(ddy)[0];

	double blurriness = horiBlurMetric > vertBlurMetric ? horiBlurMetric : vertBlurMetric;
	cout << horiBlurMetric << " " << vertBlurMetric << " ";

	return horiBlurMetric > _threshold || vertBlurMetric > _threshold? true : false;
}

void zeroCrossX(const cv::Mat &_binaryImage, cv::Mat &_output)
{
	int m = _binaryImage.rows;
	int n = _binaryImage.cols;
	_output = Mat(m,n, CV_8U);

	for (uint i = 0; i < m; i++)
	{
		for (uint j = 0; j < n - 1; j++)
		{
			if (abs(_binaryImage.at<uchar>(i, j)-_binaryImage.at<uchar>(i, j + 1)) > 0)
			{
				_output.at<uchar>(i, j) = 255;
			} 
			else
			{
				_output.at<uchar>(i, j) = 0;
			}
		}
	}
}


void zeroCrossY(const cv::Mat &_binaryImage, cv::Mat &_output)
{
	int m = _binaryImage.rows;
	int n = _binaryImage.cols;
	_output = Mat(m, n, CV_8U);

	for (uint j = 0; j < n; j++)
	{
		for (uint i = 0; i < m - 1; i++)
		{
			if (abs(_binaryImage.at<uchar>(i, j) - _binaryImage.at<uchar>(i+1, j)) > 0)
			{
				_output.at<uchar>(i, j) = 255;
			}
			else
			{
				_output.at<uchar>(i, j) = 0;
			}
		}
	}
}
