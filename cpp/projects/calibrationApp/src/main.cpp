#include "../../objectsMap/src/vision/StereoCameras.h"

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

using namespace std;

int main(int _argc, char ** _argv) {

	float squareSize = 0.0223;
	cv::Size cornerCount(7, 11); //7 11
	string leftPath = "C:/Users/GRVC/Desktop/CalibrationF/img_cam1_%d.jpg";
	string rightPath = "C:/Users/GRVC/Desktop/CalibrationF/img_cam2_%d.jpg";
	string savePath = "calib";
	StereoCameras cameras(leftPath,rightPath );

	vector<cv::Mat> framesLeft, framesRight;

	for (;;)
	{
		cv::Mat left, right;
		cameras.frames(left, right);
		if (left.cols != 0)
		{
			framesLeft.push_back(left);
			framesRight.push_back(right);
		}
		else
			break;

	}

	cameras.calibrate(framesLeft, framesRight, cornerCount, squareSize);

	cameras.save(savePath);


	return 1;
}