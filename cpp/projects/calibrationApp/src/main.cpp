#include "../../objectsMap/src/vision/StereoCameras.h"

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

using namespace std;

int main(int _argc, char ** _argv) {

	float squareSize = 0.0223;
	cv::Size cornerCount(9, 16);
	string savePath = "tralala";
	StereoCameras cameras("cam1_%d", "cam2_%d");

	vector<cv::Mat> framesLeft, framesRight;

	for (;;)
	{
		cv::Mat left, right;
		cameras.frames(left, right);
		if (left.size != 0)
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