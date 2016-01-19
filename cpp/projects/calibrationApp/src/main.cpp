#include <StereoLib/StereoCameras.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

using namespace std;

int main(int _argc, char ** _argv) {

	float squareSize = 0.0415;
	cv::Size cornerCount(7,10); //7 11
	string leftPath = "C:/Users/Pablo RS/ownCloud/Datasets/StereoTesting/Stereo Objects - Cropped sets/set7 - Catec tablecloth gt/calibrationSet/cropped set/cam2 (%d).jpg";
	string rightPath = "C:/Users/Pablo RS/ownCloud/Datasets/StereoTesting/Stereo Objects - Cropped sets/set7 - Catec tablecloth gt/calibrationSet/cropped set/cam1 (%d).jpg";
	string savePath = "calibCatec1";
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