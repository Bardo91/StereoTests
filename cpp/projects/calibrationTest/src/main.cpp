///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-10-01
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <opencv2/opencv.hpp>

#include "StereoCameras.h"

using namespace std;
using namespace cv;

int main(int _argc, char** _argv){
	vector<Mat> calibrationFrames1, calibrationFrames2;
	for (unsigned i = 0; i < 21; i++) {
		// Load image
		Mat frame1 = imread("C:/Users/Pablo RS/ownCloud/Datasets/StereoTesting/CalibrationImagesQuad (Cal_C)/cam1/img_cam1_" + to_string(i) + ".jpg");
		Mat frame2 = imread("C:/Users/Pablo RS/ownCloud/Datasets/StereoTesting/CalibrationImagesQuad (Cal_C)/cam2/img_cam2_" + to_string(i) + ".jpg");
		if(frame1.rows == 0 ||frame2.rows == 0)
			break;

		// Add image to list of images for calibration.
		calibrationFrames1.push_back(frame1);
		calibrationFrames2.push_back(frame2);
	}

	StereoCameras stereoCameras(0,1);
	stereoCameras.calibrate(calibrationFrames1, calibrationFrames2, Size(8,6),108);

	system("PAUSE");
}
