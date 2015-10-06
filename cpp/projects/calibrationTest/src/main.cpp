///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-10-01
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <opencv2/opencv.hpp>

#include "Camera.h"

using namespace std;
using namespace cv;

int main(int _argc, char** _argv){
	Camera cam(0);

	vector<Mat> calibrationFrames;
	for (unsigned i = 0; i < 21; i++) {
		// Load image
		Mat frame = imread("C:/Users/Pablo RS/ownCloud/Datasets/StereoTesting/CalibrationImagesQuad (Cal_C)/cam1/img_cam1_" + to_string(i) + ".jpg");
		if(frame.rows == 0)
			break;

		// Add image to list of images for calibration.
		calibrationFrames.push_back(frame);
	}

	if(cam.calibrate(calibrationFrames, Size(8,6), 108.0f))
		cam.saveParams("ParamsCal_C");
	else
		std::cout << "Could not calibrate Camera" << std::endl;
	system("PAUSE");
}
