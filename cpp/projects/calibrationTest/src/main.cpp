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
		Mat frame1 = imread("C:/Users/Pablo RS/ownCloud/Datasets/StereoTesting/CalibrationImages (Cal_A)/cam1/img_cam1_" + to_string(i) + ".jpg");
		Mat frame2 = imread("C:/Users/Pablo RS/ownCloud/Datasets/StereoTesting/CalibrationImages (Cal_A)/cam2/img_cam2_" + to_string(i) + ".jpg");
		if(frame1.rows == 0 ||frame2.rows == 0)
			break;

		// Add image to list of images for calibration.
		calibrationFrames1.push_back(frame1);
		calibrationFrames2.push_back(frame2);
	}

	StereoCameras stereoCameras("C:/Users/Pablo RS/ownCloud/Datasets/StereoTesting/testImages (Cal_A)/img_cam1_%d.jpg",
								"C:/Users/Pablo RS/ownCloud/Datasets/StereoTesting/testImages (Cal_A)/img_cam2_%d.jpg");
	stereoCameras.calibrate(calibrationFrames1, calibrationFrames2, Size(8,6),108);

	Mat frame1, frame2;

	for (;;) {
		stereoCameras.frames(frame1, frame2, true);
		if(frame1.rows == 0)
			break;

		cvtColor(frame1, frame1, CV_BGR2GRAY);
		cvtColor(frame2, frame2, CV_BGR2GRAY);

		Mat disparity = stereoCameras.disparity(frame2, frame1, 16*12, 21);
		imshow("disparity", disparity);
		
		vector<KeyPoint> keypoints1, keypoints2;
		FAST(frame1, keypoints1, 9);
		FAST(frame2, keypoints2, 9);

		vector<Point2i> points2d1, points2d2;
		for (KeyPoint kp : keypoints1) {
			points2d1.push_back(kp.pt);
		}
		for (KeyPoint kp : keypoints2) {
			points2d2.push_back(kp.pt);
		}


		//vector<Point3f> points3d = stereoCameras.triangulate(points2d1, points2d2);

		Mat display;
		hconcat(frame1, frame2, display);
		imshow("display", display);
		
		waitKey();
	}
}
