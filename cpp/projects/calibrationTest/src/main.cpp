///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-10-01
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

#include "StereoCameras.h"

using namespace std;
using namespace cv;

void computeFeaturesAndMatches(const Mat &_frame1, const Mat &_frame2, vector< DMatch > &_matches);

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
	stereoCameras.save("stereo_A");


	std::cout << "Calibration Parameters" << std::endl;
	std::cout << stereoCameras.camera(0).matrix() << std::endl;
	std::cout << stereoCameras.camera(0).distCoeffs() << std::endl;
	std::cout << stereoCameras.camera(1).matrix() << std::endl;
	std::cout << stereoCameras.camera(1).distCoeffs() << std::endl;

	Mat frame1, frame2;

	for (;;) {
		stereoCameras.frames(frame1, frame2, true);
		if(frame1.rows == 0)
			break;

		cvtColor(frame1, frame1, CV_BGR2GRAY);
		cvtColor(frame2, frame2, CV_BGR2GRAY);

		Mat disparity = stereoCameras.disparity(frame2, frame1, 16*12, 21);
		imshow("disparity", disparity);
		
		vector< DMatch > matches;
		computeFeaturesAndMatches(frame1, frame2, matches);
		
		Mat display;
		hconcat(frame1, frame2, display);
		imshow("display", display);
		
		waitKey();
	}
}

void computeFeaturesAndMatches(const Mat &_frame1, const Mat &_frame2, vector< DMatch > &_matches) {
	vector<KeyPoint> keypoints1, keypoints2;
	Mat descriptors1, descriptors2;
	Ptr<xfeatures2d::SURF> detector = xfeatures2d::SURF::create();
	detector->detectAndCompute(_frame1, Mat(), keypoints1, descriptors1);
	detector->detectAndCompute(_frame2, Mat(), keypoints2, descriptors2);
	FlannBasedMatcher matcher;
	matcher.match(descriptors1, descriptors2, _matches);

	double max_dist = 0; double min_dist = 100;
	
	//-- Draw only "good" matches
	Mat img_matches;
	drawMatches( _frame1, keypoints1, _frame2, keypoints2, _matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	
	//-- Show detected matches
	imshow( "Good Matches", img_matches );
	for( int i = 0; i < (int)_matches.size(); i++ )	{ 
		printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, _matches[i].queryIdx, _matches[i].trainIdx ); 
	}

}