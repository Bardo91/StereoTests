///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-10-01
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <fstream>
#include "StereoCameras.h"

#include <pcl-1.7/pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace cv;

vector<Point3f> computeFeaturesAndMatches(const Mat &_frame1, const Mat &_frame2, StereoCameras &_cameras, double _maxReprojectionError = 1);

int main(int _argc, char** _argv){
	vector<Mat> calibrationFrames1, calibrationFrames2;
	for (unsigned i = 0; i < 21; i++) {
		// Load image
		Mat frame1 = imread("/home/bardo91/Desktop/CalibrationImages (Cal_A)/cam1/img_cam1_" + to_string(i) + ".jpg");
		Mat frame2 = imread("/home/bardo91/Desktop/CalibrationImages (Cal_A)/cam2/img_cam2_" + to_string(i) + ".jpg");
		if(frame1.rows == 0 ||frame2.rows == 0)
			break;

		// Add image to list of images for calibration.
		calibrationFrames1.push_back(frame1);
		calibrationFrames2.push_back(frame2);
	}

	StereoCameras stereoCameras("/home/bardo91/Desktop/testImages/img_cam1_%d.jpg",
								"/home/bardo91/Desktop/testImages/img_cam2_%d.jpg");
	
	stereoCameras.calibrate(calibrationFrames1, calibrationFrames2, Size(8,6),108);
	stereoCameras.save("stereo_A");
	//stereoCameras.load("Stereo_A");

	std::cout << "Calibration Parameters" << std::endl;
	std::cout << stereoCameras.camera(0).matrix() << std::endl;
	std::cout << stereoCameras.camera(0).distCoeffs() << std::endl;
	std::cout << stereoCameras.camera(1).matrix() << std::endl;
	std::cout << stereoCameras.camera(1).distCoeffs() << std::endl;

	Mat frame1, frame2;

	for (;;) {
		std::cout << "Getting frames" << std::endl;
		stereoCameras.frames(frame1, frame2, true);
		if(frame1.rows == 0)
			break;

		cvtColor(frame1, frame1, CV_BGR2GRAY);
		cvtColor(frame2, frame2, CV_BGR2GRAY);

		//std::cout << "Calculating disparity" << std::endl;
		//Mat disparity = stereoCameras.disparity(frame2, frame1, 16*12, 21);
		//imshow("disparity", disparity);

		std::cout << "Computing new features and triangulating them" << std::endl;
		vector<Point2i> points1, points2;
		vector<Point3f> points3d = computeFeaturesAndMatches(frame1, frame2, stereoCameras, 4);
		Mat display;
		hconcat(frame1, frame2, display);
		imshow("display", display);

		std::cout << "Filling Point cloud" << std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);  //fill the cloud.

		//double temp_x , temp_y , temp_z;
		for (unsigned i = 0; i < points3d.size(); i++)  {
			if(points3d[i].x > -3000 && points3d[i].x < 3000){
				if(points3d[i].y > -3000 && points3d[i].y < 3000){
					if(points3d[i].z > 0 && points3d[i].z < 3000){
						pcl::PointXYZ point(points3d[i].x, points3d[i].y, points3d[i].z);
						cloud->push_back(point);
					}
				}
			}
		}


		std::cout << "Point cloud size "  << cloud->size() << std::endl;
		pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
		viewer.showCloud (cloud);
		std::cout << "Showing pointcloud" << std::endl;

		waitKey();

	}
}

vector<Point3f> computeFeaturesAndMatches(const Mat &_frame1, const Mat &_frame2, StereoCameras &_cameras, double _maxReprojectionError) {
	vector<KeyPoint> keypoints1, keypoints2;
	Mat descriptors1, descriptors2;
	Ptr<FastFeatureDetector> detector = cv::FastFeatureDetector::create();
	detector->detect(_frame1, keypoints1);
	detector->detect(_frame2, keypoints2);

	Ptr<xfeatures2d::SURF> descriptor = xfeatures2d::SURF::create();
	descriptor->compute(_frame1, keypoints1, descriptors1);
	descriptor->compute(_frame2, keypoints2, descriptors2);
	FlannBasedMatcher matcher;
	vector<DMatch> matches;
	matcher.match(descriptors1, descriptors2, matches);
	Mat img_matches;
	drawMatches( _frame1, keypoints1, _frame2, keypoints2, matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
	imshow("matches", img_matches);
	double max_dist = 0; double min_dist = 100;
	vector<Point2i> points2, points1;
	for (unsigned i = 0; i < matches.size(); i++) {
		points1.push_back(keypoints1[matches[i].queryIdx].pt);
		points2.push_back(keypoints2[matches[i].trainIdx].pt);
	}

	vector<Point3f> points3d = _cameras.triangulate(points1, points2);

	vector<Point2f> reprojection1, reprojection2;
	projectPoints(points3d, _cameras.rotation(0), _cameras.traslation(0), _cameras.camera(0).matrix(), _cameras.camera(0).distCoeffs(), reprojection1);
	projectPoints(points3d, _cameras.rotation(1), _cameras.traslation(1), _cameras.camera(1).matrix(), _cameras.camera(1).distCoeffs(), reprojection2);

	vector<Point3f> filteredPoints3d;
	vector<DMatch> fileredMatches;
	for (unsigned i = 0; i < points3d.size(); i++) {
		double rError1 = sqrt(pow(reprojection1[i].x - points1[i].x,2) + pow(reprojection1[i].y - points1[i].y,2));
		double rError2 = sqrt(pow(reprojection2[i].x - points2[i].x,2) + pow(reprojection2[i].y - points2[i].y,2));

		if (rError1 < _maxReprojectionError && rError2 < _maxReprojectionError) {
			filteredPoints3d.push_back(points3d[i]);
			fileredMatches.push_back(matches[i]);
		}
	}

	//-- Draw only "good" matches
	drawMatches( _frame1, keypoints1, _frame2, keypoints2, fileredMatches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

	//-- Show detected matches
	imshow( "Good Matches", img_matches );

	return filteredPoints3d;
}
