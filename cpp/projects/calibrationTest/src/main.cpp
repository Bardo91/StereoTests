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

vector<Point3f> computeFeaturesAndMatches(const Mat &_frame1,
		const Mat &_frame2, StereoCameras &_cameras,
		double _maxReprojectionError = 1, int _squareSize = 11);

int main(int _argc, char** _argv) {
	vector<Mat> calibrationFrames1, calibrationFrames2;
	for (unsigned i = 0; i < 21; i++) {
		// Load image
		Mat frame1 = imread(
				"/home/bardo91/Desktop/CalibrationImages (Cal_A)/cam1/img_cam1_"
						+ to_string(i) + ".jpg");
		Mat frame2 = imread(
				"/home/bardo91/Desktop/CalibrationImages (Cal_A)/cam2/img_cam2_"
						+ to_string(i) + ".jpg");
		if (frame1.rows == 0 || frame2.rows == 0)
			break;

		// Add image to list of images for calibration.
		calibrationFrames1.push_back(frame1);
		calibrationFrames2.push_back(frame2);
	}

	StereoCameras stereoCameras(
			"/home/bardo91/Desktop/testImages/img_cam1_%d.jpg",
			"/home/bardo91/Desktop/testImages/img_cam2_%d.jpg");

	//stereoCameras.calibrate(calibrationFrames1, calibrationFrames2, Size(8, 6), 108);
	//stereoCameras.save("stereo_A");
	stereoCameras.load("stereo_A");

	std::cout << "Calibration Parameters" << std::endl;
	std::cout << stereoCameras.camera(0).matrix() << std::endl;
	std::cout << stereoCameras.camera(0).distCoeffs() << std::endl;
	std::cout << stereoCameras.camera(1).matrix() << std::endl;
	std::cout << stereoCameras.camera(1).distCoeffs() << std::endl;

	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	Mat frame1, frame2;
	for (;;) {
		std::cout << "Getting frames" << std::endl;
		stereoCameras.frames(frame1, frame2,
				StereoCameras::eFrameFixing::Undistort);
		if (frame1.rows == 0)
			break;

		cvtColor(frame1, frame1, CV_BGR2GRAY);
		cvtColor(frame2, frame2, CV_BGR2GRAY);

		//std::cout << "Calculating disparity" << std::endl;
		//Mat disparity = stereoCameras.disparity(frame2, frame1, 16*12, 21);
		//imshow("disparity", disparity);

		std::cout << "Computing new features and triangulating them"
				<< std::endl;
		vector<Point3f> points3d = computeFeaturesAndMatches(frame1, frame2,stereoCameras, 20);
		if(points3d.size() == 0)
			continue;

		Mat display;
		hconcat(frame1, frame2, display);
		imshow("display", display);

		std::cout << "Filling Point cloud" << std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);  //fill the cloud.

		//double temp_x , temp_y , temp_z;
		for (unsigned i = 0; i < points3d.size(); i++) {
			if (points3d[i].x > -3000 && points3d[i].x < 3000) {
				if (points3d[i].y > -3000 && points3d[i].y < 3000) {
					if (points3d[i].z > 0 && points3d[i].z < 3000) {
						pcl::PointXYZ point(points3d[i].x, points3d[i].y,
								points3d[i].z);
						cloud->push_back(point);
					}
				}
			}
		}

		std::cout << "Point cloud size " << cloud->size() << std::endl;
		viewer.showCloud(cloud);
		std::cout << "Showing pointcloud" << std::endl;

		waitKey();
	}
}

vector<Point3f> computeFeaturesAndMatches(const Mat &_frame1, const Mat &_frame2, StereoCameras &_cameras, double _maxReprojectionError, int _squareSize) {
	assert(_squareSize % 2 == 1);	// Square size need to be odd.
	// Detecting keypoints
	/*vector<KeyPoint> keypoints1;
	Ptr<FastFeatureDetector> detector = cv::FastFeatureDetector::create(12);
	detector->detect(_frame1, keypoints1);
	vector<Point2i> points1;
	for(KeyPoint kp:keypoints1){
		points1.push_back(kp.pt);
	}*/


	vector<Point2i> points1;
	goodFeaturesToTrack(_frame1, points1, 5000, 0.001,0.5);

	std::cout << "Detected " << points1.size() << "Keypoints" << std::endl;

	// Get epipolar lines on first image.
	vector<Vec3f> epilines;
	computeCorrespondEpilines(points1, 1, _cameras.fundamentalMatrix(), epilines);

	// For each epipolar line, do template matching.
	Rect validRegion(-1, _squareSize/2 + 1, _frame2.cols+2, _frame2.rows - _squareSize - 1);
	vector<Point2i> validPoints1, validPoints2;
	for (unsigned i = 0; i < epilines.size(); i++){
		// Ignore keypoints on border
		if(	points1[i].x < _squareSize /2 ||
			points1[i].x > _frame1.cols - _squareSize /2 ||
			points1[i].y < _squareSize /2 ||
			points1[i].y > _frame1.rows - _squareSize /2 ){
			continue;
		}
		std::vector<cv::Vec3f>::const_iterator it = epilines.begin() + i;
		// Compute extremes of epipolar line projection in second image and add half of square size.
		Point2i p1(0, -(*it)[2] / (*it)[1]);
		Point2i p2(_frame2.cols,-((*it)[2] + (*it)[0] * _frame2.cols) / (*it)[1]);

		if(validRegion.contains(p1) && validRegion.contains(p2)){
			if(p1.y < p2.y){
				p1.y -= _squareSize/2;
				p2.y += _squareSize/2;
			}else{
				p1.y += _squareSize/2;
				p2.y -= _squareSize/2;
			}
		}else{
			continue;
		}

		// Compute Template Matching
		Mat corrVal;
		Rect tempRect = Rect(	Point2i(points1[i].x - _squareSize/2, points1[i].y - _squareSize/2),
								Point2i(points1[i].x + _squareSize/2, points1[i].y + _squareSize/2));
		Mat imgTemplate = _frame1(tempRect);
		Mat subImage = _frame2(Rect(p1, p2));
		matchTemplate(subImage, imgTemplate, corrVal, TemplateMatchModes::TM_CCORR_NORMED);

		// GetMax value of correlation.
		double max;
		Point max_loc;
		minMaxLoc(corrVal, NULL, &max, NULL, &max_loc);

		validPoints2.push_back(p1 + max_loc - Point2i(_squareSize/2, _squareSize/2));
		validPoints1.push_back(points1[i]);

		/*Mat displaySubMat = _frame2(Rect(p1, p2)).clone();
		cvtColor(displaySubMat, displaySubMat, CV_GRAY2BGR);
		circle(displaySubMat, max_loc, 3, Scalar(0, 255, 0));

		Mat displayGlobal;
		_frame2.copyTo(displayGlobal);
		cvtColor(displayGlobal, displayGlobal, CV_GRAY2BGR);
		circle(displayGlobal,p1+ max_loc, 3, Scalar(0, 255, 0));

		imshow("temp", _frame1(tempRect));
		imshow("SubMat", displaySubMat);
		imshow("Global", displayGlobal);
		imshow("err", corrVal);
		waitKey();*/
	}

	//------------ DRAW MATCHES
	Mat matchesDisplay;
	hconcat(_frame1, _frame2, matchesDisplay);
	cvtColor(matchesDisplay, matchesDisplay, CV_GRAY2BGR);

	for(unsigned i = 0; i < validPoints1.size(); i++){
		Point2i p1 = validPoints1[i];
		Point2i p2 = validPoints2[i] + Point2i(_frame1.cols, 0);
		Scalar color = Scalar(double(rand())/RAND_MAX * 255, double(rand())/RAND_MAX * 255, double(rand())/RAND_MAX * 255);
		circle(matchesDisplay, p1, 3, color);
		circle(matchesDisplay, p2, 3, color);
		line(matchesDisplay, p1, p2, color);

	}

	imshow("Matches", matchesDisplay);
	//------------

	std::cout << "Matched " << validPoints1.size() << " keypoints " << std::endl;
	// Triangulate points
	vector<Point3f> points3d = _cameras.triangulate(validPoints1, validPoints2);

	std::cout << "Triangulated " << points3d.size() << " points" << std::endl;

	// Reproject points to filter results
	vector<Point2f> reprojection1, reprojection2;
	projectPoints(points3d, Mat::eye(3, 3, CV_64F), Mat::zeros(3, 1, CV_64F), _cameras.camera(0).matrix(), _cameras.camera(0).distCoeffs(), reprojection1);
	projectPoints(points3d, _cameras.rotation(), _cameras.translation(), _cameras.camera(1).matrix(), _cameras.camera(1).distCoeffs(), reprojection2);

	vector<Point2f> filteredPoints1, filteredPoints2;
	vector<Point3f> filteredPoints3d;
	for (unsigned i = 0; i < points3d.size(); i++) {
		double rError1 = sqrt(pow(reprojection1[i].x - validPoints1[i].x, 2)
							+ pow(reprojection1[i].y - validPoints1[i].y, 2));
		double rError2 = sqrt(pow(reprojection2[i].x - validPoints2[i].x, 2)
							+ pow(reprojection2[i].y - validPoints2[i].y, 2));

		if (rError1 < _maxReprojectionError&& rError2 < _maxReprojectionError) {
			filteredPoints3d.push_back(points3d[i]);
			filteredPoints1.push_back(validPoints1[i]);
			filteredPoints2.push_back(validPoints2[i]);
		}
	}

	std::cout << "Filtered " << filteredPoints1.size() << "points using reprojection" << std::endl;

	return filteredPoints3d;
}
