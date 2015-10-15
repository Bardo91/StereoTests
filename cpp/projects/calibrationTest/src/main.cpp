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
#include "TimeTools.h"

#ifdef ENABLE_PCL
	#include <pcl-1.7/pcl/visualization/cloud_viewer.h>
#endif

using namespace std;
using namespace cv;

int main(int _argc, char** _argv) {
	vector<Mat> calibrationFrames1, calibrationFrames2;
	for (unsigned i = 0; true; i++) {
		// Load image
		Mat frame1 = imread("/home/bardo91/programming/Datasets/Calibration_D/Calibration_D/SmallBoard/img_cam1_" + to_string(i) + ".jpg");
		Mat frame2 = imread("/home/bardo91/programming/Datasets/Calibration_D/Calibration_D/SmallBoard/img_cam2_" + to_string(i) + ".jpg");
		if (frame1.rows == 0 || frame2.rows == 0)
			break;

		// Add image to list of images for calibration.
		calibrationFrames1.push_back(frame1);
		calibrationFrames2.push_back(frame2);
	}

	StereoCameras stereoCameras("/home/bardo91/programming/Datasets/Calibration_D/Calibration_D/LargeRandom_highFPS/img_cam1_%d.jpg", "/home/bardo91/programming/Datasets/Calibration_D/Calibration_D/LargeRandom_highFPS/img_cam2_%d.jpg");

	stereoCameras.calibrate(calibrationFrames1, calibrationFrames2, Size(15, 10), 22.3);
	stereoCameras.save("stereo_D");
	//stereoCameras.load("stereo_D");

	#ifdef ENABLE_PCL
		pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	#endif
	Mat frame1, frame2;
	BOViL::STime *timer = BOViL::STime::get();
	for (;;) {
		double t0 = timer->getTime();
		stereoCameras.frames(frame1, frame2, StereoCameras::eFrameFixing::Undistort);
		double t1 = timer->getTime();
		if (frame1.rows == 0)
			break;

		cvtColor(frame1, frame1, CV_BGR2GRAY);
		cvtColor(frame2, frame2, CV_BGR2GRAY);

		double t2 = timer->getTime();
		vector<Point3f> points3d = stereoCameras.pointCloud(frame1, frame2);
		double t3 = timer->getTime();
		std::cout << "Time undistort: \t" << t1 - t0 << "\t FPS: " << 1/(t1-t0) << std::endl;
		std::cout << "Time get point cloud:\t " << t3 - t2 << "\t FPS: " << 1/(t3-t2)<< std::endl;
		if(points3d.size() == 0)
			continue;

		#ifdef ENABLE_PCL
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);  //fill the cloud.

			//double temp_x , temp_y , temp_z;
			for (unsigned i = 0; i < points3d.size(); i++) {
				if (points3d[i].x > -3000 && points3d[i].x < 3000) {
					if (points3d[i].y > -3000 && points3d[i].y < 3000) {
						if (points3d[i].z > 0 && points3d[i].z < 3000) {
							pcl::PointXYZ point(points3d[i].x, points3d[i].y, points3d[i].z);
							cloud->push_back(point);
						}
					}
				}
			}

			viewer.showCloud(cloud);
		#endif

		Mat display;
		hconcat(frame1, frame2, display);
		imshow("display", display);
		waitKey(1);
	}
}
