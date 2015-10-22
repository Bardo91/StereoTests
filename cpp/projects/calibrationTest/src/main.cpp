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
#include "EnvironmentMap.h"
#include "ImageFilteringTools.h"

#ifdef ENABLE_PCL
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#endif

using namespace std;
using namespace cv;
using namespace pcl;

int main(int _argc, char** _argv) {
	/*vector<Mat> calibrationFrames1, calibrationFrames2;
	for (unsigned i = 0; true; i++) {
		// Load image
		Mat frame1 = imread("C:/programming/Calibration D/Calibration D/SmallBoard/img_cam1_" + to_string(i) + ".jpg");
		Mat frame2 = imread("C:/programming/Calibration D/Calibration D/SmallBoard/img_cam2_" + to_string(i) + ".jpg");
		if (frame1.rows == 0 || frame2.rows == 0)
			break;

		// Add image to list of images for calibration.
		calibrationFrames1.push_back(frame1);
		calibrationFrames2.push_back(frame2);
	}*/

	/*stereoCameras.calibrate(calibrationFrames1, calibrationFrames2, Size(15, 10), 22.3);
	stereoCameras.save("stereo_D");*/
	StereoCameras stereoCameras("C:/programming/Calibration D/Calibration D/LargeRandom_highFPS/img_cam1_%d.jpg", "C:/programming/Calibration D/Calibration D/LargeRandom_highFPS/img_cam2_%d.jpg");
	stereoCameras.load("stereo_D");

#ifdef ENABLE_PCL
	visualization::CloudViewer viewer("Simple Cloud Viewer");
#endif
	Mat frame1, frame2;
	BOViL::STime *timer = BOViL::STime::get();
	EnvironmentMap map3d(30);
	for (;;) {
		double t0 = timer->getTime();
		stereoCameras.frames(frame1, frame2, StereoCameras::eFrameFixing::Undistort);
		double t1 = timer->getTime();
		if (frame1.rows == 0)
			break;

		cvtColor(frame1, frame1, CV_BGR2GRAY);
		cvtColor(frame2, frame2, CV_BGR2GRAY);

		double cBlurThreshold = 0.75;
		bool isBlurry1 = isBlurry(frame1, cBlurThreshold);
		bool isBlurry2 = isBlurry(frame2, cBlurThreshold);
		if (isBlurry1 || isBlurry2) {
			cvtColor(frame1, frame1, CV_GRAY2BGR);
			cvtColor(frame2, frame2, CV_GRAY2BGR);
			if (isBlurry1)
				putText(frame1, "Blurry Image", Point2i(20, 30), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 5);
			if (isBlurry2)
				putText(frame2, "Blurry Image", Point2i(20, 30), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 5);
		}
		else {
			double t2 = timer->getTime();
			vector<Point3f> points3d = stereoCameras.pointCloud(frame1, frame2);
			double t3 = timer->getTime();
			cout << "Time undistort: \t" << t1 - t0 << "\t FPS: " << 1 / (t1 - t0) << endl;
			cout << "Time get point cloud:\t " << t3 - t2 << "\t FPS: " << 1 / (t3 - t2) << endl;
			if (points3d.size() == 0)
				continue;

			pcl::PointCloud<PointXYZ> cloud;
			#ifdef ENABLE_PCL
			//double temp_x , temp_y , temp_z;
			for (unsigned i = 0; i < points3d.size(); i++) {
				if (points3d[i].x > -3000 && points3d[i].x < 3000) {
					if (points3d[i].y > -3000 && points3d[i].y < 3000) {
						if (points3d[i].z > 650 && points3d[i].z < 1500) {
							PointXYZ point(points3d[i].x, points3d[i].y, points3d[i].z);
							cloud.push_back(point);
						}
					}
				}
			}

			map3d.addPoints(cloud.makeShared());
			Eigen::Matrix4f transform;
			transform << 1, 0, 0, 1500,
				0, 0.945518575599317, -0.325568154457157, 0,
				0, 0.325568154457157, 0.945518575599317, 0;
			/*for (uint i = 0; i < 15; i++)
			{
			//transformPointCloud(cloud, cloud, transform);
			//cloud = map3d.voxel(cloud);
			cout << "Voxelated cloud, i:"<< i <<", " << cloud.size() << endl;
			viewer.showCloud(cloud.makeShared());

			}*/

			transformPointCloud(cloud, cloud, transform);
			viewer.showCloud(cloud.makeShared(), "currentCloud");
			viewer.showCloud(map3d.cloud().makeShared(), "map");
			//viewer.showCloud(voxeledCloud.makeShared());
			#endif

		}



		Mat display;
		hconcat(frame1, frame2, display);
		imshow("display", display);
		waitKey(1);
	}
}
