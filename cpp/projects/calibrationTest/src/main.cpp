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
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl\common\transforms.h>
#endif

using namespace std;
using namespace cv;

int main(int _argc, char** _argv) {
	/*vector<Mat> calibrationFrames1, calibrationFrames2;
	for (unsigned i = 0; true; i++) {
		// Load image
		Mat frame1 = imread("C:/Users/GRVC/Desktop/Calibration D/SmallBoard/img_cam1_" + to_string(i) + ".jpg");
		Mat frame2 = imread("C:/Users/GRVC/Desktop/Calibration D/SmallBoard/img_cam2_" + to_string(i) + ".jpg");
		if (frame1.rows == 0 || frame2.rows == 0)
			break;

		// Add image to list of images for calibration.
		calibrationFrames1.push_back(frame1);
		calibrationFrames2.push_back(frame2);
	}*/


	StereoCameras stereoCameras("C:/Users/GRVC/Desktop/Calibration D/LargeRandom_highFPS/img_cam1_%d.jpg", "C:/Users/GRVC/Desktop/Calibration D/LargeRandom_highFPS/img_cam2_%d.jpg");

	/*stereoCameras.calibrate(calibrationFrames1, calibrationFrames2, Size(15, 10), 22.3);
	stereoCameras.save("stereo_D");*/
	stereoCameras.load("stereo_D");

#ifdef ENABLE_PCL
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);  //fill the cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr negativePoints(new pcl::PointCloud<pcl::PointXYZ>);
	sor.setInputCloud(cloud);
	sor.setMeanK(20);
	sor.setStddevMulThresh(0.01);
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
		std::cout << "Time undistort: \t" << t1 - t0 << "\t FPS: " << 1 / (t1 - t0) << std::endl;
		std::cout << "Time get point cloud:\t " << t3 - t2 << "\t FPS: " << 1 / (t3 - t2) << std::endl;
		if (points3d.size() == 0)
			continue;

#ifdef ENABLE_PCL
		//double temp_x , temp_y , temp_z;
		for (unsigned i = 0; i < points3d.size(); i++) {
			if (points3d[i].x > -3000 && points3d[i].x < 3000) {
				if (points3d[i].y > -3000 && points3d[i].y < 3000) {
					if (points3d[i].z > 0 && points3d[i].z < 1500) {
						pcl::PointXYZ point(points3d[i].x, points3d[i].y, points3d[i].z);
						cloud->push_back(point);
					}
				}
			}
		}
		sor.setNegative(false);
		double tFilter0 = timer->getTime();
		sor.filter(*filteredCloud);
		double tFilter = timer->getTime() - tFilter0;
		std::cout << "Filtering time: " << tFilter << ",with N=" << sor.getMeanK() << endl;
		std::cout << "Points filtered: " << cloud->size() - filteredCloud->size() << endl;


		Eigen::Matrix4f transform, transform2;
		transform << 1, 0, 0, 1000,
			0, 1, 0, 0,
			0, 0, 1, 0;
		transform2 = transform;
		transform2(0, 3) = -1000;
		pcl::transformPointCloud(*filteredCloud, *filteredCloud, transform);
		sor.setNegative(true);
		sor.filter(*negativePoints);
		pcl::transformPointCloud(*negativePoints, *negativePoints, transform2);
		viewer.showCloud(cloud, "cloud");
		viewer.showCloud(filteredCloud, "filtered");
		viewer.showCloud(negativePoints, "negative");
		cloud->clear();
#endif

		Mat display;
		hconcat(frame1, frame2, display);
		imshow("display", display);
		waitKey();
	}
}
