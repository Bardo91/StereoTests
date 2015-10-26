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
#include "graph2d.h"

#ifdef ENABLE_PCL
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#endif

using namespace std;
using namespace cv;
using namespace pcl;
using namespace BOViL::plot;

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
	}
	StereoCameras stereoCameras("C:/Users/GRVC/Desktop/Calibration D/LargeRandom_highFPS/img_cam1_%d.jpg", "C:/Users/GRVC/Desktop/Calibration D/LargeRandom_highFPS/img_cam2_%d.jpg");
	stereoCameras.calibrate(calibrationFrames1, calibrationFrames2, Size(15, 10), 0.0223);
	stereoCameras.save("stereo_D");*/
	StereoCameras stereoCameras("C:/Users/GRVC/Desktop/Calibration D/LargeRandom_highFPS/img_cam1_%d.jpg", "C:/Users/GRVC/Desktop/Calibration D/LargeRandom_highFPS/img_cam2_%d.jpg");
	stereoCameras.load("stereo_D");

#ifdef ENABLE_PCL
	visualization::CloudViewer viewer("Simple Cloud Viewer");
#endif
	Mat frame1, frame2;
	BOViL::STime *timer = BOViL::STime::get();
	
	EnvironmentMap::Params params;
	params.voxelSize							= 0.02;
	params.outlierMeanK							= 10;
	params.outlierStdDev						= 0.05;
	params.outlierSetNegative					= false;
	params.icpMaxTransformationEpsilon			= 1e-20; //seems to have no influence, implies local minimum found
	params.icpEuclideanEpsilon					= 1e-20; //seems to have no influence, implies local minimum found
	params.icpMaxIcpIterations					= 1000; //no increase in time.. meaning we reach exit condition much sooner
	params.icpMaxCorrespondenceDistance			= 0.1; //had it at 1 meter, now reduced it to 10 cm... results similar
	params.icpMaxCorrDistDownStep				= 0.01;
	params.icpMaxCorrDistDownStepIterations		= 1;
	params.historySize							= 5;
	params.clusterTolerance						= 0.05; //tolerance for searching neigbours in clustering. Points further apart will be in different clusters
	params.minClusterSize						= 20;
	params.maxClusterSize						= 200;
	vector<double> timePlot;
	Graph2d graph("TimePlot");

	EnvironmentMap map3d(params);
	for (;;) {
		double t0 = timer->getTime();
		stereoCameras.frames(frame1, frame2, StereoCameras::eFrameFixing::Undistort);
		double t1 = timer->getTime();
		if (frame1.rows == 0)
			break;

		cvtColor(frame1, frame1, CV_BGR2GRAY);
		cvtColor(frame2, frame2, CV_BGR2GRAY);

		double cBlurThreshold = 0.8;
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
				if (points3d[i].x > -3 && points3d[i].x < 3) {
					if (points3d[i].y > -3 && points3d[i].y < 3) {
						if (points3d[i].z > 0.65 && points3d[i].z < 1.5) {
							PointXYZ point(points3d[i].x, points3d[i].y, points3d[i].z);
							cloud.push_back(point);
						}
					}
				}
			}

			map3d.addPoints(cloud.makeShared());
			//viewer.showCloud(map3d.cloud().makeShared(), "map");
			std::vector<pcl::PointIndices> mClusterIndices;
			mClusterIndices = map3d.clusterCloud(map3d.cloud().makeShared());


			int j = 0;
			for (std::vector<pcl::PointIndices>::const_iterator it = mClusterIndices.begin(); it != mClusterIndices.end(); ++it)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
				for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
					cloud_cluster->points.push_back(map3d.cloud().points[*pit]); //*
				cloud_cluster->width = cloud_cluster->points.size();
				cloud_cluster->height = 1;
				cloud_cluster->is_dense = true;
				viewer.showCloud(cloud_cluster, "cloud"+to_string(j));

				std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
				j++;
			}
			#endif

		}

		Mat display;
		hconcat(frame1, frame2, display);
		imshow("display", display);
		double t3 = timer->getTime();
		timePlot.push_back(t3-t0);
		graph.clean();
		graph.draw(timePlot, 0, 0, 255, Graph2d::eDrawType::Lines);
		waitKey(0);
	}

	waitKey();
}
