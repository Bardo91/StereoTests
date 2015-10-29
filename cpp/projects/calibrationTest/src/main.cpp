///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-10-01
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <opencv2/opencv.hpp>
//#include <opencv2/core/eigen.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <fstream>

#include "StereoCameras.h"
#include "TimeTools.h"
#include "EnvironmentMap.h"
#include "ImageFilteringTools.h"
#include "graph2d.h"
#include "Gui.h"
#include "ObjectCandidate.h"

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>


using namespace std;
using namespace cv;
using namespace pcl;
using namespace BOViL::plot;

int main(int _argc, char** _argv) {
	if (_argc < 2) {
		std::cerr << "Not enough input arguments" << std::endl;
	}

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
	
	StereoCameras stereoCameras(string(_argv[1]) + "LargeRandom_highFPS/img_cam1_%d.jpg", string(_argv[1]) + "LargeRandom_highFPS/img_cam2_%d.jpg");
	stereoCameras.load("stereo_D");
	stereoCameras.roi(Rect(30,0,640-30,480), Rect(0,0,640-30,480));

	Gui::init("My_gui", stereoCameras);
	Gui* gui = Gui::get();
	
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
	params.historySize							= 2;
	params.clusterTolerance						= 0.035; //tolerance for searching neigbours in clustering. Points further apart will be in different clusters
	params.minClusterSize						= 15;
	params.maxClusterSize						= 200;
	params.floorDistanceThreshold				= 0.01;
	params.floorMaxIters						= 1000;
	params.floorCameraMinAngle					= M_PI/180 * 180;
	params.floorCameraMaxAngle					= M_PI/180 * 30;

	vector<double> timePlot;
	Graph2d graph("TimePlot");

	EnvironmentMap map3d(params);
	for (;;) {
		double t0 = timer->getTime();
		stereoCameras.frames(frame1, frame2, StereoCameras::eFrameFixing::Undistort);
		double t1 = timer->getTime();
		gui->updateStereoImages(frame1, frame2);
		
		if (frame1.rows == 0)
			break;

		cvtColor(frame1, frame1, CV_BGR2GRAY);
		cvtColor(frame2, frame2, CV_BGR2GRAY);

		double cBlurThreshold = 0.8;
		bool isBlurry1 = isBlurry(frame1, cBlurThreshold);
		bool isBlurry2 = isBlurry(frame2, cBlurThreshold);
		if(isBlurry1) 
			gui->putBlurry(true);
		if(isBlurry2) 
			gui->putBlurry(false);

		if(!isBlurry1 && !isBlurry2) {
			double t2 = timer->getTime();
			vector<Point3f> points3d = stereoCameras.pointCloud(frame1, frame2);
			double t3 = timer->getTime();
			cout << "Time undistort: \t" << t1 - t0 << "\t FPS: " << 1 / (t1 - t0) << endl;
			cout << "Time get point cloud:\t " << t3 - t2 << "\t FPS: " << 1 / (t3 - t2) << endl;
			if (points3d.size() == 0)
				continue;

			pcl::PointCloud<PointXYZ> cloud;
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
			gui->clearMap();
			gui->clearPcViewer();
			//sorry but I didn't find a better way to transform between cv and eigen, there is a function eigen2cv but I have problems
			Mat R(3,3, CV_64F), T(3,1, CV_64F);
			Eigen::Matrix4f a = map3d.lastView2MapTransformation().inverse();
			cout  << "eigen: " << endl << a << endl;
			R.at<double>(0, 0) = a(0, 0);
			R.at<double>(0, 1) = a(0, 1);
			R.at<double>(0, 2) = a(0, 2);
			R.at<double>(1, 0) = a(1, 0);
			R.at<double>(1, 1) = a(1, 1);
			R.at<double>(1, 2) = a(1, 2);
			R.at<double>(2, 0) = a(2, 0);
			R.at<double>(2, 1) = a(2, 1);
			R.at<double>(2, 2) = a(2, 2);
			cout << "R: " << endl << R << endl;
			T.at<double>(0, 0) = a(0, 3);
			T.at<double>(1, 0) = a(1, 3);
			T.at<double>(2, 0) = a(2, 3);
			cout << "T: " << endl << T << endl;
			//Eigen::Matrix<float, 4, 4> tralala = map3d.lastView2MapTransformation().inverse();
			//eigen2cv(tralala, R);
			
			gui->drawMap(map3d.cloud().makeShared());
			gui->addPointToPcViewer(cloud.makeShared());
			
			std::vector<pcl::PointIndices> mClusterIndices;
			pcl::PointCloud<PointXYZ>::Ptr currentViewCleanedCloud;
			currentViewCleanedCloud = map3d.voxel(map3d.filter(cloud.makeShared()));
			pcl::PointCloud<PointXYZ>::Ptr cloudForProcessing;
			bool useMapForClusters = true;
			if (useMapForClusters) {
				cloudForProcessing = map3d.cloud().makeShared();
				stereoCameras.updateGlobalRT(R, T);
			}
			else {
				cloudForProcessing = currentViewCleanedCloud;
			}

			ModelCoefficients plane = map3d.extractFloor(map3d.cloud().makeShared());
			gui->drawPlane(plane, 0,0,1.5);
			PointCloud<PointXYZ>::Ptr cropedCloud = cloudForProcessing;
			map3d.cropCloud(cropedCloud, plane);
			mClusterIndices = map3d.clusterCloud(cropedCloud);

			std::vector<ObjectCandidate> candidates;
			//create candidates from indices
			for (pcl::PointIndices indices : mClusterIndices)
				candidates.push_back(ObjectCandidate(indices, cloudForProcessing, true));
			//draw all candidates
			for(ObjectCandidate candidate : candidates)
				gui->drawCandidate(candidate);
		}

		double t3 = timer->getTime();
		timePlot.push_back(t3-t0);
		graph.clean();
		graph.draw(timePlot, 0, 0, 255, Graph2d::eDrawType::Lines);
		waitKey(0);
		
	}

	waitKey();
}
