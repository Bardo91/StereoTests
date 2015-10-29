//
//
//
//
//


#include "Gui.h"
#include "mainApplication.h"

#include <iostream>
#include <fstream>

using namespace cjson;
using namespace cv;
using namespace std;

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::init(int _argc, char ** _argv) {
	bool result = true;
	result &= loadArguments(_argc, _argv);
	result &= initGui();
	result &= initCameras();
	result &= init3dMap();

	return result;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::step() {
	return false;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::loadArguments(int _argc, char ** _argv) {
	if (_argc != 2) {
		cerr << "Bad input arguments" << endl;
		return false;
	} else {
		ifstream file;
		file.open(string(_argv[1]));
		return mConfig.parse(file);
	}
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::initGui() {
	Gui::init(mConfig["gui"]["name"]);

	return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::initCameras(){
	StereoCameras stereoCameras(mConfig["cameras"]["left"], mConfig["cameras"]["right"]);
	stereoCameras.load(mConfig["cameras"]["paramFile"]);

	return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::init3dMap(){
	EnvironmentMap::Params params;
	params.voxelSize							= mConfig["mapParams"]["voxelSize"];					//0.02;
	params.outlierMeanK							= mConfig["mapParams"]["outlierMeanK"];					//10;
	params.outlierStdDev						= mConfig["mapParams"]["outlierStdDev"];				//0.05;
	params.outlierSetNegative					= (bool) mConfig["mapParams"]["outlierSetNegative"];	//false;
	params.icpMaxTransformationEpsilon			= mConfig["mapParams"]["icpMaxTransformationEpsilon"];	//1e-20; //seems to have no influence, implies local minimum found
	params.icpEuclideanEpsilon					= mConfig["mapParams"]["icpEuclideanEpsilon"];			//1e-20; //seems to have no influence, implies local minimum found
	params.icpMaxIcpIterations					= mConfig["mapParams"]["icpMaxIcpIterations"];			//1000; //no increase in time.. meaning we reach exit condition much sooner
	params.icpMaxCorrespondenceDistance			= mConfig["mapParams"]["icpMaxCorrespondenceDistance"]; //0.1; //had it at 1 meter, now reduced it to 10 cm... results similar
	params.icpMaxCorrDistDownStep				= mConfig["mapParams"]["icpMaxCorrDistDownStep"];		//0.01;
	params.icpMaxCorrDistDownStepIterations		= mConfig["mapParams"]["icpMaxCorrDistDownStepIterations"]; //1;
	params.historySize							= (int) mConfig["mapParams"]["historySize"];			//2;
	params.clusterTolerance						= mConfig["mapParams"]["clusterTolerance"];				//0.035; //tolerance for searching neigbours in clustering. Points further apart will be in different clusters
	params.minClusterSize						= mConfig["mapParams"]["minClusterSize"];				//15;
	params.maxClusterSize						= mConfig["mapParams"]["maxClusterSize"];				//200;
	params.floorDistanceThreshold				= mConfig["mapParams"]["floorDistanceThreshold"];		//0.01;
	params.floorMaxIters						= (int) mConfig["mapParams"]["floorMaxIters"];			//1000;
	params.floorCameraMinAngle					= mConfig["mapParams"]["floorCameraMinAngle"];			//M_PI/180 * 180;
	params.floorCameraMaxAngle					= mConfig["mapParams"]["floorCameraMaxAngle"];			//M_PI/180 * 30;

	mMap.params(params);

	return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepSearchPointsOnImage(vector<Point3f> &_points3d){
	Mat frame1, frame2;
	mCameras.frames(frame1, frame2, StereoCameras::eFrameFixing::Undistort);
	gui->updateStereoImages(frame1, frame2);

	cvtColor(frame1, frame1, CV_BGR2GRAY);
	cvtColor(frame2, frame2, CV_BGR2GRAY);

	double cBlurThreshold = 0.8;
	bool isBlurry1 = isBlurry(frame1, cBlurThreshold);
	bool isBlurry2 = isBlurry(frame2, cBlurThreshold);
	if(isBlurry1) 
		gui->putBlurry(true);
	if(isBlurry2) 
		gui->putBlurry(false);

	if (isBlurry1 || !isBlurry2) 
		return false;

	points3d = stereoCameras.pointCloud(frame1, frame2);	
	if (points3d.size() == 0)
		return false;

	return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepUpdateMap(vector<Point3f> &_points3d, PointCloud<PointXYZ>::Ptr &_cloud){
	PointCloud<PointXYZ> cloud;
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
	gui->drawMap(map3d.cloud().makeShared());
	gui->addPointToPcViewer(cloud.makeShared());
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepGetCandidates(){
	vector<PointIndices> mClusterIndices;
	PointCloud<PointXYZ>::Ptr currentViewCleanedCloud;
	currentViewCleanedCloud = map3d.voxel(map3d.filter(cloud.makeShared()));
	PointCloud<PointXYZ>::Ptr cloudForProcessing;
	bool useMapForClusters = true;
	if(useMapForClusters)
		cloudForProcessing = map3d.cloud().makeShared();
	else
		cloudForProcessing = currentViewCleanedCloud;

	ModelCoefficients plane = map3d.extractFloor(map3d.cloud().makeShared());
	gui->drawPlane(plane, 0,0,1.5);
	PointCloud<PointXYZ>::Ptr cropedCloud = cloudForProcessing;
	map3d.cropCloud(cropedCloud, plane);
	vector<PointCloud<PointXYZ>::Ptr> clusters;
	map3d.clusterCloud(cloudForProcessing, clusters);


	for (PointCloud<PointXYZ>::Ptr cluster : clusters) {
		gui->addCluster(cluster, 3, rand()*255/RAND_MAX, rand()*255/RAND_MAX, rand()*255/RAND_MAX);
		vector<Point3f> clusterPoints;
		if (useMapForClusters) {
			Eigen::Matrix4f invT = map3d.lastView2MapTransformation().inverse();
			PointCloud<PointXYZ> clusterFromCameraView;
			transformPointCloud(*cluster, clusterFromCameraView, invT);
			for (const PointXYZ point : clusterFromCameraView)
				clusterPoints.push_back(Point3f(point.x, point.y, point.z));
		}
		else {
			for (const PointXYZ point : *cluster)
				clusterPoints.push_back(Point3f(point.x, point.y, point.z));
		}

		vector<Point2f> reprojection1, reprojection2;
		projectPoints(clusterPoints, Mat::eye(3, 3, CV_64F), Mat::zeros(3, 1, CV_64F),stereoCameras.camera(0).matrix(), stereoCameras.camera(0).distCoeffs(), reprojection1);
		projectPoints(clusterPoints, stereoCameras.rotation(), stereoCameras.translation(), stereoCameras.camera(1).matrix(), stereoCameras.camera(1).distCoeffs(), reprojection2);
		unsigned r, g, b; r = rand() * 255 / RAND_MAX; g = rand() * 255 / RAND_MAX; b = rand() * 255 / RAND_MAX;
		gui->addCluster(cluster, 3, r, g, b);
		gui->drawPoints(reprojection1, true, r, g, b);
		gui->drawPoints(reprojection2, false, r, g, b);
		// Calculate convexHull
		vector<Point2f> convexHull1, convexHull2;
		convexHull(reprojection1, convexHull1);
		convexHull(reprojection2, convexHull2);
		gui->drawPolygon(convexHull1, true, r, g, b);
		gui->drawPolygon(convexHull2, false, r, g, b);

		cout << "PointCloud representing the Cluster: " << cluster->points.size() << " data points." << endl;
	}
}
}
