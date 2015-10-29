//
//
//
//
//


#include "Gui.h"
#include "ImageFilteringTools.h"
#include "mainApplication.h"

#include <cassert>
#include <fstream>
#include <iostream>

using namespace cjson;
using namespace cv;
using namespace pcl;
using namespace std;

//---------------------------------------------------------------------------------------------------------------------
MainApplication::MainApplication(int _argc, char ** _argv) {
	bool result = true;
	result &= loadArguments(_argc, _argv);
	result &= initCameras();
	result &= initGui();
	result &= init3dMap();

	assert(result);
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::step() {
	Mat frame1, frame2;
	if(!stepGetImages(frame1, frame2)) return false;

	std::vector<cv::Point3f> points3d;
	if(!stepTriangulatePoints(frame1, frame2, points3d)) return false;

	pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud;
	if(!stepUpdateMap(points3d, newCloud)) return false;

	if(!stepUpdateCameraRotation()) return false;

	if(!stepGetCandidates(newCloud)) return false;

	return true;
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
bool MainApplication::initCameras(){
	mCameras = new StereoCameras(mConfig["cameras"]["left"], mConfig["cameras"]["right"]);
	Json leftRoi = mConfig["cameras"]["leftRoi"];
	Json rightRoi = mConfig["cameras"]["rightRoi"];
	mCameras->roi(Rect(leftRoi[0],leftRoi[1],leftRoi[2],leftRoi[3]), Rect(rightRoi[0],rightRoi[1],rightRoi[2],rightRoi[3]));
	mCameras->load(mConfig["cameras"]["paramFile"]);
	return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::initGui() {
	Gui::init(mConfig["gui"]["name"], *mCameras);
	mGui = Gui::get();
	return mGui != nullptr ? true: false;
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
	params.historySize							= (int) mConfig["mapParams"]["historySize"];			//2;
	params.clusterTolerance						= mConfig["mapParams"]["clusterTolerance"];				//0.035; //tolerance for searching neigbours in clustering. Points further apart will be in different clusters
	params.minClusterSize						= mConfig["mapParams"]["minClusterSize"];				//15;
	params.maxClusterSize						= mConfig["mapParams"]["maxClusterSize"];				//200;
	params.floorDistanceThreshold				= mConfig["mapParams"]["floorDistanceThreshold"];		//0.01;
	params.floorMaxIters						= (int) mConfig["mapParams"]["floorMaxIters"];			//M_PI/180 * 30;

	mMap.params(params);

	return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepGetImages(cv::Mat & _frame1, cv::Mat & _frame2) {
	mCameras->frames(_frame1, _frame2, StereoCameras::eFrameFixing::Undistort);
	if (_frame1.rows == 0)
		return false;

	mGui->updateStereoImages(_frame1, _frame2);
	Rect leftRoi = mCameras->roi(true);
	Rect rightRoi = mCameras->roi(false);
	mGui->drawBox(leftRoi, true, 0,255,0);
	mGui->drawBox(rightRoi, false, 0,255,0);

	cvtColor(_frame1, _frame1, CV_BGR2GRAY);
	cvtColor(_frame2, _frame2, CV_BGR2GRAY);

	bool isBlurry1 = isBlurry(_frame1, mConfig["blurThreshold"]);
	bool isBlurry2 = isBlurry(_frame2, mConfig["blurThreshold"]);
	if(isBlurry1) 
		mGui->putBlurry(true);
	if(isBlurry2) 
		mGui->putBlurry(false);
	
	if(isBlurry1 || isBlurry2)
		return false;

	return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepTriangulatePoints(const cv::Mat &_frame1, const cv::Mat &_frame2, std::vector<cv::Point3f> &_points3d){
	_points3d = mCameras->pointCloud(_frame1, _frame2);	

	return _points3d.size() != 0? true:false;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepUpdateMap(const vector<Point3f> &_points3d, PointCloud<PointXYZ>::Ptr &_cloud){
	for (unsigned i = 0; i < _points3d.size(); i++) {
		if (_points3d[i].x > -3 && _points3d[i].x < 3) {
			if (_points3d[i].y > -3 && _points3d[i].y < 3) {
				if (_points3d[i].z > 0.65 && _points3d[i].z < 1.5) {
					PointXYZ point(_points3d[i].x, _points3d[i].y, _points3d[i].z);
					_cloud->push_back(point);
				}
			}
		}
	}

	mMap.addPoints(_cloud);
	mGui->clearMap();
	mGui->clearPcViewer();
	mGui->drawMap(mMap.cloud().makeShared());
	mGui->addPointToPcViewer(_cloud);
	return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepUpdateCameraRotation() {
	//sorry but I didn't find a better way to transform between cv and eigen, there is a function eigen2cv but I have problems
	Mat R(3,3, CV_64F), T(3,1, CV_64F);
	Eigen::Matrix4f a = mMap.lastView2MapTransformation().inverse();
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

	mCameras->updateGlobalRT(R, T);	
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepGetCandidates(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_newCloud){
	std::vector<pcl::PointIndices> mClusterIndices;
	pcl::PointCloud<PointXYZ>::Ptr currentViewCleanedCloud;
	currentViewCleanedCloud = mMap.voxel(mMap.filter(_newCloud));
	pcl::PointCloud<PointXYZ>::Ptr cloudForProcessing;
	cloudForProcessing = mMap.cloud().makeShared();

	ModelCoefficients plane = mMap.extractFloor(mMap.cloud().makeShared());
	mGui->drawPlane(plane, 0,0,1.5);
	PointCloud<PointXYZ>::Ptr cropedCloud = cloudForProcessing;
	mMap.cropCloud(cropedCloud, plane);
	mClusterIndices = mMap.clusterCloud(cropedCloud);

	std::vector<ObjectCandidate> candidates;
	//create candidates from indices
	for (pcl::PointIndices indices : mClusterIndices)
		candidates.push_back(ObjectCandidate(indices, cloudForProcessing, true));
	//draw all candidates
	for (ObjectCandidate candidate : candidates) {
		if(mMap.distanceToPlane(candidate.cloud(), plane) < 0.05)	// Draw only candidates close to the floor.
			mGui->drawCandidate(candidate);	
	}
}