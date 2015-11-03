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
MainApplication::MainApplication(int _argc, char ** _argv):mTimePlot("Global Time") {
	bool result = true;
	result &= loadArguments(_argc, _argv);
	result &= initCameras();
	result &= initGui();
	result &= init3dMap();
	mTimer = BOViL::STime::get();

	assert(result);
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::step() {
	Mat frame1, frame2;
	double t0 = mTimer->getTime();
	if(!stepGetImages(frame1, frame2)) return false;
	double t1 = mTimer->getTime();
	PointCloud<PointXYZ>::Ptr cloud;
	if(!stepTriangulatePoints(frame1, frame2, cloud)) return false;
	double t2 = mTimer->getTime();
	if(!stepUpdateMap(cloud)) return false;
	double t3 = mTimer->getTime();
	if(!stepUpdateCameraRotation()) return false;
	double t4 = mTimer->getTime();
	if(!stepGetCandidates()) return false;
	double t5 = mTimer->getTime();

	tGetImages.push_back(t1-t0);
	tTriangulate.push_back(tGetImages[tGetImages.size()-1] + t2-t1);
	tUpdateMap.push_back(tTriangulate[tTriangulate.size()-1] + t3-t2);
	tUpdCam.push_back(tUpdateMap[tUpdateMap.size()-1] + t4-t3);
	tCandidates.push_back(tUpdCam[tUpdCam.size()-1] + t5-t4);

	mTimePlot.clean();
	mTimePlot.draw(tGetImages	, 255, 0, 0,	BOViL::plot::Graph2d::eDrawType::Lines);
	mTimePlot.draw(tTriangulate	, 0, 255, 0,	BOViL::plot::Graph2d::eDrawType::Lines);
	mTimePlot.draw(tUpdateMap, 0, 0, 255,	BOViL::plot::Graph2d::eDrawType::Lines);
	mTimePlot.draw(tUpdCam		, 255, 255, 0,	BOViL::plot::Graph2d::eDrawType::Lines);
	mTimePlot.draw(tCandidates	, 0, 255, 255,	BOViL::plot::Graph2d::eDrawType::Lines);
	mTimePlot.show();
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
		assert(file.is_open());
		return mConfig.parse(file);
	}
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::initCameras(){
	mCameras = new StereoCameras(mConfig["cameras"]["left"], mConfig["cameras"]["right"]);
	Json leftRoi = mConfig["cameras"]["leftRoi"];
	Json rightRoi = mConfig["cameras"]["rightRoi"];
	mCameras->roi(	Rect(leftRoi["x"],leftRoi["y"],leftRoi["width"],leftRoi["height"]), 
					Rect(rightRoi["x"],rightRoi["y"],rightRoi["width"],rightRoi["height"]));
	mCameras->load(mConfig["cameras"]["paramFile"]);
	mCameras->rangeZ(mConfig["cameras"]["pointRanges"]["z"]["min"], mConfig["cameras"]["pointRanges"]["z"]["max"]);
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
	bool isBlurry1, isBlurry2;

	_frame1 = mCameras->camera(0).frame();
	if (_frame1.rows != 0) {
		isBlurry1  = isBlurry(_frame1, mConfig["cameras"]["blurThreshold"]);
	}else{ return false; }
	
	_frame2 = mCameras->camera(1).frame();
	if (_frame2.rows != 0) {
		isBlurry2  = isBlurry(_frame2, mConfig["cameras"]["blurThreshold"]);
	}else{ return false; }

	if (isBlurry1 || isBlurry2) {	// Splitted only for drawing purposes... 666 Dont like it too much.
		mGui->updateStereoImages(_frame1, _frame2);
		
		if(isBlurry1) 
			mGui->putBlurry(true);
		if(isBlurry2) 
			mGui->putBlurry(false);

		Rect leftRoi = mCameras->roi(true);
		Rect rightRoi = mCameras->roi(false);
		mGui->drawBox(leftRoi, true, 0,255,0);
		mGui->drawBox(rightRoi, false, 0,255,0);

		cvtColor(_frame1, _frame1, CV_BGR2GRAY);
		cvtColor(_frame2, _frame2, CV_BGR2GRAY);
		
		return false;
	} else {
		_frame1 = mCameras->camera(0).undistort(_frame1);
		_frame2 = mCameras->camera(1).undistort(_frame2);

		mGui->updateStereoImages(_frame1, _frame2);
		Rect leftRoi = mCameras->roi(true);
		Rect rightRoi = mCameras->roi(false);
		mGui->drawBox(leftRoi, true, 0,255,0);
		mGui->drawBox(rightRoi, false, 0,255,0);

		cvtColor(_frame1, _frame1, CV_BGR2GRAY);
		cvtColor(_frame2, _frame2, CV_BGR2GRAY);
		
		return true;
	}
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepTriangulatePoints(const cv::Mat &_frame1, const cv::Mat &_frame2, PointCloud<PointXYZ>::Ptr &_points3d){
	pair<int,int> disparityRange(mConfig["cameras"]["disparityRange"]["min"], mConfig["cameras"]["disparityRange"]["max"]);
	int squareSize =  mConfig["cameras"]["templateSquareSize"];
	int maxReprojectionError = mConfig["cameras"]["maxReprojectionError"];
	_points3d = mCameras->pointCloud(_frame1, _frame2, disparityRange, squareSize, maxReprojectionError);	

	return _points3d->size() != 0? true:false;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepUpdateMap(const PointCloud<PointXYZ>::Ptr &_cloud){
	mGui->clearMap();
	mGui->clearPcViewer();
	mMap.addPoints(cloud, mMap.Simple);
	mGui->drawMap(mMap.cloud().makeShared());
	mGui->addPointToPcViewer(_cloud);
	mGui->spinOnce();
	return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepUpdateCameraRotation() {
	//sorry but I didn't find a better way to transform between cv and eigen, there is a function eigen2cv but I have problems
	Mat R(3,3, CV_64F), T(3,1, CV_64F);
	Eigen::Matrix4f a = mMap.lastView2MapTransformation().inverse();
	//cout  << "eigen: " << endl << a << endl;
	R.at<double>(0, 0) = a(0, 0);
	R.at<double>(0, 1) = a(0, 1);
	R.at<double>(0, 2) = a(0, 2);
	R.at<double>(1, 0) = a(1, 0);
	R.at<double>(1, 1) = a(1, 1);
	R.at<double>(1, 2) = a(1, 2);
	R.at<double>(2, 0) = a(2, 0);
	R.at<double>(2, 1) = a(2, 1);
	R.at<double>(2, 2) = a(2, 2);
	//cout << "R: " << endl << R << endl;
	T.at<double>(0, 0) = a(0, 3);
	T.at<double>(1, 0) = a(1, 3);
	T.at<double>(2, 0) = a(2, 3);
	//cout << "T: " << endl << T << endl;
	mCameras->updateGlobalRT(R, T);	
	mGui->drawCamera(a.block<3,3>(0,0), a.col(3));

	return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepGetCandidates(){
	ModelCoefficients plane = mMap.extractFloor(mMap.cloud().makeShared());
	if (plane.values.size() == 0)
		return false;
	mGui->drawPlane(plane, 0,0,1.5);
	PointCloud<PointXYZ>::Ptr cropedCloud = mMap.cloud().makeShared();
	mMap.cropCloud(cropedCloud, plane);

	std::vector<pcl::PointIndices> mClusterIndices;
	mClusterIndices = mMap.clusterCloud(cropedCloud);

	std::vector<ObjectCandidate> candidates;
	//create candidates from indices
	for (pcl::PointIndices indices : mClusterIndices)
		candidates.push_back(ObjectCandidate(indices, cropedCloud, true));
	//draw all candidates
	for (ObjectCandidate candidate : candidates) {
		if(mMap.distanceToPlane(candidate.cloud(), plane) < 0.05)	// Draw only candidates close to the floor.
			mGui->drawCandidate(candidate);	
	}
	return true;
}
