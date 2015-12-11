//
//
//
//
//


#include "utils/gui/Gui.h"
#include "mainApplication.h"

#include <cassert>
#include <fstream>
#include <iostream>

#include <StereoLib/ImageFilteringTools.h>

#include <implementations/sensors/ImuSimulatorSensor.h>
#include <implementations/sensors/MavrosSensor.h>

using namespace cjson;
using namespace cv;
using namespace pcl;
using namespace std;
using namespace Eigen;

//---------------------------------------------------------------------------------------------------------------------
MainApplication::MainApplication(int _argc, char ** _argv):mTimePlot("Global Time") {
	bool result = true;
	result &= loadArguments(_argc, _argv);
	result &= initCameras();
	result &= initGui();
	result &= init3dMap();
	result &= initRecognitionSystem();
	result &= initLoadGt();
	result &= initImuAndEkf();

	mTimer = BOViL::STime::get();

	if (result) {
		std::cout << "Main application configured and initiallized" << std::endl;
	}
	else {
		std::cerr << "Main application could not be neither configured and initialized" <<std::endl;
	}
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::step() {
	Mat frame1, frame2;
	ImuData imuData;
	double t0 = mTimer->getTime();
	if(!stepGetImuData(imuData)) return false;
	if(!stepGetImages(frame1, frame2)) return false;
	double t1 = mTimer->getTime();
	PointCloud<PointXYZ>::Ptr cloud;
	if(!stepTriangulatePoints(frame1, frame2, cloud)) return false;
	double t2 = mTimer->getTime();
	Vector4f position;
	Quaternion<float> orientation;
	if(!stepEkf(imuData, position, orientation)) return false;
	double t3 = mTimer->getTime();
	if(!stepUpdateMap(cloud, position, orientation)) return false;
	double t4 = mTimer->getTime();
	if(!stepUpdateCameraPose()) return false;
	double t5 = mTimer->getTime();
	if(!stepGetCandidates()) return false;
	double t6 = mTimer->getTime();
	if(!stepCathegorizeCandidates(mCandidates, frame1, frame2)) return false;
	double t7 = mTimer->getTime();
	if (!stepCheckGroundTruth()) return false;


	tGetImages.push_back(t1-t0);
	tTriangulate.push_back(tGetImages[tGetImages.size()-1] + t2-t1);
	tUpdateMap.push_back(tTriangulate[tTriangulate.size()-1] + t4-t3);
	tUpdCam.push_back(tUpdateMap[tUpdateMap.size()-1] + t5-t4);
	tCandidates.push_back(tUpdCam[tUpdCam.size()-1] + t6-t5);
	tCathegorize.push_back(tCandidates[tCandidates.size()-1] + t7-t6);


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
// Private Interface
//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::loadArguments(int _argc, char ** _argv) {
	if (_argc != 2) {
		cerr << "Bad input arguments" << endl;
		return false;
	}
	else {
		ifstream file;
		file.open(string(_argv[1]));
		if (file.is_open())
			return mConfig.parse(file);
		else {
			std::cout << "Can't open main configuration file" << std::endl;
			return false;
		}
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
	params.clusterTolerance						= mConfig["mapParams"]["clusterAffiliationMaxDistance"];				//0.035; //tolerance for searching neigbours in clustering. Points further apart will be in different clusters
	params.minClusterSize						= mConfig["mapParams"]["minClusterSize"];				//15;
	params.maxClusterSize						= mConfig["mapParams"]["maxClusterSize"];				//200;
	params.floorDistanceThreshold				= mConfig["mapParams"]["floorDistanceThreshold"];		//0.01;
	params.floorMaxIters						= (int) mConfig["mapParams"]["floorMaxIters"];			//M_PI/180 * 30;

	mMap.params(params);

	return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::initRecognitionSystem() {
	if (mConfig.contains("recognitionSystem")) {
		mRecognitionSystem = new RecognitionSystem(mConfig["recognitionSystem"]);
		return true;
	}else
		return false;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::initImuAndEkf() {
	if (mConfig.contains("ekf")){
		if (mConfig["ekf"]["source"] == "file") {
			mImu = new ImuSimulatorSensor(mConfig["ekf"]["path"]);
			Eigen::MatrixXd Q;
			array<double, 12> qArray;
			for (int i = 0; i < mConfig["ekf"]["Q"].size(); i++) {
				qArray[i] = mConfig["ekf"]["Q"](i);
			}
			Q = Eigen::Matrix<double,12,1>(qArray.data()).asDiagonal();

			Eigen::MatrixXd R;
			array<double, 6> rArray;
			for (int i = 0; i < mConfig["ekf"]["R"].size(); i++) {
				rArray[i] = mConfig["ekf"]["R"](i);
			}
			R = Eigen::Matrix<double,6,1>(rArray.data()).asDiagonal();

			Eigen::MatrixXd x0;
			array<double, 12> x0Array;
			for (int i = 0; i < mConfig["ekf"]["x0"].size(); i++) {
				x0Array[i] = mConfig["ekf"]["x0"](i);
			}
			x0 = Eigen::Matrix<double,12,1>(x0Array.data());

			mEkf.setUpEKF(Q, R, x0);

			Eigen::MatrixXd sf, C1, C2, T;
			array<double, 3> sfArray, c1Array,c2Array, tArray;

			for (int i = 0; i < mConfig["ekf"]["bias"]["ScaleFactor"].size(); i++) { sfArray[i] = mConfig["ekf"]["bias"]["ScaleFactor"](i); }
			for (int i = 0; i < mConfig["ekf"]["bias"]["C1"].size(); i++) { c1Array[i] = mConfig["ekf"]["bias"]["C1"](i); }
			for (int i = 0; i < mConfig["ekf"]["bias"]["C2"].size(); i++) { c2Array[i] = mConfig["ekf"]["bias"]["C2"](i); }
			for (int i = 0; i < mConfig["ekf"]["bias"]["T"].size(); i++) { tArray[i] = mConfig["ekf"]["bias"]["T"](i); }
			
			sf = Eigen::Matrix<double,3,1>(sfArray.data());
			C1 = Eigen::Matrix<double,3,1>(c1Array.data());
			C2 = Eigen::Matrix<double,3,1>(c2Array.data());
			T = Eigen::Matrix<double,3,1>(tArray.data());
			
			mEkf.parameters(sf,  C1, C2, T);

			mImu2CamT = AngleAxisf		(float(mConfig["ekf"]["Imu2Cam"]["x"])/180.0*M_PI, Vector3f::UnitX())
						* AngleAxisf	(float(mConfig["ekf"]["Imu2Cam"]["y"])/180.0*M_PI,  Vector3f::UnitY())
						* AngleAxisf	(float(mConfig["ekf"]["Imu2Cam"]["z"])/180.0*M_PI, Vector3f::UnitZ());
			
			mGravityOffImuSys = calculateGravityOffset();

			return true;
		}
		else if (mConfig["ekf"]["source"] == "real") {
			// 666 TODO: not defined yet

			//Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(12, 12)*0.01;
			//Eigen::MatrixXd R = Eigen::MatrixXd::Identity(6, 6)*0.01;
			//R.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3)*0.1;
			//Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(12, 1);
			//x0.block<3, 1>(9, 0) = q*linAcc - gravity;
			//
			//mEkf.setUpEKF(Q, R, x0);
			//mEkf.parameters({ 0,0,0 }, { -1,-1,-1 }, q*linAcc - gravity, { 0.3,0.7,0.5 });
			return true;
		}
		else {
			return false;
		}
	}
	else
		return false;
}

Eigen::Vector3f MainApplication::calculateGravityOffset() {
	Eigen::Vector3f gravity = Eigen::Vector3f::Zero();
	
	// calculate offset.
	int nSamples = mConfig["ekf"]["nSamplesForOffSet"];
	for (int i = 0;i < nSamples; i++) {
		ImuData imuData = mImu->get();
	
		Eigen::Quaternion<float> q(imuData.mQuaternion[3], imuData.mQuaternion[0], imuData.mQuaternion[1], imuData.mQuaternion[2]);
		Eigen::Vector3f linAcc, angSpeed;
		linAcc << imuData.mLinearAcc[0],  imuData.mLinearAcc[1], imuData.mLinearAcc[2];
	
		gravity += q*linAcc;
	}
	
	gravity = gravity / nSamples;
	
	return gravity;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::initLoadGt() {
	if (mConfig.contains("groundTruth") && mConfig["groundTruth"].contains("path")) {
		ifstream file;
		file.open(mConfig["groundTruth"]["path"]);
		assert(file.is_open());
		Json gtCandidates;
		if (gtCandidates.parse(file)) {
			unsigned numObjs = int(gtCandidates["metadata"]["numObjects"]);
			unsigned numLabels = int(gtCandidates["metadata"]["numLabels"]);

			Json mArrayObjs = gtCandidates["data"];
			for (unsigned i = 0; i < numObjs; i++) {
				vector<double> probs(numLabels, 0);
				probs[int(mArrayObjs(i)["label"])] = 1.0;

				PointXYZ point( mArrayObjs(i)["position"]["x"],
								mArrayObjs(i)["position"]["y"],
								mArrayObjs(i)["position"]["z"]);

				PointCloud<PointXYZ> cloud;
				cloud.push_back(point);

				ObjectCandidate candidate(cloud.makeShared());
				candidate.addView(Mat(), probs);
				mCandidateGroundTruth.push_back(candidate);
			}
			return true;
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepGetImages(Mat & _frame1, Mat & _frame2) {
	Mat gray1, gray2;
	bool isBlurry1, isBlurry2;
	cout << "Blurriness: ";
	_frame1 = mCameras->camera(0).frame();
	cvtColor(_frame1, gray1, CV_BGR2GRAY);
	if (_frame1.rows != 0) {
		isBlurry1 = isBlurry(gray1, mConfig["cameras"]["blurThreshold"]);
	}
	else { return false; }

	_frame2 = mCameras->camera(1).frame();
	cvtColor(_frame2, gray2, CV_BGR2GRAY);
	if (_frame2.rows != 0) {
		isBlurry2 = isBlurry(gray2, mConfig["cameras"]["blurThreshold"]);
	}
	else { return false; }
	cout << endl;

	if (isBlurry1 || isBlurry2) {
		mGui->updateStereoImages(_frame1, _frame2);

		if(isBlurry1)
			mGui->putBlurry(true);
		if(isBlurry2) 
			mGui->putBlurry(false);

		Rect leftRoi = mCameras->roi(true);
		Rect rightRoi = mCameras->roi(false);
		mGui->drawBox(leftRoi, true, 0,255,0);
		mGui->drawBox(rightRoi, false, 0,255,0);

		
		return false;
	} else {
		_frame1 = mCameras->camera(0).undistort(_frame1);
		_frame2 = mCameras->camera(1).undistort(_frame2);

		mGui->updateStereoImages(_frame1, _frame2);
		Rect leftRoi = mCameras->roi(true);
		Rect rightRoi = mCameras->roi(false);
		mGui->drawBox(leftRoi, true, 0,255,0);
		mGui->drawBox(rightRoi, false, 0,255,0);

// 		_frame1 = gray1; //not correct because they are distorted images
// 		_frame2 = gray2;
		cvtColor(_frame1, _frame1, CV_BGR2GRAY);
		cvtColor(_frame2, _frame2, CV_BGR2GRAY);
		
		return true;
	}
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepGetImuData(ImuData &_imuData) {
	_imuData = mImu->get();

	if (mPreviousTime == -1) {	// Initialization.
		mPreviousTime = _imuData.mTimeSpan - 0.01;
	}

	return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepTriangulatePoints(const Mat &_frame1, const Mat &_frame2, PointCloud<PointXYZ>::Ptr &_points3d){
	pair<int,int> disparityRange(mConfig["cameras"]["disparityRange"]["min"], mConfig["cameras"]["disparityRange"]["max"]);
	int squareSize =  mConfig["cameras"]["templateSquareSize"];
	int maxReprojectionError = mConfig["cameras"]["maxReprojectionError"];
	_points3d = mCameras->pointCloud(_frame1, _frame2, disparityRange, squareSize, maxReprojectionError);	

	return _points3d->size() != 0? true:false;
}

bool MainApplication::stepEkf(const ImuData & _imuData, Eigen::Vector4f &_position, Eigen::Quaternion<float> &_quaternion) {

	// Get and adapt imu data.
	Eigen::Quaternion<float> q(_imuData.mQuaternion[3], _imuData.mQuaternion[0], _imuData.mQuaternion[1], _imuData.mQuaternion[2]);
	Eigen::Matrix<float,3,1> linAcc;
	linAcc << _imuData.mLinearAcc[0],  _imuData.mLinearAcc[1], _imuData.mLinearAcc[2];
	linAcc = mImu2CamT*(q*linAcc - mGravityOffImuSys);
	
	// Create observable state variable vetor.
	Eigen::MatrixXf zk(6,1);
	zk << mMap.cloud().sensor_origin_, linAcc;

	// Update EKF.
	mEkf.stepEKF(zk.cast<double>(), _imuData.mTimeSpan - mPreviousTime);
	mPreviousTime = _imuData.mTimeSpan;
	
	// Save state.
	Eigen::Matrix<float,12,1> state = mEkf.getStateVector().cast<float>();
	_position << state.block<3,1>(0,0), 1;
	_quaternion = q;

	return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepUpdateMap(const PointCloud<PointXYZ>::Ptr &_cloud, const Vector4f &_translationPrediction, const Quaternionf &_qRotationPrediction){
	mGui->clearMap();
	mGui->clearPcViewer();
	auto rotatedCloud = mMap.addPoints(_cloud, _translationPrediction, _qRotationPrediction, mMap.Simple);
	if(rotatedCloud->size() != 0)
		Gui::get()->addPointToPcViewer(rotatedCloud, 3, 255, 10, 10);

	mGui->drawMap(mMap.cloud().makeShared());
	mGui->addPointToPcViewer(_cloud);
	mGui->spinOnce();
	return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepUpdateCameraPose() {
	mGui->drawCamera(mMap.cloud().sensor_orientation_.matrix(), mMap.cloud().sensor_origin_);

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

	vector<PointIndices> mClusterIndices;
	mClusterIndices = mMap.clusterCloud(cropedCloud);
	
	vector<ObjectCandidate> newCandidates;
	//create candidates from indices
	for (PointIndices indices : mClusterIndices) {
		ObjectCandidate candidate(indices, cropedCloud);
		if(mMap.distanceToPlane(candidate.cloud(), plane) < 0.05)
			newCandidates.push_back(candidate);
	}
	ObjectCandidate::matchSequentialCandidates(mCandidates, newCandidates, mConfig["mapParams"]["consecutiveClusterCentroidMatchingThreshold"]);
	
	return true;
}

Rect boundBox(vector<Point2f> _points2d) {
	int minX=99999, minY=999999, maxX=0, maxY=0;
	for (Point2f point : _points2d) {
		if(point.x < minX)
			minX = point.x;
		if(point.y < minY)
			minY = point.y;
		if(point.x > maxX)
			maxX = point.x;
		if(point.y > maxY)
			maxY = point.y;
	}

	return Rect(	minX<0?0:minX, 
					minY<0?0:minY, 
					maxX-minX, 
					maxY-minY);
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepCathegorizeCandidates(std::vector<ObjectCandidate>& _candidates, const cv::Mat &_frame1,const  cv::Mat &_frame2) {
	if(_candidates.size() == 0)
		return false;

	for (ObjectCandidate &candidate : _candidates) {
		vector<Point3f> points3d;
		for (PointXYZ pointpcl : *candidate.cloud()) {
			Point3f point(pointpcl.x, pointpcl.y, pointpcl.z);
			points3d.push_back(point);
		}
		vector<Point2f> reprojection1 = mCameras->project3dPoints(points3d, true, mMap.cloud().sensor_origin_, mMap.cloud().sensor_orientation_);
		vector<Point2f> reprojection2 = mCameras->project3dPoints(points3d, false, mMap.cloud().sensor_origin_, mMap.cloud().sensor_orientation_);

		Rect validFrame(0,0,_frame1.cols, _frame1.rows);
		Mat view = _frame1(boundBox(reprojection1)&validFrame);
		std::vector<double> probs1 = mRecognitionSystem->categorize(view);
		candidate.addView(view, probs1);

		Mat view2 = _frame2(boundBox(reprojection2)&validFrame);
		std::vector<double> probs2 = mRecognitionSystem->categorize(view2);
		candidate.addView(view2, probs2);

		/*std::cout << "Image left: " << cathegories[0].first << ": " << cathegories[0].second << std::endl;
		std::cout << "Image left: " << cathegories2[0].first << ": " << cathegories2[0].second << std::endl;
		imshow("view", view);
		imshow("view2", view2);
		waitKey();*/

		mGui->drawCandidate(candidate, mMap.cloud().sensor_origin_, mMap.cloud().sensor_orientation_);	
	}

	return true;
}

bool MainApplication::stepCheckGroundTruth()
{
	if (mCandidates.size() != 0 && mCandidateGroundTruth.size() != 0) {
		cout << "---------------- Recognition results ----------------" << endl;
		float threshold = mConfig["mapParams"]["consecutiveClusterCentroidMatchingThreshold"];
		vector<pair<int, float>> matchIndexDist = ObjectCandidate::matchCandidates(mCandidates, mCandidateGroundTruth, threshold);
		for (int i = 0; i < mCandidates.size(); i++) {
			int match = matchIndexDist[i].first;
			if (match == -1) {
				cout << "No match found, distance to closest is: " << matchIndexDist[i].second << endl;
			}
			else {
				int gtLabel = mCandidateGroundTruth[match].cathegory().first;
				int querryCandidateLabel = mCandidates[i].cathegory().first;
				if (gtLabel == querryCandidateLabel)
				{
					cout << i << ":label " << gtLabel << " with probability " << mCandidates[i].cathegory().second << endl;
				}
				else
				{
					cout << i << ":wrong match with " << querryCandidateLabel << " with probability " << mCandidates[i].cathegory().second << endl;
				}

			}
		}
		return true;
	}
	else {
		return false;
	}
}
