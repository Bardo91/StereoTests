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
#include <StereoLib/FloorSubstractorCCS.h>

#include <implementations/sensors/ImuSimulatorSensor.h>
#include <implementations/sensors/MavrosSensor.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


using namespace cjson;
using namespace cv;
using namespace pcl;
using namespace std;
using namespace Eigen;


const int BIT_MAST_ERROR_IMU = 0;
const int BIT_MAST_ERROR_IMAGES = 1;
const int BIT_MAST_ERROR_FORECAST = 2;
const int BIT_MAST_ERROR_TRIANGULATE = 3;
const int BIT_MAST_ERROR_MAP = 4;
const int BIT_MAST_ERROR_EKF = 5;
const int BIT_MAST_ERROR_GETCANDIDATES = 6;
const int BIT_MAST_ERROR_CATEGORIZINGCANDIDATES = 7;

//---------------------------------------------------------------------------------------------------------------------
MainApplication::MainApplication(int _argc, char ** _argv):mTimePlot("Global Time"), mPositionPlot("Drone position"), mVelocityPlot("Drone Velocity"), mThresholdPlot("ICP error") {
	bool result = true;
	result &= loadArguments(_argc, _argv);
	result &= initCameras();
	result &= initGui();
	result &= init3dMap();
	result &= initRecognitionSystem();
	//result &= initLoadGt();
	result &= initImuAndEkf();
	
	initLog();

	/**/
	mLearnFloor = bool(mConfig["floorSubstractor"]["learn"]);
	mFloorSubstractor = new FloorSubstractorCCS();
	/**/
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
	mGui->clearMap();
	mGui->clearPcViewer();

	long errorBitList = 0;	// This variable store in each bit if each step was fine or not.

	// --> Get Imu Data
	ImuData imuData;
	if (!stepGetImuData(imuData)) {
		errorBitList |= (1 << BIT_MAST_ERROR_IMU);
		std::cout << "-> STEP: Error getting imu data" << std::endl;
	}

	// --> Get images and check if they are blurry or not.
	Mat frame1, frame2, frame1Gray, frame2Gray;
	if (!stepGetImages(frame1, frame2)) {
		errorBitList |= (1 << BIT_MAST_ERROR_IMAGES);
		std::cout << "-> STEP: Error getting images data or images are blurry" << std::endl;
	}

	cvtColor(frame1, frame1Gray, CV_BGR2GRAY);
	cvtColor(frame2, frame2Gray, CV_BGR2GRAY);


	// --> If system is not set-up yet.
	if (mIsFirstIter) {
		// If current images are not good.
		if ((errorBitList & (1 << BIT_MAST_ERROR_IMAGES))) {
			std::cout << "-> STEP: Waiting for a good image to set up system references" << std::endl;
			return false; // Do not iterate until there is a good image to work with.
		}
		// If have good images. Initiallize references and keep going.
		else {
			mInitialRot = Eigen::Quaternion<float>(imuData.mQuaternion[3], imuData.mQuaternion[0], imuData.mQuaternion[1], imuData.mQuaternion[2]);
			mIsFirstIter = false;
			std::cout << "-> STEP: Set current orientation as initial orientation" << std::endl;
		}
	}

	

	// --> Estimate position from previous step for either ICP or EKF depending on if the images are good or not.
	Eigen::Vector4f forecastX = Eigen::Vector4f::Ones();
	if (!(errorBitList & (1 << BIT_MAST_ERROR_IMU))) {
		auto prevX = mEkf.getStateVector().cast<float>();
		double incT = imuData.mTimeSpan - mPreviousTime;

		// x(k) = x(k-1) + v(k-1)*incT + a(k-1)*incT*incT/2;
		forecastX.block<3, 1>(0, 0) =	prevX.block<3, 1>(0, 0) +			
										(Matrix3f::Identity()*incT)*prevX.block<3, 1>(3, 0) +
										(Matrix3f::Identity()*(incT*incT / 2))*prevX.block<3, 1>(6, 0);
		std::cout << "-> STEP: Forecast from previous state is" << std::endl;
		std::cout << forecastX<<std::endl;
	}
	else {
		errorBitList |= (1<<BIT_MAST_ERROR_FORECAST);
		std::cout << "-> STEP: Error predicting new position" << std::endl;
	}
	

	PointCloud<PointXYZ>::Ptr cloud;
	// If forecast is fine.
	if (!(errorBitList & (1 << BIT_MAST_ERROR_FORECAST))) {
		// If images are fine: calculate the guess for the ICP in order to get current position estimation
		// to iterate over EKF
		if (!(errorBitList & (1 << BIT_MAST_ERROR_IMAGES))) {
			Eigen::Quaternion<float> orientation(imuData.mQuaternion[3], imuData.mQuaternion[0], imuData.mQuaternion[1], imuData.mQuaternion[2]);

			// Transform from north CS to camera's CS
			Eigen::Transform<float, 3, Affine> pose = Eigen::Translation3f(forecastX.block<3, 1>(0, 0))*orientation;
			pose = mCam2Imu*mInitialRot.inverse()*pose*mCam2Imu.inverse();

			// Calculate new cloud from input images
			if (stepTriangulatePoints(frame1Gray, frame2Gray, cloud)) {
				// Update pose
				Vector4f position;
				position << pose.translation().block<3, 1>(0, 0), 1;

				if (!stepUpdateMap(cloud, position, Quaternionf(pose.rotation()))) {
					errorBitList |= (1 << BIT_MAST_ERROR_MAP);
					std::cout << "-> STEP: Error while updating map" << std::endl;
				}
				else {
					mGui->drawCamera(mMap.cloud().sensor_orientation_.matrix(), mMap.cloud().sensor_origin_,0,255,0);
				}
			}
			else {
				errorBitList |= (1 << BIT_MAST_ERROR_TRIANGULATE);
				std::cout << "-> STEP: Error generating new point cloud" << std::endl;
			}
		}

		// If we dont have good images or the ICP fails: use forecastX to iterate over EKF
		if((errorBitList & (1 << BIT_MAST_ERROR_IMAGES)) || (errorBitList & (1 << BIT_MAST_ERROR_MAP))) {
			// Get state vector in "north coordinate system"
			Eigen::Quaternion<float> q(imuData.mQuaternion[3], imuData.mQuaternion[0], imuData.mQuaternion[1], imuData.mQuaternion[2]);
			Eigen::Translation3f sensorPos(forecastX.block<3,1>(0,0));

			// Transform from north CS to camera's CS
			Eigen::Transform<float,3, Affine> pose = sensorPos*q;
			pose = mCam2Imu*mInitialRot.inverse()*pose*mCam2Imu.inverse();

			// Update pose
			mMap.updateSensorPose(pose.matrix().block<4,1>(0,3), Quaternionf(pose.rotation()));
			//mMap.updateSensorPose(pose.matrix().block<4, 1>(0, 3), mMap.cloud().sensor_orientation_);
		}
	}
	Eigen::Quaternion<float> q(imuData.mQuaternion[3], imuData.mQuaternion[0], imuData.mQuaternion[1], imuData.mQuaternion[2]);
	Eigen::Translation3f sensorPos(forecastX.block<3, 1>(0, 0));

	Eigen::Transform<float, 3, Affine> pose = sensorPos*q;
	pose = mCam2Imu*mInitialRot.inverse()*pose*mCam2Imu.inverse();
	mGui->drawCamera(pose.rotation(), pose.matrix().block<4, 1>(0, 3), 0, 0, 255);
	// Draw camera pose for display purpose.
	mGui->spinOnce();

	// Iterate EKF
	if (!(errorBitList & (1 << BIT_MAST_ERROR_FORECAST))) {
		if (!stepEkf(imuData)) {
			errorBitList |= (1<<BIT_MAST_ERROR_EKF);
			std::cout << "-> STEP: Error during EKF" << std::endl;
		}
		std::cout << "-> STEP: EKF state after iteration with current data" << std::endl;
		std::cout << mEkf.getStateVector().transpose() << std::endl;
	}
	else {
		errorBitList |= (1<<BIT_MAST_ERROR_EKF);
		std::cout << "-> STEP: Cannot perform EKF" << std::endl;
	}

	if (!stepGetCandidates()) {
		errorBitList |= (1 << BIT_MAST_ERROR_GETCANDIDATES);
		std::cout << "-> STEP: Error getting candidates" << std::endl;
	}

	if (!stepCathegorizeCandidates(mCandidates, frame1, frame2)) {
		errorBitList |= (1 << BIT_MAST_ERROR_CATEGORIZINGCANDIDATES);
		std::cout << "-> STEP: Error cathegorizing candidates" << std::endl;
	}


	//	Check if need to learn or relearn floor
	if (mLearnFloor) {
		Eigen::Quaternion<float> q(imuData.mQuaternion[3], imuData.mQuaternion[0], imuData.mQuaternion[1], imuData.mQuaternion[2]);

		Vector4f verticalCCS = (mCam2Imu*mInitialRot.inverse()*Vector4f::UnitZ());

		pcl::ModelCoefficients::Ptr planeCoeff(new pcl::ModelCoefficients);
		if (learnFloor(verticalCCS.block<3, 1>(0, 0), planeCoeff, double(mConfig["mapParams"]["floorMaxAllowedRotationToDrone"])*M_PI / 180.0)) {
			mLearnFloor = false;
			mIsFirstIter = true;
			mMap.clear();
			mCandidates.clear();
			std::cout << "-> STEP: Learned floor pattern" << std::endl;
		}
		else {
			std::cout << "-> STEP: Failed to Learn floor pattern" << std::endl;
		}
	}

	// <----------->
	// Store and plot positions data 666 debug
	auto xEkf = mEkf.getStateVector();
	auto xIcp = (mInitialRot*mCam2Imu.inverse()*
				(Translation3f(mMap.cloud().sensor_origin_.block<3,1>(0,0))*mMap.cloud().sensor_orientation_)
				*mCam2Imu).translation();


	posXekf.push_back(xEkf(0,0));
	posYekf.push_back(xEkf(1,0));
	posZekf.push_back(xEkf(2,0));
	velXekf.push_back(xEkf(3,0));
	velYekf.push_back(xEkf(4,0));
	velZekf.push_back(xEkf(5,0));
	posXicp.push_back(xIcp(0,0));
	posYicp.push_back(xIcp(1,0));
	posZicp.push_back(xIcp(2,0));
	posXfore.push_back(forecastX(0,0));
	posYfore.push_back(forecastX(1,0));
	posZfore.push_back(forecastX(2,0));
	threshold.push_back(mMap.fittingScore());
	mThresholdPlot.draw(threshold, 255, 0, 0, BOViL::plot::Graph2d::eDrawType::Lines);
	mPositionPlot.draw(posXekf, 255,0,0, BOViL::plot::Graph2d::eDrawType::Lines);
	mPositionPlot.draw(posYekf, 0,255,0, BOViL::plot::Graph2d::eDrawType::Lines);
	mPositionPlot.draw(posZekf, 0,0,255, BOViL::plot::Graph2d::eDrawType::Lines);
	mPositionPlot.draw(posXicp, 255,0,0, BOViL::plot::Graph2d::eDrawType::FilledCircles);
	mPositionPlot.draw(posYicp, 0,255,0, BOViL::plot::Graph2d::eDrawType::FilledCircles);
	mPositionPlot.draw(posZicp, 0,0,255, BOViL::plot::Graph2d::eDrawType::FilledCircles);
	mPositionPlot.draw(posXfore, 255,0,0, BOViL::plot::Graph2d::eDrawType::Circles);
	mPositionPlot.draw(posYfore, 0,255,0, BOViL::plot::Graph2d::eDrawType::Circles);
	mPositionPlot.draw(posZfore, 0,0,255, BOViL::plot::Graph2d::eDrawType::Circles);
	
	mVelocityPlot.draw(velXekf, 255,0,0, BOViL::plot::Graph2d::eDrawType::Lines);
	mVelocityPlot.draw(velYekf, 0,255,0, BOViL::plot::Graph2d::eDrawType::Lines);
	mVelocityPlot.draw(velZekf, 0,0,255, BOViL::plot::Graph2d::eDrawType::Lines);
	// <----------->
	if (!mLearnFloor) {
		save2Log();
	}
	// If any error occurs return false. And if not return true;
	if (errorBitList) 
		return false;
	else 
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
	params.icpMaxAngleChangeCompared2ProvidedGuess = mConfig["mapParams"]["icpMaxAngleChangeCompared2ProvidedGuess"]; //
	params.icpMaxTranslationChangeCompared2ProvidedGuess = mConfig["mapParams"]["icpMaxTranslationChangeCompared2ProvidedGuess"];

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

			auto imu2camQ = AngleAxisf	(float(mConfig["ekf"]["Imu2Cam"]["rot"]["z"])/180.0*M_PI, Vector3f::UnitZ())
							* AngleAxisf	(float(mConfig["ekf"]["Imu2Cam"]["rot"]["y"])/180.0*M_PI,  Vector3f::UnitY())
							*AngleAxisf		(float(mConfig["ekf"]["Imu2Cam"]["rot"]["x"])/180.0*M_PI, Vector3f::UnitX());
			auto imu2CamT = Translation3f(	mConfig["ekf"]["Imu2Cam"]["trans"]["x"],
											mConfig["ekf"]["Imu2Cam"]["trans"]["y"],
											mConfig["ekf"]["Imu2Cam"]["trans"]["z"]);

			mCam2Imu = (imu2CamT*imu2camQ).inverse();
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
	ImuData imuData;
	for (int i = 0;i < nSamples; i++) {
		 imuData = mImu->get();
	
		Eigen::Quaternion<float> q(imuData.mQuaternion[3], imuData.mQuaternion[0], imuData.mQuaternion[1], imuData.mQuaternion[2]);
		Eigen::Vector3f linAcc, angSpeed;
		linAcc << imuData.mLinearAcc[0],  imuData.mLinearAcc[1], imuData.mLinearAcc[2];
	
		gravity += q*linAcc;
	}

	mInitialRot = Eigen::Quaternion<float> (imuData.mQuaternion[3], imuData.mQuaternion[0], imuData.mQuaternion[1], imuData.mQuaternion[2]);

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
bool MainApplication::learnFloor(const Eigen::Vector3f &_verticalCCS, pcl::ModelCoefficients::Ptr &_planeCoeff, const double &_maxAngle) {
	auto cloud = mMap.cloud().makeShared();
	// Try to fit plane
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *_planeCoeff);
	
	if (inliers->indices.size() == 0) {
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
		return false;
	}
	
	mGui->drawPlane(*_planeCoeff);
	Vector3f planeNormal = Vector3f(-_planeCoeff->values[0], -_planeCoeff->values[1], -_planeCoeff->values[2]);

	float angle = acos(_verticalCCS.dot(planeNormal) / (_verticalCCS.norm()*planeNormal.norm()));
	cout << angle*180/M_PI << "deg Angle" << endl;
	//cout << "............." << endl;
	//cout << _verticalCCS.transpose() << endl;
	//cout << planeNormal.transpose() << endl;
	//cout << angle*180/M_PI << endl;
	//cout << "............." << endl;
	
	// Extract patches

	if (angle < _maxAngle) {
		std::vector<Mat> patches;
		bool ensuredBigPatch = false;	// Need to ensure that have at least one big patch that can be representative of floor.
		for(ObjectCandidate candidate : mCandidates) {
			auto views = candidate.views();
			for (Mat view : views) {
				if (view.rows > 100 && view.cols > 100)
					ensuredBigPatch = true;
			}
			patches.insert(patches.end(), views.begin(), views.end());
		}
		if (ensuredBigPatch) {
			return mFloorSubstractor->train(patches);
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
	_frame2 = mCameras->camera(1).frame();

	cvtColor(_frame1, gray1, CV_BGR2GRAY);
	if (_frame1.rows != 0) {
		isBlurry1 = isBlurry(gray1, mConfig["cameras"]["blurThreshold"]);
	}
	else { return false; }

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
		if (mFloorSubstractor->isTrained()) {
			mFloorSubstractor->substract(_frame1, _frame1);
			mFloorSubstractor->substract(_frame2, _frame2);
		}

		_frame1 = mCameras->camera(0).undistort(_frame1);
		_frame2 = mCameras->camera(1).undistort(_frame2);

		mGui->updateStereoImages(_frame1, _frame2);
		Rect leftRoi = mCameras->roi(true);
		Rect rightRoi = mCameras->roi(false);
		mGui->drawBox(leftRoi, true, 0,255,0);
		mGui->drawBox(rightRoi, false, 0,255,0);
		
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
	_points3d = mCameras->pointCloud(_frame1, _frame2, disparityRange, squareSize,mConfig["cameras"]["maxTemplateScore"], maxReprojectionError);	

	return _points3d->size() != 0? true:false;
}

bool MainApplication::stepEkf(const ImuData & _imuData) {
	// Get and adapt imu data.
	Eigen::Quaternion<float> q(_imuData.mQuaternion[3], _imuData.mQuaternion[0], _imuData.mQuaternion[1], _imuData.mQuaternion[2]);
	Eigen::Matrix<float, 3, 1> linAcc;
	linAcc << _imuData.mLinearAcc[0], _imuData.mLinearAcc[1], _imuData.mLinearAcc[2];
	linAcc = q*linAcc - mGravityOffImuSys;

	auto sensorPos = mMap.cloud().sensor_origin_;
	auto sensorRot = mMap.cloud().sensor_orientation_;
	Eigen::Matrix4f icpRes = Eigen::Matrix4f::Zero();
	icpRes.block<3,3>(0,0) = sensorRot.matrix();
	icpRes.block<4,1>(0,3) = sensorPos;
	
	auto sensorPoseNorthCS = mInitialRot*mCam2Imu.inverse()*Transform<float,3, Affine>(icpRes)*mCam2Imu;

	// Create observable state variable vetor.
	Eigen::MatrixXf zk(6, 1);
	zk << sensorPoseNorthCS.translation(), linAcc;

	// Update EKF.
	std::cout << "-> STEP: Increment of time in EKF: " << _imuData.mTimeSpan - mPreviousTime << std::endl;
	mEkf.stepEKF(zk.cast<double>(), _imuData.mTimeSpan - mPreviousTime);
	mPreviousTime = _imuData.mTimeSpan;
	
	return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepUpdateMap(const PointCloud<PointXYZ>::Ptr &_cloud, const Vector4f &_translationPrediction, const Quaternionf &_qRotationPrediction){
	mGui->clearMap();
	mGui->clearPcViewer();

	PointCloud<PointXYZ>::Ptr addedCloudCameraCS;
	bool hasConverged = mMap.addPoints(_cloud, _translationPrediction, _qRotationPrediction, mMap.Simple,double(mConfig["mapParams"]["maxFittingScore"]), addedCloudCameraCS);

	if(addedCloudCameraCS->size() != 0)
		Gui::get()->addPointToPcViewer(addedCloudCameraCS, 3, 255, 10, 10);

	mGui->drawMap(mMap.cloud().makeShared());
	mGui->addPointToPcViewer(_cloud);
	mGui->addCluster(mMap.GuessCloud(), 2, 0, 0, 255);
	mGui->addCluster(mMap.AlignedCloud(), 3,255,255,255);

	if (!hasConverged) {
		mGui->drawCamera(mMap.ICPres().block<3, 3>(0, 0), mMap.ICPres().col(3), 255, 0, 0);
	}
	//mGui->spinOnce();
	return hasConverged;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepUpdateCameraPose() {
	mGui->drawCamera(mMap.cloud().sensor_orientation_.matrix(), mMap.cloud().sensor_origin_);

	return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::stepGetCandidates(){
	if (mMap.cloud().size() < 10) {
		return false;
	}

	//ModelCoefficients plane = mMap.extractFloor(mMap.cloud().makeShared());
	//if (plane.values.size() == 0)
	//	return false;
	//mGui->drawPlane(plane, 0,0,1.5);
	PointCloud<PointXYZ>::Ptr cropedCloud = mMap.cloud().makeShared();
	//mMap.cropCloud(cropedCloud, plane);

	vector<PointIndices> mClusterIndices;
	mClusterIndices = mMap.clusterCloud(cropedCloud);
	
	vector<ObjectCandidate> newCandidates;
	//create candidates from indices
	for (PointIndices indices : mClusterIndices) {
		ObjectCandidate candidate(indices, cropedCloud);
		/*if(mMap.distanceToPlane(candidate.cloud(), plane) < 0.05)*/
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
		if (view.rows > 20 && view.cols > 20) {
			Mat grayView;
			cvtColor(view, grayView, CV_BGR2GRAY);
			std::vector<double> probs1 = mRecognitionSystem->categorize(grayView);
			candidate.addView(view, probs1);
		}

		Mat view2 = _frame2(boundBox(reprojection2)&validFrame);
		if (view2.rows > 20 && view2.cols > 20) {
			Mat grayView;
			cvtColor(view2, grayView, CV_BGR2GRAY);
			std::vector<double> probs2 = mRecognitionSystem->categorize(grayView);
			candidate.addView(view2, probs2);
		}

		/*std::cout << "Image left: " << cathegories[0].first << ": " << cathegories[0].second << std::endl;
		std::cout << "Image left: " << cathegories2[0].first << ": " << cathegories2[0].second << std::endl;
		imshow("view", view);
		imshow("view2", view2);
		waitKey();*/

		mGui->drawCandidate(candidate, mMap.cloud().sensor_origin_, mMap.cloud().sensor_orientation_);	
	}

	return true;
}

//---------------------------------------------------------------------------------------------------------------------
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

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::initLog() {
	mExeFolder = "execution_" + to_string(time(NULL))+"/";
	CreateDirectory(mExeFolder.c_str(), NULL);

	mCameraLog.open(mExeFolder + "cameraLog.txt");
	mMapLog.open(mExeFolder + "mapLog.txt");
	mFloorLog.open(mExeFolder + "floorLog.txt");

	return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool MainApplication::save2Log() {
	auto map = mMap.cloud();
	Quaternionf camOri = map.sensor_orientation_;
	Vector4f camPos = map.sensor_origin_;
	// Store camera info. position and orientation in quaternions
	mCameraLog << camPos.block<3,1>(0,0).transpose() << "\t";
	mCameraLog << camOri.x() << "\t" << camOri.y() << "\t" << camOri.z() << "\t" << camOri.w() << std::endl;
	mCameraLog.flush();

	// Store Map info.
	mMapLog << map.size() << std::endl;
	mMapLog.flush();

	// Store Floor information;
	mFloorLog << mFloorSubstractor->isTrained() << std::endl;
	mFloorLog.flush();

	// Store Candidate Information.
	if (mCandidateLogs.size() < mCandidates.size()) {
		int sizeToExtend = mCandidates.size() - mCandidateLogs.size();
		for (unsigned i = 0; i < sizeToExtend; i++) {
			mCandidateLogs.push_back(ofstream(mExeFolder+"candidate_" + to_string(mCandidateLogs.size())+".txt"));
			mCandidateCloudsLogs.push_back(ofstream(mExeFolder+"candidateClouds_" + to_string(mCandidateCloudsLogs.size()) + ".txt"));
		}
	}
	for (unsigned i = 0; i < mCandidates.size(); i++) {
		auto candidate = mCandidates[i];
		auto cloud = *candidate.cloud();
		auto cathegories = candidate.cathegoryHistory();
		candidate.cathegory();
		mCandidateLogs[i] << candidate.centroid().block<3, 1>(0, 0).transpose() << "\t" << cloud.size() << "\t";
		for (unsigned j = 0; j < cathegories.size(); j++) {
			mCandidateLogs[i] << *cathegories[j].rbegin() << "\t";
		}
		mCandidateLogs[i] << std::endl;
		mCandidateLogs[i].flush();

		for (unsigned j = 0; j < cloud.size(); j++) {
			mCandidateCloudsLogs[i] << cloud[j].x << "\t" << cloud[j].y << "\t" << cloud[j].z << "\t";
		}
		mCandidateCloudsLogs[i] << std::endl;
		mCandidateCloudsLogs[i].flush();
	}
	return true;
}
