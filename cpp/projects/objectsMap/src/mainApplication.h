//
//
//
//
//


#ifndef MAINAPPLICATION_H_
#define MAINAPPLICATION_H_

#include "utils/gui/graph2d.h"
#include "utils/gui/Gui.h"
#include "utils/TimeTools.h"

#include <StereoLib/map3d/EnvironmentMap.h>
#include <StereoLib/ml/RecognitionSystem.h>
#include <StereoLib/StereoCameras.h>
#include <StereoLib/EkfImuIcp.h>

#include <implementations/sensors/ImuSensor.h>

#include <cjson/json.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

class MainApplication {
public:
	MainApplication	(int _argc, char** _argv);
	bool step	();

private:
	bool loadArguments						(int _argc, char** _argv);
	bool initCameras						();
	bool initGui							();
	bool init3dMap							();
	bool initRecognitionSystem				();
	bool initImuAndEkf						();
	Eigen::Vector3f calculateGravityOffset	();
	bool initLoadGt							();	// 666 Debug.

	bool stepGetImages(cv::Mat &_frame1, cv::Mat &_frame2);
	bool stepGetImuData(ImuData &_imuData);
	bool stepTriangulatePoints(const cv::Mat &_frame1, const cv::Mat &_frame2, pcl::PointCloud<pcl::PointXYZ>::Ptr &_points3d);
	bool stepEkf(const ImuData &_imuData, Eigen::Vector4f &_position, Eigen::Quaternion<float> &_quaternion);
	bool stepUpdateMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_points3d, const Eigen::Vector4f &_translationPrediction, const Eigen::Quaternionf &_qRotationPrediction);
	bool stepUpdateCameraPose();
	bool stepGetCandidates();
	bool stepCathegorizeCandidates(std::vector<ObjectCandidate> &_candidates, const cv::Mat &_frame1,const  cv::Mat &_frame2);

	bool stepCheckGroundTruth();	// 666 Debug.
private:
	StereoCameras		*mCameras;
	EnvironmentMap		mMap;
	Gui					*mGui;
	RecognitionSystem	*mRecognitionSystem;
	std::vector<ObjectCandidate> mCandidates;

	ImuSensor			*mImu;
	double				mPreviousTime = -1;
	Eigen::Matrix3f		mImu2CamT;
	Eigen::Quaternionf	mInitialRot;
	Eigen::Vector3f		mGravityOffImuSys;
	EkfImuIcp			mEkf;

	BOViL::plot::Graph2d mTimePlot;
	std::vector<double> tGetImages, tTriangulate, tUpdateMap, tUpdCam, tCandidates, tCathegorize;
	BOViL::STime *mTimer;

	cjson::Json mConfig;

	std::vector<ObjectCandidate> mCandidateGroundTruth;	// 666 Debug.
};

#endif	//	MAINAPPLICATION_H_