//
//
//
//
//


#ifndef MAINAPPLICATION_H_
#define MAINAPPLICATION_H_

#include "graph2d.h"
#include "Gui.h"
#include "EnvironmentMap.h"
#include "vision/RecognitionSystem.h"
#include "StereoCameras.h"
#include "TimeTools.h"

#include <cjson/json.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

class MainApplication {
public:
	MainApplication	(int _argc, char** _argv);
	bool step	();

private:
	bool loadArguments			(int _argc, char** _argv);
	bool initCameras			();
	bool initGui				();
	bool init3dMap				();
	bool initRecognitionSystem	();

	bool stepGetImages(cv::Mat &_frame1, cv::Mat &_frame2);
	bool stepTriangulatePoints(const cv::Mat &_frame1, const cv::Mat &_frame2, pcl::PointCloud<pcl::PointXYZ>::Ptr &_points3d);
	bool stepUpdateMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_points3d);
	bool stepUpdateCameraRotation();
	bool stepGetCandidates(std::vector<ObjectCandidate> &_candidates);
	bool stepCathegorizeCandidates(std::vector<ObjectCandidate> &_candidates, const cv::Mat &_frame1,const  cv::Mat &_frame2);

private:
	StereoCameras		*mCameras;
	EnvironmentMap		mMap;
	Gui					*mGui;
	RecognitionSystem	*mRecognitionSystem;

	BOViL::plot::Graph2d mTimePlot;
	std::vector<double> tGetImages, tTriangulate, tUpdateMap, tUpdCam, tCandidates, tCathegorize;
	BOViL::STime *mTimer;

	cjson::Json mConfig;
};

#endif	//	MAINAPPLICATION_H_