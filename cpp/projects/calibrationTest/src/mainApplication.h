//
//
//
//
//


#ifndef MAINAPPLICATION_H_
#define MAINAPPLICATION_H_

#include "Gui.h"
#include "EnvironmentMap.h"
#include "StereoCameras.h"
#include "TimeTools.h"
#include "graph2d.h"

#include <cjson/json.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

class MainApplication {
public:
	MainApplication	(int _argc, char** _argv);
	bool step	();
private:
	bool loadArguments	(int _argc, char** _argv);
	bool initCameras	();
	bool initGui		();
	bool init3dMap		();

	bool stepGetImages(cv::Mat &_frame1, cv::Mat &_frame2);
	bool stepTriangulatePoints(const cv::Mat &_frame1, const cv::Mat &_frame2, std::vector<cv::Point3f> &_points3d);
	bool stepUpdateMap(const std::vector<cv::Point3f> &_points3d);
	bool stepUpdateCameraRotation();
	bool stepGetCandidates();

private:
	StereoCameras	*mCameras;
	EnvironmentMap	mMap;
	Gui				*mGui;
	BOViL::plot::Graph2d mTimePlot;
	std::vector<double> tGetImages, tTriangulate, tUpdateCamera, tUpdCam, tCandidates;
	BOViL::STime *mTimer;

	cjson::Json mConfig;
};

#endif	//	MAINAPPLICATION_H_