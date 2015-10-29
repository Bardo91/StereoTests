//
//
//
//
//


#ifndef MAINAPPLICATION_H_
#define MAINAPPLICATION_H_

#include "StereoCameras.h"
#include "EnvironmentMap.h"

#include <cjson/json.h>
#include <opencv2/opencv.hpp>

class MainApplication {
public:
	bool init	(int _argc, char** _argv);
	bool step	();
private:
	bool loadArguments	(int _argc, char** _argv);
	bool initGui		();
	bool initCameras	();
	bool init3dMap		();

	bool stepSearchPointsOnImage(std::vector<cv::Point3f> &_points3d);
	bool stepUpdateMap(std::vector<cv::Point3f> &_points3d, pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud);
	bool stepGetCandidates();

private:
	StereoCameras	mCameras;
	EnvironmentMap	mMap;

	cjson::Json mConfig;
};

#endif	//	MAINAPPLICATION_H_