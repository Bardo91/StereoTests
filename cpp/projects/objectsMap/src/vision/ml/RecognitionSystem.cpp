//
//
//
//
//
//


#include "RecognitionSystem.h"

#include <cassert>

using namespace cjson;
using namespace cv;
using namespace std;

//---------------------------------------------------------------------------------------------------------------------
// Public interface
//---------------------------------------------------------------------------------------------------------------------
RecognitionSystem::RecognitionSystem(cjson::Json _configFile) {
	mBow.params(_configFile["bow"]);
	mBow.load(_configFile["bow"]["modelPath"]);
}

//---------------------------------------------------------------------------------------------------------------------
std::vector<double> RecognitionSystem::categorize(const cv::Mat &_view) {
	return mBow.evaluate(_view);
}

//---------------------------------------------------------------------------------------------------------------------
// Private methods
//---------------------------------------------------------------------------------------------------------------------
