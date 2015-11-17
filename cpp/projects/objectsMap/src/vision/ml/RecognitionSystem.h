//
//
//
//
//
//





#ifndef RECOGNITIONSYStEM_H_
#define RECOGNITIONSYStEM_H_

#include "BoW.h"

#include <vector>
#include <cjson/Json.h>
#include <opencv2/opencv.hpp>

// Forward declaration
class ObjectCandidate;

class RecognitionSystem {
public:		// Public interface.
	RecognitionSystem(cjson::Json _configFile);

	std::vector<double> categorize(const cv::Mat &_newView);

private:	// Private methods.

private:	// Private members.
	BoW			mBow;
};


#endif	//	RECOGNITIONSYStEM_H_