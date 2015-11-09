//
//
//
//
//
//





#ifndef RECOGNITIONSYStEM_H_
#define RECOGNITIONSYStEM_H_

#include "ml/BoW.h"

#include <vector>
#include <cjson/Json.h>
#include <opencv2/opencv.hpp>

// Forward declaration
class ObjectCandidate;

class RecognitionSystem {
public:		// Public interface.
	RecognitionSystem(cjson::Json _configFile);

	std::vector<std::pair<unsigned, float>> categorize(const cv::Mat &_newView);

private:	// Private methods.
	void setBowParams(cjson::Json _params);
	void setModel(cjson::Json _params);
	cv::ml::SVM::Types decodeSvmType(std::string _string);
	cv::ml::SVM::KernelTypes decodeKernelType(std::string _string);

	void setFeatures(cjson::Json _params);

private:	// Private members.
	algorithm::MlModel		*mMlModel;
	algorithm::BoW			mBow;
	algorithm::BoW::Params	mBowParams;
};


#endif	//	RECOGNITIONSYStEM_H_