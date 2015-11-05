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

// Forward declaration
class ObjectCandidate;

class RecognitionSystem {
public:		// Public interface.
	RecognitionSystem(cjson::Json _configFile);

	void categorize(ObjectCandidate &_candidate);

private:	// Private methods.
	void setBowParams(cjson::Json _params);
	void setModel(cjson::Json _params);
	cv::ml::SVM::Types decodeSvmType(std::string _string);
	cv::ml::SVM::KernelTypes decodeKernelType(std::string _string);

	void setFeatures(cjson::Json _params);

private:	// Private members.
	algorithm::BoW	mBow;
	algorithm::BoW::Params		mBowParams;
};


#endif	//	RECOGNITIONSYStEM_H_