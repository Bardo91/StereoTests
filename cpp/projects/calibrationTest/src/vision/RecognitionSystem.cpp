//
//
//
//
//
//


#include "RecognitionSystem.h"

#include <cassert>

using namespace algorithm;
using namespace cjson;
using namespace cv;
using namespace std;

//---------------------------------------------------------------------------------------------------------------------
// Public interface
//---------------------------------------------------------------------------------------------------------------------
RecognitionSystem::RecognitionSystem(cjson::Json _configFile) {
	setBowParams(_configFile["bowParams"]);
	setModel(_configFile["mlModel"]);
	setFeatures(_configFile["features"]);
}

//---------------------------------------------------------------------------------------------------------------------
void RecognitionSystem::categorize(ObjectCandidate & _candidate) {

}

//---------------------------------------------------------------------------------------------------------------------
// Private methods
//---------------------------------------------------------------------------------------------------------------------
void RecognitionSystem::setBowParams(cjson::Json _params) {
	mBowParams.vocSize = (int) _params["vocabularySize"];
	mBow.params(mBowParams);
}

//---------------------------------------------------------------------------------------------------------------------
cv::ml::SVM::Types RecognitionSystem::decodeSvmType(std::string _string){
	if (_string == "c_svc") {
		return ml::SVM::Types::C_SVC;
	}
	else if (_string == "v_svc") {
		return ml::SVM::Types::NU_SVC;
	}
	else if (_string == "v_svr") {
		return ml::SVM::Types::NU_SVR;
	}
	else if (_string == "eps_svr") {
		return ml::SVM::Types::EPS_SVR;
	}
	else {
		assert(false);
	}
}

//---------------------------------------------------------------------------------------------------------------------
cv::ml::SVM::KernelTypes RecognitionSystem::decodeKernelType(std::string _string) {
	if (_string == "c_svc") {
		return ml::SVM::KernelTypes::RBF;
	}
	else {
		assert(false);
	}
}

//---------------------------------------------------------------------------------------------------------------------
void RecognitionSystem::setModel(cjson::Json _params) {
	if (_params["name"] == "SVM") {
		SvmModel model;
		model.setParams(_params["c"], _params["gamma"], decodeSvmType(_params["svmType"]), decodeKernelType(_params["kernel"]));
	}
	else if (_params["name"] = "LDA") {

	}
	else {
		assert();
	}
}

//---------------------------------------------------------------------------------------------------------------------
void RecognitionSystem::setFeatures(cjson::Json _params) {
	// get detector
	if (_params["detector"] == "SIFT") {
		mBowParams.extractorType = BoW::Params::eExtractorType::SIFT;
	}
	else if (_params["detector"] == "FAST") {
		mBowParams.extractorType = BoW::Params::eExtractorType::FAST;
	}
	else if (_params["detector"] == "ORB") {
		mBowParams.extractorType = BoW::Params::eExtractorType::ORB;
	}
	else if (_params["detector"] == "SURF") {
		mBowParams.extractorType = BoW::Params::eExtractorType::SURF;
	} else if (_params["detector"] == "MSER") {
		mBowParams.extractorType = BoW::Params::eExtractorType::MSER;
	}
	else {
		assert(false);
	}


	// Get descriptor
	if (_params["detector"] == "SIFT") {
		mBowParams.descriptorType = BoW::Params::eDescriptorType::SIFT;
	}
	else if (_params["detector"] == "SIFT") {
		mBowParams.descriptorType = BoW::Params::eDescriptorType::ORB;
	}
	else if (_params["detector"] == "SIFT") {
		mBowParams.descriptorType = BoW::Params::eDescriptorType::SURF;
	}
	else {
		assert(false);
	}
}
