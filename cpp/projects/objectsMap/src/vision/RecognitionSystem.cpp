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
	setBowParams(_configFile["bovwParams"]);
	setModel(_configFile["mlModel"]);
	setFeatures(_configFile["features"]);
	mBow.load(_configFile["mlModel"]["modelPath"]);	// 666 Bow class load vocabulary and svm/LDA model, but in json all is inside mlModel
}

//---------------------------------------------------------------------------------------------------------------------
std::vector<std::pair<unsigned, float>> RecognitionSystem::categorize(const cv::Mat &_view) {
	// For now, only one region.
	vector<Rect> regions;
	regions.push_back(Rect(0,0,_view.cols, _view.rows));

	return mBow.evaluate(_view, regions);
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
		return  ml::SVM::Types::EPS_SVR;	// 666 never reach this.
	}
}

//---------------------------------------------------------------------------------------------------------------------
cv::ml::SVM::KernelTypes RecognitionSystem::decodeKernelType(std::string _string) {
	if (_string == "RBF") {
		return ml::SVM::KernelTypes::RBF;
	}
	else {
		assert(false);
		return  ml::SVM::KernelTypes::RBF;	// 666 never reach this.
	}
}

//---------------------------------------------------------------------------------------------------------------------
void RecognitionSystem::setModel(cjson::Json _params) {
	if (_params["name"] == "SVM") {
		mMlModel = new SvmModel();
		static_cast<SvmModel*>(mMlModel)->setParams(_params["params"]["c"], _params["params"]["gamma"], decodeSvmType(_params["params"]["svmType"]), decodeKernelType(_params["params"]["kernel"]));
		mBow.model(*mMlModel);
	}
	else if (_params["name"] = "LDA") {
		// 666 fill this too.
	}
	else {
		assert(false);
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
	if (_params["descriptor"] == "SIFT") {
		mBowParams.descriptorType = BoW::Params::eDescriptorType::SIFT;
	}
	else if (_params["descriptor"] == "ORB") {
		mBowParams.descriptorType = BoW::Params::eDescriptorType::ORB;
	}
	else if (_params["descriptor"] == "SURF") {
		mBowParams.descriptorType = BoW::Params::eDescriptorType::SURF;
	}
	else {
		assert(false);
	}

	mBow.params(mBowParams);
}
