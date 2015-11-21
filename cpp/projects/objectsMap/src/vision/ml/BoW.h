///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Single object detection - ObjectDetector
//		Author: Pablo R.S.
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cassert>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <cjson/json.h>
#include <libsvmpp/Svmpp.h>


#ifndef _ALGORITHM_BOW_H_
#define _ALGORITHM_BOW_H_


//-----------------------------------------------------------------------------------------------------------------
/// Base object detector class.
class BoW {
public:
	/// Set parameters for bow model
	void params(cjson::Json _paramFile);
		
	/// Get parameters of bow model;
	cjson::Json params() const;

	/// Train model with given dataset
	void train(const std::vector<cv::Mat> &_images, const std::vector<double> &_groundTruth);

	/// Evaluate input
	std::vector<double> evaluate(cv::Mat _image);

	/// Save model
	void save(std::string _name);

	/// Load model
	void load(std::string _name);

private:	// Private methods
	void defaultParams();
	void decodeParams(cjson::Json _paramFile);
	void decodeSvmParams(cjson::Json _svmParams);
	void decodeTrainGrids(cjson::Json _gridFile);
	void decodeSvmType(std::string _type, std::string _kernel);
	void decodeHistogramParams(cjson::Json _gridFile);

	cv::Mat computeDescriptor(cv::Mat _frame);

private:	// Private members
	// Histogram creation
	cv::Ptr<cv::FeatureDetector> mDetector;
	cv::HOGDescriptor mHog;
	enum class eDetector {SIFT, SURF, HOG};
	eDetector mDetecetorType;
	cv::Ptr<cv::DescriptorMatcher> mHistogramMatcher;
	cv::BOWKMeansTrainer *mBowTrainer;
	cv::BOWImgDescriptorExtractor *mHistogramExtractor;
	cv::Mat mCodebook;

	// Clasification model
	svmpp::Svm						mSvm;
	svmpp::Svm::Params				mSvmParams;
	bool							mAutoTrain = false;
	std::vector<svmpp::ParamGrid>	mParamGrids;

	cjson::Json mParams;
};



#endif	// _ALGORITHM_BOW_H_
