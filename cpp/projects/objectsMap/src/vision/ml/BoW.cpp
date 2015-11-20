///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Single object detection - ObjectDetector
//		Author: Pablo R.S.
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <fstream>
#include <numeric>

#include "BoW.h"
#include <opencv2/xfeatures2d.hpp>

using namespace std;
using namespace cv;
using namespace cjson;
using namespace svmpp;

//---------------------------------------------------------------------------------------------------------------------
// Public Interface
//---------------------------------------------------------------------------------------------------------------------
void BoW::params(Json _paramFile) {
	mParams = _paramFile;
	decodeParams(mParams);
}

//---------------------------------------------------------------------------------------------------------------------
Json BoW::params() const{
	return mParams;
}

//---------------------------------------------------------------------------------------------------------------------
void BoW::train(const vector<Mat> &_images, const vector<double> &_groundTruth){
	// Get all descriptors to form vocabulary.
	Mat descriptorsAll;
	vector<Mat> descriptorPerImg;
	for (unsigned i = 0; i < _images.size(); i++) {	// For each image in dataset
		vector<KeyPoint> keypoints;
		Mat descriptors;
		mDetector->detect(_images[i], keypoints);
		mDetector->compute(_images[i], keypoints, descriptors);
		descriptorsAll.push_back(descriptors);
		descriptorPerImg.push_back(descriptors);
	}
	// Form codebook from all descriptors
	mCodebook = mBowTrainer->cluster(descriptorsAll);
	mHistogramExtractor->setVocabulary(mCodebook);

	// Compute histograms
	vector<vector<double>> X;
	for (unsigned i = 0; i < _images.size(); i++) {
		Mat histogram;
		mHistogramExtractor->compute(descriptorPerImg[i], histogram);
		vector<double> x;
		for (int i = 0; i < histogram.cols; i++) {
			x.push_back(histogram.at<float>(i));
		}
		X.push_back(x);
	}
	TrainSet set;
	set.addEntries(X, _groundTruth);

	if (mAutoTrain) {
		mSvm.trainAuto(set, mSvmParams, mParamGrids);
	}
	else {
		mSvm.train(mSvmParams, set);
	}
}

//---------------------------------------------------------------------------------------------------------------------
vector<double> BoW::evaluate(Mat _image) {
	// Get histogram
	Mat descriptor;
	vector<KeyPoint> keypoints;
	mDetector->detect(_image, keypoints);
	mDetector->compute(_image, keypoints, descriptor);
	Mat histogram;
	mHistogramExtractor->compute(descriptor, histogram);
	vector<double> x;
	for (int i = 0; i < histogram.cols; i++) {
		x.push_back(histogram.at<float>(i));
	}
	Query query(x);

	// Evaluate
	vector<double> probs;
	mSvm.predict(query, probs);

	return probs;
}

//---------------------------------------------------------------------------------------------------------------------
void BoW::save(string _name) {
	mSvm.save(_name);
	FileStorage codebook(_name + ".vocabulary.xml", FileStorage::WRITE);
	codebook << "vocabulary" << mCodebook;
}

//---------------------------------------------------------------------------------------------------------------------
void BoW::load(string _name) {
	mSvm.load(_name);
	FileStorage codebook(_name + ".vocabulary.xml", FileStorage::READ);
	codebook["vocabulary"] >> mCodebook;
	mHistogramExtractor->setVocabulary(mCodebook);
}

//---------------------------------------------------------------------------------------------------------------------
//	Private interface.
//---------------------------------------------------------------------------------------------------------------------
void BoW::defaultParams() {
	mSvmParams.svm_type = C_SVC;
	mSvmParams.kernel_type = RBF;
	mSvmParams.cache_size = 100;
	mSvmParams.gamma = 0.1;
	mSvmParams.C = 1;
	mSvmParams.eps = 1e-5;
	mSvmParams.p = 0.1;
	mSvmParams.shrinking = 0;
	mSvmParams.probability = 1;
	mSvmParams.nr_weight = 0;
	mSvmParams.weight_label = nullptr;
	mSvmParams.weight = nullptr;
}

//---------------------------------------------------------------------------------------------------------------------
void BoW::decodeParams(::Json _paramFile) {
	decodeSvmParams(_paramFile["svm"]);
	decodeHistogramParams(_paramFile["histogramMatcher"]);
}

//---------------------------------------------------------------------------------------------------------------------
void BoW::decodeSvmParams(::Json _svmParams) {
	defaultParams();

	decodeSvmType(_svmParams["type"], _svmParams["kernel"]);
	
	// Decode parameters
	if (_svmParams.contains("c")) {
		mSvmParams.C = _svmParams["c"];
	}
	if (_svmParams.contains("nu")) {
		mSvmParams.nu = _svmParams["nu"];
	}
	if (_svmParams.contains("gamma")) {
		mSvmParams.gamma = _svmParams["gamma"];
	}

	// Decode autotrain parameters
	if (_svmParams["autoTrain"]) {
		decodeTrainGrids(_svmParams["trainGrids"]);
		mAutoTrain = true;
	}
}

//---------------------------------------------------------------------------------------------------------------------
void BoW::decodeTrainGrids(::Json _gridFile) {
	if (_gridFile.contains("c")) {
		mParamGrids.push_back(ParamGrid(ParamGrid::Type::C, _gridFile["c"]["min"], _gridFile["c"]["max"], _gridFile["c"]["step"]));
	}
	if (_gridFile.contains("gamma")) {
		mParamGrids.push_back(ParamGrid(ParamGrid::Type::Gamma, _gridFile["gamma"]["min"], _gridFile["gamma"]["max"], _gridFile["gamma"]["step"]));
	}
}

//---------------------------------------------------------------------------------------------------------------------
void BoW::decodeSvmType(string _type, string _kernel) {
	// Decode svm type
	if (_type == "c_svc") {
		mSvmParams.svm_type = C_SVC;
	} else if (_type == "nu_svc") {
		mSvmParams.svm_type = NU_SVC;
	}

	// Decode kernel
	if (_kernel == "linear") {
		mSvmParams.kernel_type = LINEAR;
	} else if (_kernel == "RBF") {
		mSvmParams.kernel_type = RBF;
	}
}

//---------------------------------------------------------------------------------------------------------------------
void BoW::decodeHistogramParams(::Json _histogramParams) {
	// Set features.
	if (_histogramParams["descriptor"] == "SIFT") {
		mDetector = xfeatures2d::SIFT::create();
	}
	else if (_histogramParams["descriptor"] == "SURF"){
		mDetector = xfeatures2d::SURF::create();
	}

	// Matcher
	mHistogramMatcher = DescriptorMatcher::create((string) _histogramParams["matcher"]);

	// Bow trainer
	int dictionarySize = _histogramParams["vocabularySize"];
	TermCriteria tc(CV_TERMCRIT_ITER, 10, 0.001);
	int retries = 1;
	int flags = KMEANS_PP_CENTERS;
	mBowTrainer = new BOWKMeansTrainer(dictionarySize, tc, retries, flags);

	// Set matcher.
	mHistogramExtractor  = new BOWImgDescriptorExtractor(mDetector, mHistogramMatcher);
}


