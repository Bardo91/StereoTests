///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Single object detection - ObjectDetector
//		Author: Pablo R.S.
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <fstream>
#include <numeric>

#include "BoW.h"
#include "Corpus.h"
#include <opencv2/xfeatures2d.hpp>

using namespace std;
using namespace cv;
using namespace cv::ml;

namespace algorithm {
	//-----------------------------------------------------------------------------------------------------------------
	void BoW::params(Params _params) {
		mParams = _params;

		setDetector(mParams.descriptorType);
		mHistogramMatcher = DescriptorMatcher::create("BruteForce");

		TermCriteria tc(CV_TERMCRIT_ITER, 10, 0.001);
		int retries = 1;
		int flags = KMEANS_PP_CENTERS;
		mBowTrainer = new BOWKMeansTrainer(mParams.vocSize, tc, retries, flags);

		mHistogramExtractor = new BOWImgDescriptorExtractor(mDetector, mHistogramMatcher);
	}

	//-----------------------------------------------------------------------------------------------------------------
	BoW::Params BoW::params() const {
		return mParams;
	}

	//-----------------------------------------------------------------------------------------------------------------
	void BoW::model(MlModel & _model) {
		mModel = &_model;
	}

	//-----------------------------------------------------------------------------------------------------------------
	MlModel & BoW::model() const {
		return *mModel;
	}

	//-----------------------------------------------------------------------------------------------------------------
	void BoW::train(std::string _imagePathTemplate, std::string _gtFile) {
		// Load dataset from paths
		unsigned index = 1;
		vector<Mat> images;
		for (;;) {	// For each image in dataset
			Mat frame = loadImage(_imagePathTemplate, index++);
			if (frame.rows == 0) {
				break;	// Can't open image.
			}
			double scaleFactor = 1;
			for (unsigned i = 0; i < mParams.nScalesTrain; i++) {
				Mat resizedImage;
				resize(frame, resizedImage, Size(), scaleFactor, scaleFactor);
				images.push_back(resizedImage);
				scaleFactor*=mParams.scaleFactor;
			}
		}
		Mat groundTruth = loadGroundTruth(_gtFile);

		// Train models
		Ptr<ml::TrainData> trainData =  createTrainData(images, groundTruth);
		mModel->trainModel(trainData);
	}

	//-----------------------------------------------------------------------------------------------------------------
	void BoW::train(const std::vector<cv::Mat> &_images, const cv::Mat &_groundTruth) {
		Ptr<ml::TrainData> trainData = createTrainData(_images, _groundTruth);;
		mModel->trainModel(trainData);
	}

	//-----------------------------------------------------------------------------------------------------------------
	std::vector<std::pair<unsigned, float>> BoW::evaluate(Mat _image) {
		std::vector<std::pair<unsigned, float>> topics;
		std::vector<Mat> descriptors;
		std::vector<Mat> histograms;
		
		Mat descriptor;
		vector<KeyPoint> keypoints;
		mDetector->detect(_image, keypoints);
		mDetector->compute(_image, keypoints, descriptor);
		Mat histogram;
		mHistogramExtractor->compute(descriptor, histogram);
		Mat results;
		mModel->predict(histogram, results);
		for (unsigned i = 0; i < results.rows; i++) {
			topics.push_back(std::pair<unsigned, float>(results.at<float>(i,1),results.at<float>(i,0)));
		}
		return topics;
	}

	//-----------------------------------------------------------------------------------------------------------------
	void BoW::save(std::string _name) {
		FileStorage fs(_name + ".yml", FileStorage::WRITE);
		fs << "vocabulary" << mCodebook;
		fs.release();
		mModel->save(_name);
	}

	//-----------------------------------------------------------------------------------------------------------------
	void BoW::load(std::string _name) {
		FileStorage fs(_name + ".yml", FileStorage::READ);
		fs["vocabulary"] >> mCodebook;
		fs.release(); 
		mHistogramExtractor->setVocabulary(mCodebook);
		mModel->load(_name);
	}

	//-----------------------------------------------------------------------------------------------------------------
	cv::Mat BoW::loadImage(std::string _filePatern, unsigned _index){
		unsigned imageId = _index;
		string fileName = _filePatern.substr(0, _filePatern.find("%d")) + to_string(_index) + _filePatern.substr(_filePatern.find("%d")+2, _filePatern.length());
		Mat img = imread(fileName, CV_LOAD_IMAGE_GRAYSCALE);	//	Load image using file path and as grayscale image.

		if(img.rows == 0)
			return img;

		double factor = 300.0/(img.rows > img.cols ? img.rows: img.cols);
		resize(img, img, Size(), factor, factor);
		return img;

	}

	//-----------------------------------------------------------------------------------------------------------------
	Ptr<ml::TrainData> BoW::createTrainData(const std::vector<cv::Mat> &_images, const cv::Mat &_groundTruth) {
		Mat descriptorsAll;
		vector<Mat> descriptorPerImg;
		unsigned index = 1;
		for (Mat image:_images) {	// For each image in dataset
			vector<KeyPoint> keypoints;
			Mat descriptors;
			mDetector->detect(image, keypoints);
			mDetector->compute(image, keypoints, descriptors);
			descriptorsAll.push_back(descriptors);
		}
		// Form codebook from all descriptors
		mCodebook = mBowTrainer->cluster(descriptorsAll);
		descriptorsAll.release();

		mHistogramExtractor->setVocabulary(mCodebook);

		// Compute histograms
		Mat data;
		vector<Mat> oriHist;
		//for (Mat image : images) {	// For each image in dataset
		for (Mat image:_images) {
			Mat descriptor;
			vector<KeyPoint> keypoints;
			mDetector->detect(image, keypoints);
			mDetector->compute(image, keypoints, descriptor);
			Mat histogram;
			mHistogramExtractor->compute(descriptor, histogram);
			data.push_back(histogram);
		}

		return ml::TrainData::create(data, ml::SampleTypes::ROW_SAMPLE, _groundTruth);
	}

	//-----------------------------------------------------------------------------------------------------------------
	void BoW::setDetector(BoW::Params::eDescriptorType) {
		Ptr<FeatureDetector> detector;
		Ptr<FeatureDetector> descriptor;

		switch (mParams.descriptorType) {
		case Params::eDescriptorType::SIFT:
			mDetector = xfeatures2d::SIFT::create();
			break;
		case Params::eDescriptorType::SURF:
			mDetector = xfeatures2d::SURF::create();
			break;
		case Params::eDescriptorType::ORB:
			mDetector = ORB::create();
			break;
		}
	}

	//-----------------------------------------------------------------------------------------------------------------
	Mat BoW::loadGroundTruth(const std::string & _gtFile) const{
		Mat groundTruth(0, 1, CV_32SC1);
		ifstream gtFile(_gtFile);
		assert(gtFile.is_open());
		for (unsigned i = 0; !gtFile.eof();i++) {
			int gtVal;
			gtFile >> gtVal;
			for (unsigned j = 0; j < mParams.nScalesTrain; j++) {
				groundTruth.push_back(gtVal);
			}

		}
		return groundTruth;
	}

	//-----------------------------------------------------------------------------------------------------------------
	// MODELS
	//-----------------------------------------------------------------------------------------------------------------

	// SVM MODEL
	void SvmModel::setParams(double _c, double _gamma, cv::ml::SVM::Types _svmType, cv::ml::SVM::KernelTypes _svmKernel, bool _autoTrain) {
		mSvm = SVM::create();
		mSvm->setType(_svmType);
		mSvm->setKernel(_svmKernel);
		mAutoTrain = _autoTrain;
		if (!_autoTrain) {
			mSvm->setGamma(_gamma);
			mSvm->setC(_c);
		}
		
	}

	//-----------------------------------------------------------------------------------------------------------------
	void SvmModel::trainModel(const cv::Ptr<cv::ml::TrainData>& _trainData) {
		if (mAutoTrain) {
			mSvm->trainAuto(_trainData, 10, ParamGrid(1,100,1.5), ParamGrid(0.01,1,2));
		}
		else {
			mSvm->train(_trainData);
		}
	}

	//-----------------------------------------------------------------------------------------------------------------
	void SvmModel::predict(const cv::Mat & _newData, cv::Mat & _result) {
		mSvm->predict(_newData, _result);
	}

	//-----------------------------------------------------------------------------------------------------------------
	void SvmModel::save(const std::string _name){
		mSvm->save(_name + ".svm");
	}

	//-----------------------------------------------------------------------------------------------------------------
	void SvmModel::load(const std::string _name) {
		mSvm = Algorithm::load<SVM>(_name + ".svm");
	}


	// LDA MODEL
	void LdaModel::setParams(double _alpha, double _beta) {
		mAlpha = _alpha;
		mBeta = _beta;
	}

	//-----------------------------------------------------------------------------------------------------------------
	void LdaModel::trainModel(const cv::Ptr<cv::ml::TrainData>& _trainData) {
		BOViL::algorithms::Corpus corpus;

		Mat samples = _trainData->getSamples();

		for (int i = 0; i < samples.rows; i++) {
			BOViL::algorithms::Document doc;
			Mat row = samples.row(i);
			double minNonZero = 2;
			for (unsigned i = 0; i < row.cols; i++) {
				if (minNonZero > row.at<float>(i) && row.at<float>(i) != 0) {
					minNonZero = row.at<float>(i);
				}
			}
			row = row/minNonZero;
			for (int j = 0; j < samples.cols; j++) {
				for (int k = 0; k < row.at<float>(j); k++) {
					doc.addWord(j);
				}
			}
			doc.shuffle();
			corpus.addDocument(doc);
		}

		mModel.train(corpus, _trainData->getClassLabels().rows, samples.cols, mAlpha, mBeta, 200);
	}

	//-----------------------------------------------------------------------------------------------------------------
	void LdaModel::predict(const cv::Mat & _newData, cv::Mat & _result) {
		vector<bool> validData;

		BOViL::algorithms::Corpus corpus;
		for (int i = 0; i < _newData.rows; i++) {
			BOViL::algorithms::Document doc;
			Mat row = _newData.row(i);
			double minNonZero = 2;
			for (unsigned i = 0; i < row.cols; i++) {
				if (isnan(row.at<float>(i)) || isinf(row.at<float>(i))) {
					validData.push_back(false);
					break;
				}
				if (minNonZero > row.at<float>(i) && row.at<float>(i) != 0) {
					minNonZero = row.at<float>(i);
				}
			}
			if (validData.size () - 1 == i) {	// Bad data
				continue;
			}
			else {
				validData.push_back(true);
			}

			row = row/minNonZero;
			for (int j = 0; j < row.cols; j++) {
				for (int k = 0; k < row.at<float>(j); k++) {
					doc.addWord(k);
				}
			}
			doc.shuffle();
			corpus.addDocument(doc);
		}

		vector<vector<double>> probs  = mModel.evaluate(corpus);

		_result = Mat(validData.size(), 2, CV_32F);
		int probIndex = 0;
		for (unsigned i = 0; i < validData.size(); i++) {
			if (validData[i]) {
				unsigned maxIndex = 0;
				double maxProb = 0.0;
				for (unsigned j = 0; j < probs[probIndex].size(); j++) {
					if (probs[probIndex][j] > maxProb) {
						maxIndex = j;
						maxProb = probs[probIndex][j];
					}
				}
				_result.at<float>(i, 0) = maxProb;
				_result.at<float>(i, 1) = maxIndex;
				probIndex++;
			}
			else {
				_result.at<float>(i, 0) = 0.0;
				_result.at<float>(i, 1) = 1;
			}
		}
	}

	//-----------------------------------------------------------------------------------------------------------------
	void LdaModel::save(const std::string _name) {
		mModel.save(_name);
	}

	//-----------------------------------------------------------------------------------------------------------------
	void LdaModel::load(const std::string _name) {
		mModel.load(_name);
	}

}	// namespace algorithm