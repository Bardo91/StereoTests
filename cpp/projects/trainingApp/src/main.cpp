///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon SoriacvImagesPath
//		Date:	2015-10-02
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "../../objectsMap/src/vision/ml/BoW.h"
using namespace algorithm;
using namespace std;

//---------------------------------------------------------------------------------------------------------------------
int main(int _argc, char ** _argv) {
	BoW bow;

	BoW::Params params;
	params.extractorType = BoW::Params::eExtractorType::SIFT;
	params.descriptorType = BoW::Params::eDescriptorType::SIFT;
	params.imageSizeTrain = 200;
	params.nScalesTrain = 1;
	params.scaleFactor = 0.5;
	params.vocSize = 500;

	bow.params(params);

	SvmModel svm;
	svm.setParams(2.25, 0.16384, cv::ml::SVM::Types::C_SVC,  cv::ml::SVM::KernelTypes::RBF, true);
	bow.model(svm);

	string imageTemplate = "C:/programming/demo_bagofwords/dataset/datasetTools/train/img (%d).jpg";
	string gtFile = "C:/programming/demo_bagofwords/dataset/datasetTools/train/gt.txt";
	bow.train(imageTemplate, gtFile);
	bow.save("tools");

	string cvImagesPath = "C:/programming/demo_bagofwords/dataset/datasetTools/cv/";
	vector<cv::Mat> cvImages;
	for (int i = 1;i<99999;i++) {
		cv::Mat image = cv::imread(cvImagesPath + "img (" + to_string(i) +").jpg");
		if(image.rows == 0)
			break;

		cvImages.push_back(image);
	}


	vector<vector<pair<unsigned,float>>> results;
	int index = 0;
	for (cv::Mat image : cvImages) {
		vector<cv::Rect> regions;
		regions.push_back(cv::Rect(0, 0, image.cols, image.rows));
		results.push_back(bow.evaluate(image, regions));

		std::cout << "Image " << index << ". Label " << results[index][0].first << ". Prob " << results[results.size() -1][0].second << std::endl;
		index++;
	}

	system("PAUSE");

}
