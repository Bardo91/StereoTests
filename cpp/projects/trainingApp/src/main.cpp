///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-10-02
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <vision/ml/BoW.h>

using namespace algorithm;
using namespace std;

//---------------------------------------------------------------------------------------------------------------------
int main(int _argc, char ** _argv) {
	BoW bow;

	BoW::Params params;
	params.extractorType = BoW::Params::eExtractorType::SIFT;
	params.descriptorType = BoW::Params::eDescriptorType::SIFT;
	params.imageSizeTrain = 640;
	params.nScalesTrain = 3;
	params.scaleFactor = 0.5;
	params.vocSize = 500;

	bow.params(params);

	SvmModel svm;
	svm.setParams(0.1,2,cv::ml::SVM::Types::C_SVC, cv::ml::SVM::KernelTypes::RBF, true);
	bow.model(svm);

	string imageTemplate = "";
	string gtFile = "";
	bow.train(imageTemplate, gtFile);
}
