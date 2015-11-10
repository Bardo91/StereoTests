///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon SoriacvImagesPath
//		Date:	2015-10-02
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "../../objectsMap/src/vision/ml/models/BoW.h"
#include "../../objectsMap/src/vision/StereoCameras.h"

#include <sstream>
#include <Windows.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace algorithm;
using namespace std;
using namespace cv;

void initBow(BoW &_bow);
void initMap(StereoCameras &_cameras);

// To do in a loop
void calculatePointCloud(StereoCameras &_cameras, vector<Point3f> &_cloud);
void reprojectPoints(const vector<Point3f> &_cloud, vector<Point2f> &_leftPoints, vector<Point2f> &_rightPoints);
void getSubImagesAndGroundTruth();

// For training
void trainModel(BoW &_bow, vector<Mat> &_images, Mat &_groundTruth);


//---------------------------------------------------------------------------------------------------------------------
int main(int _argc, char ** _argv) {
	BoW bow;



	/*BoW bow;

	BoW::Params params;
	params.extractorType = BoW::Params::eExtractorType::SIFT;
	params.descriptorType = BoW::Params::eDescriptorType::SIFT;
	params.imageSizeTrain = 640;
	params.nScalesTrain = 1;
	params.scaleFactor = 0.5;
	params.vocSize = 500;

	bow.params(params);

	SvmModel svm;
	svm.setParams(2.25, 0.16384, cv::ml::SVM::Types::C_SVC,  cv::ml::SVM::KernelTypes::RBF);
	bow.model(svm);

	string imageTemplate = "C:/programming/datasets/objectsMaps/training (%d).jpg";
	string gtFile = "C:/programming/datasets/objectsMaps/gt.txt";
	bow.train(imageTemplate, gtFile);
	bow.save("canjuiboxcrasen2");

	string cvImagesPath = "C:/programming/datasets/objectsMaps/";
	vector<cv::Mat> cvImages;
	for (int i = 1;i<999;i=i+20) {
		cv::Mat image = cv::imread(cvImagesPath + "training (" + to_string(i) +").jpg");
		if(image.rows == 0)
			break;

		cv::resize(image, image, cv::Size(320,240));
		cvImages.push_back(image);
	}


	vector<vector<pair<unsigned,float>>> results;
	int index = 0;
	for (cv::Mat image : cvImages) {
		vector<cv::Rect> regions;
		regions.push_back(cv::Rect(0, 0, image.cols, image.rows));
		results.push_back(bow.evaluate(image, regions));

		stringstream ss;

		ss << "Image " << index << ". Label " << results[index][0].first << ". Prob " << results[results.size() -1][0].second;

		cv::putText(image, ss.str(), cv::Point2i(30,30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0), 2);
		cv::imshow("display", image);
		cv::waitKey();
		index++;
	}

	system("PAUSE");*/

}
