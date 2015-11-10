///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon SoriacvImagesPath
//		Date:	2015-10-02
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "../../objectsMap/src/vision/ml/models/BoW.h"
#include "../../objectsMap/src/vision/StereoCameras.h"
#include "../../objectsMap/src/vision/ImageFilteringTools.h"

#include <sstream>
#include <Windows.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace algorithm;
using namespace std;
using namespace cv;

void initBow(BoW &_bow);
void initMap(StereoCameras *_cameras);

// To do in a loop
void calculatePointCloud(StereoCameras *_cameras, vector<Point3f> &_cloud);
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


void calculatePointCloud(StereoCameras *_cameras, vector<Point3f> &_cloud, Json &_config)
{
	Mat gray1, gray2, _frame1, _frame2;
	bool isBlurry1, isBlurry2;
	cout << "Blurriness: ";
	_frame1 = _cameras->camera(0).frame();
	cvtColor(_frame1, gray1, CV_BGR2GRAY);
	if (_frame1.rows != 0) {
		isBlurry1 = isBlurry(gray1, _config["cameras"]["blurThreshold"]);
	}
	else { return; }

	_frame2 = _cameras->camera(1).frame();
	cvtColor(_frame2, gray2, CV_BGR2GRAY);
	if (_frame2.rows != 0) {
		isBlurry2 = isBlurry(gray2, _config["cameras"]["blurThreshold"]);
	}
	else { return; }
	cout << endl;

	Mat imageTogether;
	if (isBlurry1 || isBlurry2) {
		hconcat(_frame1, _frame2, imageTogether);

		if (isBlurry1)
			putText(imageTogether, "Blurry Image", Point2i(20, 30), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 4);
		if (isBlurry2)
			putText(imageTogether, "Blurry Image", Point2i(20 + _frame1.cols, 30), FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 4);

		imshow("StereoViewer", imageTogether);

		Rect leftRoi = _cameras->roi(true);
		Rect rightRoi = _cameras->roi(false);
		// 		mGui->drawBox(leftRoi, true, 0, 255, 0);
		// 		mGui->drawBox(rightRoi, false, 0, 255, 0);
		return;
	}
	else {
		_frame1 = _cameras->camera(0).undistort(_frame1);
		_frame2 = _cameras->camera(1).undistort(_frame2);
		hconcat(_frame1, _frame2, imageTogether);
		imshow("StereoViewer", imageTogether);

		Rect leftRoi = _cameras->roi(true);
		Rect rightRoi = _cameras->roi(false);

		cvtColor(_frame1, _frame1, CV_BGR2GRAY);
		cvtColor(_frame2, _frame2, CV_BGR2GRAY);
	}

	pair<int, int> disparityRange(_config["cameras"]["disparityRange"]["min"], _config["cameras"]["disparityRange"]["max"]);
	int squareSize = _config["cameras"]["templateSquareSize"];
	int maxReprojectionError = _config["cameras"]["maxReprojectionError"];
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud = _config->pointCloud(_frame1, _frame2, disparityRange, squareSize, maxReprojectionError);
	_cloud.clear();
	for (pcl::PointXYZ point : cloud)
	{
		_cloud.push_back(Point3f(point.x, point.y, point.z));
	}
	return;

}
