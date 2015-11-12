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
#include <pcl/filters/statistical_outlier_removal.h>
#include <cjson/json.h>
#include <fstream>
#include <opencv2/xfeatures2d.hpp>

using namespace algorithm;
using namespace std;
using namespace cv;
using namespace cjson;

void initConfig(string _path, Json &_config);
StereoCameras * initCameras(Json &_config);

// To do in a loop
void calculatePointCloud(StereoCameras *_cameras, vector<Point3f> &_cloud, Mat &_frame1, Mat &_frame2, Json &_config);
void getSubImages(StereoCameras *_cameras, const vector<Point3f> &_cloud, const Mat &_frame1, const Mat &_frame2, Mat &_viewLeft, Mat &_viewRight);
Mat loadGroundTruth(string _path);
pcl::PointCloud<pcl::PointXYZ> filter(const pcl::PointCloud<pcl::PointXYZ> &_inputCloud, Json &_config);


void createTrainingImages(StereoCameras * _cameras, Json &_config, vector<Mat> &_images);
void showMatch(const Mat &groundTruth, const Mat &results, vector<Mat> &images);



//---------------------------------------------------------------------------------------------------------------------
int main(int _argc, char ** _argv) {
	StereoCameras *cameras;
	Json config;

	assert(_argc == 2);	// Added path to training configuration file.
	initConfig(string(_argv[1]), config);

	cameras = initCameras(config["cameras"]);

	vector<Mat> images;
	Mat groundTruth = loadGroundTruth(config["gtFile"]);
	createTrainingImages(cameras, config, images);


	if (config["recognitionSystem"]["train"]) {

		int dictionarySize = 500;
		TermCriteria tc(CV_TERMCRIT_ITER, 10, 0.001);
		int retries = 1;
		int flags = KMEANS_PP_CENTERS;
		BOWKMeansTrainer bowTrainer(dictionarySize, tc, retries, flags);

		Ptr<FeatureDetector> detector = xfeatures2d::SIFT::create();
		Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
		//auto data = bow.createTrainData(images, groundTruth);
		Mat descriptorsAll;
		unsigned index = 1;
		//for (Mat image : images) {	// For each image in dataset
									// Look for interest points to compute features in there.
		for (int k = 0; k < images.size(); k += 2) {
			Mat image = images[k];
			vector<KeyPoint> keypoints;
			Mat descriptors;
			detector->detect(image, keypoints);
			detector->compute(image, keypoints, descriptors);
			descriptorsAll.push_back(descriptors);
		}

// 		Mat descriptors32;
// 		descriptorsAll.convertTo(descriptors32, CV_32F, 1.0 / 255.0);
		Mat vocabulary = bowTrainer.cluster(descriptorsAll);


		BOWImgDescriptorExtractor histogramExtractor(detector, matcher);

		histogramExtractor.setVocabulary(vocabulary);
		FileStorage codebook(string(config["recognitionSystem"]["mlModel"]["modelPath"])+".yml", FileStorage::WRITE);
		codebook << "vocabulary" << vocabulary;


		FileStorage histogramsFile("histogramPerImgCv.yml", FileStorage::WRITE);
		index = 0;
		//Mat gt = loadGroundTruth("C:/programming/datasets/train3d/gt.txt");
		Mat data;
		vector<Mat> oriHist;
		//for (Mat image : images) {	// For each image in dataset
		for (int k = 0; k < images.size(); k += 2) {
			Mat image = images[k];
			Mat descriptor;
			vector<KeyPoint> keypoints;
			detector->detect(image, keypoints);
			detector->compute(image, keypoints, descriptor);
			Mat histogram;
			histogramExtractor.compute(descriptor, histogram);
			histogramsFile << "hist_" + to_string(index) << histogram;
			index++;
			data.push_back(histogram);
			oriHist.push_back(histogram);
		}

		Ptr<ml::TrainData> trainData = ml::TrainData::create(data, ml::SampleTypes::ROW_SAMPLE, groundTruth);

		cv::Ptr<cv::ml::SVM> mSvm;
		mSvm = cv::ml::SVM::create();
		mSvm->setType(ml::SVM::Types::C_SVC);
		mSvm->setKernel(ml::SVM::KernelTypes::RBF);
		if (config["recognitionSystem"]["mlModel"]["params"]["auto"]) {
			Json params = config["recognitionSystem"]["mlModel"]["params"];
			mSvm->trainAuto(trainData, 10, 
							cv::ml::ParamGrid(params["c_grid"](0), params["c_grid"](1), params["c_grid"](2)), 
							cv::ml::ParamGrid(params["g_grid"](0), params["g_grid"](1), params["g_grid"](2)));
		}
		else {
			mSvm->setGamma(config["recognitionSystem"]["mlModel"]["params"]["gamma"]);
			mSvm->setC(config["recognitionSystem"]["mlModel"]["params"]["c"]);
			mSvm->train(trainData);
		}
		mSvm->save(string(config["recognitionSystem"]["mlModel"]["modelPath"]));

		/*system("PAUSE");
		vector<vector<pair<unsigned, float>>> results;
		for (unsigned i = 0; i < oriHist.size(); i++) {
			Mat results;
			mSvm->predict(oriHist[i], results);

			stringstream ss;
			ss << "Image " << i << ". Label " << results.at<float>(0, 0) << ". Prob " << results.at<float>(0, 1);
			cout << ss.str() << endl;
			Mat image = images[i];
			cv::putText(image, ss.str(), cv::Point2i(30, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0), 2);
			cv::imshow("display", image);
			cv::waitKey();
		}*/

		// stuff for showing the result class
		vector<Mat> cvImages;
		string cvPath = "C:/programming/datasets/train3d/cv/";
		for (unsigned i = 0; i < 165;i++) {
			Mat frame = imread(cvPath + "view2_"+to_string(i)+".jpg");
			if(frame.rows == 0)
				break;
			else
				cvImages.push_back(frame);
		}


		for (int i = 0; i < cvImages.size(); i += 2) {
			Mat image = cvImages[i];

			Mat descriptor;
			vector<KeyPoint> keypoints;
			detector->detect(image, keypoints);
			detector->compute(image, keypoints, descriptor);
			// 			Mat descriptors32;
			// 			descriptor.convertTo(descriptors32, CV_32F, 1.0 / 255.0);
			Mat histogram;
			histogramExtractor.compute(descriptor, histogram);
			Mat results;
			mSvm->predict(histogram, results);

			showMatch(groundTruth, results, images);



			stringstream ss;
			ss << "Image " << i << ". Label " << results.at<float>(0, 0) << ". Prob " << results.at<float>(0, 1);
			cout << ss.str() << endl;
			cv::putText(image, ss.str(), cv::Point2i(30, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0), 2);
			cv::imshow("display", image);
			cv::waitKey();
		}
	}
}

void initConfig(string _path, Json & _data) {
	ifstream file;
	file.open(_path);
	assert(file.is_open());
	_data.parse(file);
}

//---------------------------------------------------------------------------------------------------------------
StereoCameras * initCameras(Json &_config) {
	StereoCameras *  cameras = new StereoCameras(_config["left"], _config["right"]);
	Json leftRoi = _config["leftRoi"];
	Json rightRoi = _config["rightRoi"];
	cameras->roi(	Rect(leftRoi["x"],leftRoi["y"],leftRoi["width"],leftRoi["height"]), 
		Rect(rightRoi["x"],rightRoi["y"],rightRoi["width"],rightRoi["height"]));
	cameras->load(_config["paramFile"]);
	cameras->rangeZ(_config["pointRanges"]["z"]["min"], _config["pointRanges"]["z"]["max"]);
	return cameras;
}

//---------------------------------------------------------------------------------------------------------------------
Mat loadGroundTruth(string _path) {
	Mat groundTruth(0, 1, CV_32SC1);
	ifstream gtFile(_path);
	assert(gtFile.is_open());
	for (unsigned i = 0; !gtFile.eof();i++) {
		int gtVal;
		gtFile >> gtVal;
		groundTruth.push_back(gtVal);
		//groundTruth.push_back(gtVal);
	}
	return groundTruth;
}

//---------------------------------------------------------------------------------------------------------------------
void calculatePointCloud(StereoCameras *_cameras, vector<Point3f> &_cloud, Mat &_frame1, Mat &_frame2, Json &_config) {
	Mat gray1, gray2;
	bool isBlurry1, isBlurry2;
	cout << "Blurriness: ";
	_frame1 = _cameras->camera(0).frame();
	if (_frame1.rows != 0) {
		cvtColor(_frame1, gray1, CV_BGR2GRAY);
		isBlurry1 = isBlurry(gray1, _config["cameras"]["blurThreshold"]);
	}
	else { return; }

	_frame2 = _cameras->camera(1).frame();
	if (_frame2.rows != 0) {
		cvtColor(_frame2, gray2, CV_BGR2GRAY);
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

		//imshow("StereoViewer", imageTogether);
		return;
	}
	else {
		_frame1 = _cameras->camera(0).undistort(_frame1);
		_frame2 = _cameras->camera(1).undistort(_frame2);
		hconcat(_frame1, _frame2, imageTogether);
		//imshow("StereoViewer", imageTogether);

		Rect leftRoi = _cameras->roi(true);
		Rect rightRoi = _cameras->roi(false);

		cvtColor(_frame1, _frame1, CV_BGR2GRAY);
		cvtColor(_frame2, _frame2, CV_BGR2GRAY);
	}

	pair<int, int> disparityRange(_config["cameras"]["disparityRange"]["min"], _config["cameras"]["disparityRange"]["max"]);
	int squareSize = _config["cameras"]["templateSquareSize"];
	int maxReprojectionError = _config["cameras"]["maxReprojectionError"];
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud = *_cameras->pointCloud(_frame1, _frame2, disparityRange, squareSize, maxReprojectionError);
	cloud = filter(cloud, _config);
	_cloud.clear();
	for (pcl::PointXYZ point : cloud) {
		_cloud.push_back(Point3f(point.x, point.y, point.z));
	}
}

//---------------------------------------------------------------------------------------------------------------------
Rect bound(vector<Point2f> _points2d) {
	int minX=99999, minY=999999, maxX=0, maxY=0;
	for (Point2f point : _points2d) {
		if(point.x < minX)
			minX = point.x;
		if(point.y < minY)
			minY = point.y;
		if(point.x > maxX)
			maxX = point.x;
		if(point.y > maxY)
			maxY = point.y;
	}

	return Rect(minX, minY, maxX-minX, maxY-minY);
}

//---------------------------------------------------------------------------------------------------------------------
void getSubImages(StereoCameras *_cameras, const vector<Point3f> &_cloud, const Mat &_frame1, const Mat &_frame2, Mat &_viewLeft, Mat &_viewRight){
	vector<Point2f> pointsLeft  = _cameras->project3dPointsWCS(_cloud, true);
	vector<Point2f> pointsRight  = _cameras->project3dPointsWCS(_cloud, false);
	
	Mat display;
	hconcat(_frame1, _frame2, display);

	for (unsigned i = 0; i < pointsLeft.size(); i++) {
		circle(display, pointsLeft[i], 3, Scalar(0,255,0));
		circle(display, pointsRight[i] + Point2f( _frame1.cols, 0), 3, Scalar(0,255,0));
	}

	Rect r1 = bound(pointsLeft);
	Rect r2 = bound(pointsRight);
	

	_viewLeft = _frame1(r1);
	_viewRight = _frame2(r2);

	rectangle(display, r1, Scalar(0,0,255));
	r2.x +=_frame1.cols;
	rectangle(display, r2, Scalar(0,0,255));

	imshow("display", display);
}

pcl::PointCloud<pcl::PointXYZ> filter(const pcl::PointCloud<pcl::PointXYZ> &_inputCloud, Json &_config) {
	pcl::PointCloud<pcl::PointXYZ> filteredCloud;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ>		outlierRemoval;
	outlierRemoval.setMeanK(_config["mapParams"]["outlierMeanK"]);
	outlierRemoval.setStddevMulThresh(_config["mapParams"]["outlierStdDev"]);
	outlierRemoval.setNegative((bool)_config["mapParams"]["outlierSetNegative"]);
	outlierRemoval.setInputCloud(_inputCloud.makeShared());
	outlierRemoval.filter(filteredCloud);
	return filteredCloud;

}

void createTrainingImages(StereoCameras * _cameras, Json &_config, vector<Mat> &_images)
{
	Mat frame1, frame2, vLeft, vRight;
	for (;;) {
		vector<Point3f> cloud;
		calculatePointCloud(_cameras, cloud, frame1, frame2, _config);
		if (cloud.size() == 0)
			break;

		getSubImages(_cameras, cloud, frame1, frame2, vLeft, vRight);
		_images.push_back(vLeft);
		_images.push_back(vRight);

		waitKey(3);
	}
}

void showMatch(const Mat &groundTruth, const Mat &results,  vector<Mat> &images) {
	int j;
	int res = results.at<float>(0, 1);
	cout << "result: " << res << endl;
	for (j = 0; j < groundTruth.rows; j++) {
	
		int gt = groundTruth.at<int>(j);
		cout << gt << " ";
		if (res == gt)
			break;
	}
	cout << endl;
	//cv::putText(imageSh, ss.str(), cv::Point2i(30, 30), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0), 2);
	j = min(j*2, 625);
	Mat image = images[j].clone();
	cv::imshow("match", image);
}
