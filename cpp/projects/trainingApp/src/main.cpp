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
void initBow(BoW &_bow, Json &_config);
StereoCameras * initCameras(Json &_config);

// To do in a loop
void calculatePointCloud(StereoCameras *_cameras, vector<Point3f> &_cloud, Mat &_frame1, Mat &_frame2, Json &_config);
void getSubImages(StereoCameras *_cameras, const vector<Point3f> &_cloud, const Mat &_frame1, const Mat &_frame2, Mat &_viewLeft, Mat &_viewRight);
Mat loadGroundTruth(string _path);
pcl::PointCloud<pcl::PointXYZ> filter(const pcl::PointCloud<pcl::PointXYZ> &_inputCloud, Json &_config);


void createTrainingImages(StereoCameras * _cameras, Json &_config, vector<Mat> &_images);
void showMatch(const Mat &groundTruth, unsigned _label, vector<Mat> &images);



//---------------------------------------------------------------------------------------------------------------------
int main(int _argc, char ** _argv) {
	StereoCameras *cameras;
	Json config;
	BoW bow;

	assert(_argc == 2);	// Added path to training configuration file.
	initConfig(string(_argv[1]), config);
	initBow(bow, config["recognitionSystem"]);
	cameras = initCameras(config["cameras"]);

	vector<Mat> images;
	Mat groundTruth = loadGroundTruth(config["gtFile"]);
	createTrainingImages(cameras, config, images);


	if (config["recognitionSystem"]["train"]) {

		bow.train(images, groundTruth);
		bow.save(config["recognitionSystem"]["mlModel"]["modelPath"]);

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
			std::vector<std::pair<unsigned, float>>  results = bow.evaluate(cvImages[i]);

			showMatch(groundTruth, results[0].first, images);

			stringstream ss;
			ss << "Image " << i << ". Label " << results[0].first << ". Prob " << results[0].second;
			cout << ss.str() << endl;
			cv::imshow("display", cvImages[i]);
			cv::waitKey();
		}
	}
	else {
		bow.load(config["recognitionSystem"]["mlModel"]["modelPath"]);

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
			std::vector<std::pair<unsigned, float>>  results = bow.evaluate(cvImages[i]);

			showMatch(groundTruth, results[0].first, images);

			stringstream ss;
			ss << "Image " << i << ". Label " << results[0].first << ". Prob " << results[0].second;
			cout << ss.str() << endl;
			cv::imshow("display", cvImages[i]);
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
void initBow(BoW &_bow, Json &_config) {
	BoW::Params params;

	if (_config.contains("multiScale")) {
		params.nScalesTrain = (int)_config["multiScale"]["nScales"];
		params.scaleFactor = _config["multiScale"]["scaleFactor"];
	}
	else {
		params.nScalesTrain = 1;
		params.scaleFactor = 1;
	}

	params.descriptorType = BoW::Params::eDescriptorType::SIFT;
	params.histMatcher = BoW::Params::eHistogramMatcher::FlannBased;
	params.vocSize = (int) _config["bovwParams"]["vocabularySize"];
	_bow.params(params);


	if (_config["mlModel"]["name"] == "SVM") {
		MlModel *model = new SvmModel();
		static_cast<SvmModel*>(model)->setParams(_config["mlModel"]["params"]["c"], _config["mlModel"]["params"]["gamma"], ml::SVM::Types::C_SVC, ml::SVM::KernelTypes::RBF, true);
		_bow.model(*model);
	}
	else if (_config["mlModel"]["name"] == "LDA") {
		MlModel *model = new LdaModel();
		static_cast<LdaModel*>(model)->setParams(_config["mlModel"]["params"]["alpha"], _config["mlModel"]["params"]["beta"]);
		_bow.model(*model);
	}
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
		groundTruth.push_back(gtVal);
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

void showMatch(const Mat &groundTruth, unsigned _label,  vector<Mat> &images) {
	int j;
	int res = _label;
	cout << "result: " << res << endl;
	for (j = 0; j < groundTruth.rows; j++) {
	
		int gt = groundTruth.at<int>(j);
		cout << gt << " ";
		if (res == gt)
			break;
	}
	cout << endl;
	j = min(j*2, 625);
	Mat image = images[j].clone();
	cv::imshow("match", image);
}
