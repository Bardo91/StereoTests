///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon SoriacvImagesPath
//		Date:	2015-10-02
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <sstream>
#include <Windows.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <cjson/json.h>
#include <fstream>
#include <opencv2/xfeatures2d.hpp>

#include <libsvmpp/Svmpp.h>

#include <StereoLib/ml/BoW.h>
#include <StereoLib/StereoCameras.h>
#include <StereoLib/ImageFilteringTools.h>

//using namespace algorithm;
using namespace std;
using namespace cv;
using namespace cjson;

void initConfig(string _path, Json &_config);
StereoCameras * initCameras(Json &_config);

// To do in a loop
void calculatePointCloud(StereoCameras *_cameras, vector<Point3f> &_cloud, Mat &_frame1, Mat &_frame2, Json &_config);
void getSubImages(StereoCameras *_cameras, const vector<Point3f> &_cloud, const Mat &_frame1, const Mat &_frame2, Mat &_viewLeft, Mat &_viewRight);
vector<double> loadGroundTruth(string _path);
pcl::PointCloud<pcl::PointXYZ> filter(const pcl::PointCloud<pcl::PointXYZ> &_inputCloud, Json &_config);


void createTrainingImages(StereoCameras * _cameras, Json &_config, vector<Mat> &_images);
void showMatch(const Mat &groundTruth, const Mat &results, vector<Mat> &images);



//---------------------------------------------------------------------------------------------------------------------
int main(int _argc, char ** _argv) {
	StereoCameras *cameras;
	Json config;
	BoW bow;
	assert(_argc == 2);	// Added path to training configuration file.
	initConfig(string(_argv[1]), config);

	cameras = initCameras(config["cameras"]);
	bow.params(config["recognitionSystem"]["bow"]);

	if (config["train"]) {
		vector<Mat> images;
		if (config["generateTrainSet"]) {
			createTrainingImages(cameras, config, images);
		}
		else {
			int index = 0;
			for (;;) {
				string left = string(config["cameras"]["left1"]);
				string right = string(config["cameras"]["right1"]);
				left = left.substr(0,left.find("%d")) +to_string(index)+ left.substr(left.find("%d")+2);
				right = right.substr(0,right.find("%d")) +to_string(index)+ right.substr(right.find("%d")+2);
				index++;
				Mat frame1 = imread(left);	//cameras->camera(0).frame();
				Mat frame2 = imread(right);	//cameras->camera(1).frame();
				if (frame1.rows != 0 && frame2.rows != 0) {
					//resize(frame1, frame1, Size(150,150));
					//resize(frame2, frame2, Size(150,150));
					images.push_back(frame1);
					images.push_back(frame2);
					Mat display;
					imshow("display1", frame1);
					imshow("display2", frame2);
					waitKey(3);
				}
				else {
					break;
				}
			}
		}

		vector<double> groundTruth = loadGroundTruth(config["gtFile"]);
		
		bow.train(images, groundTruth);
		bow.save(config["recognitionSystem"]["bow"]["modelPath"]);
	}
	else {
		bow.load(config["recognitionSystem"]["bow"]["modelPath"]);

		vector<Mat> cvImages;
		string path = string(config["cameras"]["left1"]);
		for (unsigned i = 1; i < 9999;i++) {
			Mat frame = imread(path.substr(0,path.find("%d")) +to_string(i)+ path.substr(path.find("%d")+2));
			if(frame.rows == 0)
				break;
			else {
				//resize(frame, frame, Size(150, 150));
				cvImages.push_back(frame);
			}
		}

		for (int i = 0; i < cvImages.size(); i++) {

			std::vector<double> probs = bow.evaluate(cvImages[i]);
			double maxLabel = max_element(probs.begin(), probs.end()) - probs.begin();
			cout << "Image " << i << ". Label " << maxLabel << ". Prob: ";
			for (unsigned n = 0; n < probs.size(); n++) {
				cout << probs[n] << ", ";
			}
			cout << endl;
			cv::imshow("display", cvImages[i]);
			cv::waitKey();
		}
	}
}

//---------------------------------------------------------------------------------------------------------------
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
vector<double> loadGroundTruth(string _path) {
	vector<double> groundTruth;
	ifstream gtFile(_path);
	assert(gtFile.is_open());
	for (unsigned i = 0; !gtFile.eof();i++) {
		int gtVal;
		gtFile >> gtVal;
		groundTruth.push_back(gtVal);
		groundTruth.push_back(gtVal);	// load twice, pair of images.
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
	int maxTemplateScore = _config["cameras"]["maxTemplateScore"];
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud = *_cameras->pointCloud(_frame1, _frame2, disparityRange, squareSize,maxTemplateScore, maxReprojectionError);
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

	return Rect(minX-10, minY-10, maxX-minX+20, maxY-minY+20);
}

//---------------------------------------------------------------------------------------------------------------------
void getSubImages(StereoCameras *_cameras, const vector<Point3f> &_cloud, const Mat &_frame1, const Mat &_frame2, Mat &_viewLeft, Mat &_viewRight){
	vector<Point2f> pointsLeft = _cameras->project3dPoints(_cloud, true, { 0,0,0,1 }, Eigen::Quaternionf(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ())));
	vector<Point2f> pointsRight  = _cameras->project3dPoints(_cloud, false, { 0,0,0,1 }, Eigen::Quaternionf(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ())));
	
	Mat display;
	hconcat(_frame1, _frame2, display);

	for (unsigned i = 0; i < pointsLeft.size(); i++) {
		circle(display, pointsLeft[i], 3, Scalar(0,255,0));
		circle(display, pointsRight[i] + Point2f( _frame1.cols, 0), 3, Scalar(0,255,0));
	}

	Rect r1 = bound(pointsLeft);
	Rect r2 = bound(pointsRight);
	
	Rect validRoi(0,0,_frame1.cols, _frame1.rows);

	_viewLeft = _frame1(r1&validRoi);
	_viewRight = _frame2(r2&validRoi);

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

void createTrainingImages(StereoCameras * _cameras, Json &_config, vector<Mat> &_images){
	Mat frame1, frame2, vLeft, vRight;
	CreateDirectory("CroppedSet", NULL);
	int index = 0;
	for (;;) {
		vector<Point3f> cloud;
		calculatePointCloud(_cameras, cloud, frame1, frame2, _config);
		if (cloud.size() == 0)
			break;

		getSubImages(_cameras, cloud, frame1, frame2, vLeft, vRight);
		_images.push_back(vLeft);
		_images.push_back(vRight);

		cvtColor(vLeft, vLeft, CV_GRAY2BGR);
		cvtColor(vRight, vRight, CV_GRAY2BGR);

		imwrite("CroppedSet/img_left_"+to_string(index)  + ".jpg", vLeft);
		imwrite("CroppedSet/img_right_"+to_string(index) + ".jpg", vRight);
		index++;

		waitKey(3);
	}
}
