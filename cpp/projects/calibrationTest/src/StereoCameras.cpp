//
//
//
//
//

#include "Gui.h"
#include "ParallelFeatureMatcher.h"
#include "StereoCameras.h"

using namespace cv;
using namespace std;
using namespace pcl;

//---------------------------------------------------------------------------------------------------------------------
StereoCameras::StereoCameras(unsigned _indexCamera1, unsigned _indexCamera2): mCamera1(_indexCamera1), mCamera2(_indexCamera2) {
	updateGlobalRT(cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3,1, CV_64F));
}

StereoCameras::StereoCameras(string _pattern1, string _pattern2): mCamera1(_pattern1), mCamera2(_pattern2) {
	updateGlobalRT(cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(3, 1, CV_64F));
}

//---------------------------------------------------------------------------------------------------------------------
void StereoCameras::calibrate(const vector<Mat> &_calibrationImages1, const vector<Mat> &_calibrationImages2, Size _boardSize, float _squareSize) {
	vector<vector<Point2f>> imagePoints1, imagePoints2;

	mCamera1.calibrate(_calibrationImages1, _boardSize, _squareSize, imagePoints1);
	mCamera2.calibrate(_calibrationImages2, _boardSize, _squareSize, imagePoints2);

	calibrateStereo(imagePoints1, imagePoints2, _calibrationImages1[0].size(), _boardSize, _squareSize);
	mCalibrated = true;
}

//---------------------------------------------------------------------------------------------------------------------
void StereoCameras::frames(Mat & _frame1, Mat & _frame2,  eFrameFixing _fixes) {
	switch(_fixes){
	case eFrameFixing::None:
		_frame1 = mCamera1.frame();
		_frame2 = mCamera2.frame();
		break;
	case eFrameFixing::Undistort:
		_frame1 = mCamera1.frame(true);
		_frame2 = mCamera2.frame(true);
		break;
	case eFrameFixing::UndistortAndRectify:{
		_frame1 = mCamera1.frame(true);
		_frame2 = mCamera2.frame(true);
		Size imageSize = _frame1.size();
		Mat R1, R2, P1, P2, Q;
		Rect validRoi[2];
		stereoRectify(mCamera1.matrix(), mCamera1.distCoeffs(), mCamera2.matrix(), mCamera2.distCoeffs(), imageSize, mR, mT, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, imageSize, &validRoi[0], &validRoi[1]);

		Mat rmap[2][2];

		// Calculate remap functions
		initUndistortRectifyMap(mCamera1.matrix(), mCamera1.distCoeffs(), R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
		initUndistortRectifyMap(mCamera2.matrix(), mCamera2.distCoeffs(), R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

		// Undirstort and rectify images
		Mat rectifiedFrame1, rectifiedFrame2;

		remap(_frame1, _frame1, rmap[0][0], rmap[0][1], INTER_LINEAR);
		remap(_frame2, _frame2, rmap[1][0], rmap[1][1], INTER_LINEAR);
		break;
		}
	}
}

//---------------------------------------------------------------------------------------------------------------------
Mat StereoCameras::disparity(const Mat & _frame1, const Mat & _frame2, unsigned _disparityRange, unsigned _blockSize) {
	Ptr<StereoSGBM> disparityMaker = StereoSGBM::create(0, _disparityRange, _blockSize);

	Mat disparity;
	disparityMaker->compute(_frame1, _frame2, disparity);

	double minVal,maxVal;

	minMaxLoc( disparity, &minVal, &maxVal );
	disparity.convertTo(disparity, CV_8UC1, 255/(maxVal - minVal));

	return disparity;
}

//---------------------------------------------------------------------------------------------------------------------
void StereoCameras::roi(cv::Rect _leftRoi, cv::Rect _rightRoi) {
	mLeftRoi	= _leftRoi;
	mRightRoi	= _rightRoi;
}

//---------------------------------------------------------------------------------------------------------------------
cv::Rect StereoCameras::roi(bool _isLeft) {
	if(_isLeft)
		return mLeftRoi;
	else
		return mRightRoi;
}

//---------------------------------------------------------------------------------------------------------------------
PointCloud<PointXYZ>::Ptr StereoCameras::pointCloud(const cv::Mat &_frame1, const cv::Mat &_frame2, pair<int, int> _disparityRange, int _squareSize, int _maxReprojectionError) {
	// Compute keypoint only in first image
	vector<Point2i> keypoints;
	computeFeatures(_frame1, keypoints);

	std::cout << "Features computed in frame1: " << keypoints.size() << std::endl;

	// Compute projection of epipolar lines into second image.
	std::vector<cv::Vec3f> epilines;
	computeEpipoarLines(keypoints, epilines);

	// For each epipolar line calculate equivalent feature by template matching.
	Rect secureRegion(	_squareSize/2 + 1, 
						_squareSize/2 + 1, 
						_frame2.cols - 2*(_squareSize/2 + 1), 
						_frame2.rows - 2*(_squareSize/2 + 1));
	Rect validLeft	= mLeftRoi & secureRegion;
	Rect validRight = mRightRoi & secureRegion;


	const unsigned cNumProcs = 8;
	vector<vector<Point2i>> vpoints1(cNumProcs), vpoints2(cNumProcs);

	// Match features using ParallelFeatureMatcher Class
	parallel_for_(Range(0,cNumProcs), ParallelFeatureMatcher(_frame1, _frame2, keypoints, epilines, _disparityRange, _squareSize, vpoints1, vpoints2, validLeft, validRight));

	vector<Point2i> points1, points2;

	for (vector<Point2i> v : vpoints1) {
		points1.insert(points1.end(), v.begin(), v.end());
	}

	for (vector<Point2i> v : vpoints2) {
		points2.insert(points2.end(), v.begin(), v.end());
	}

	std::cout << "Features matched: " << points1.size() << std::endl;
	// Triangulate points using features in both images.
	vector<Point3f> points3d = triangulate(points1, points2);
	// Filter points using reprojection.
	vector<Point3f> points3dFiltered = filterPoints(_frame1, _frame2, points1, points2, points3d, _maxReprojectionError);
	std::cout << "Points Filtered with reprojection: " << points3dFiltered.size() << std::endl;

	// Filter points by range.
	PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>());
	for (unsigned i = 0; i < points3dFiltered.size(); i++) {
		if (points3dFiltered[i].z > mMinZ && points3dFiltered[i].z < mMaxZ) {
			PointXYZ point(points3dFiltered[i].x, points3dFiltered[i].y, points3dFiltered[i].z);
			cloud->push_back(point);
		}
	}

	return cloud;
}

//---------------------------------------------------------------------------------------------------------------------
std::vector<cv::Point2f> StereoCameras::project3dPointsWCS(const std::vector<cv::Point3f>& _points, bool _isLeftCamera) {
	std::vector<cv::Point2f> points2d;

	if (_isLeftCamera) {
		projectPoints(_points, mGlobalR, mGlobalT, camera(0).matrix(), camera(0).distCoeffs(), points2d);
	}
	else {
		projectPoints(_points, mR*mGlobalR, mT + mR*mGlobalT,  camera(1).matrix(), camera(1).distCoeffs(), points2d);

	}
	return points2d;
}

//---------------------------------------------------------------------------------------------------------------------
Camera & StereoCameras::camera(unsigned _index) {
	assert(_index < 2);
	if(_index == 0)
		return mCamera1;
	else {
		return mCamera2;
	}
}

//---------------------------------------------------------------------------------------------------------------------
Mat StereoCameras::rotation() const{
	return mR;
}

//---------------------------------------------------------------------------------------------------------------------
Mat StereoCameras::translation() const{
	return mT;
}

//---------------------------------------------------------------------------------------------------------------------
Mat StereoCameras::essentialMatrix() const{
	return mE;
}

//---------------------------------------------------------------------------------------------------------------------
Mat StereoCameras::fundamentalMatrix() const{
	return mF;
}


//---------------------------------------------------------------------------------------------------------------------
bool StereoCameras::isCalibrated() const {
	return mCalibrated;
}

void StereoCameras::save(string _filePath) {
	mCamera1.saveParams(_filePath + "_cam1");
	mCamera2.saveParams(_filePath + "_cam2");
	
	FileStorage fs(_filePath + "_stero.yml", FileStorage::WRITE);
	fs << "mR" << mR;
	fs << "mT" << mT;
	fs << "mE" << mE;
	fs << "mF" << mF;
}

void StereoCameras::load(string _filePath) {
	mCamera1.params(_filePath + "_cam1");
	mCamera2.params(_filePath + "_cam2");

	FileStorage fs(_filePath + "_stero.yml", FileStorage::READ);
	fs["mR"] >> mR;
	fs["mT"] >> mT;
	fs["mE"] >> mE;
	fs["mF"] >> mF;

	mCalibrated = true;
}

void StereoCameras::updateGlobalRT(const cv::Mat &_R,const cv::Mat &_T)
{
	mGlobalR = _R;
	mGlobalT = _T;
}

cv::Mat StereoCameras::globalRotation() const
{
	return mGlobalR;
}

cv::Mat StereoCameras::globalTranslation() const
{
	return mGlobalT;
}

void StereoCameras::rangeZ(double _min, double _max){ 
	mMinZ = _min;
	mMaxZ = _max;
}

//---------------------------------------------------------------------------------------------------------------------
//		Private methods
//---------------------------------------------------------------------------------------------------------------------
void StereoCameras::calibrateStereo(const vector<vector<Point2f>> &_imagePoints1, const vector<vector<Point2f>> &_imagePoints2, Size _imageSize, Size _boardSize, float _squareSize) {
	vector<vector<Point2f>> filteredPoints1 = _imagePoints1;
	vector<vector<Point2f>> filteredPoints2 = _imagePoints2;

	for (unsigned i = 0; i < filteredPoints1.size(); i++) {
		if (filteredPoints1[i].size() !=   _boardSize.width*_boardSize.height || filteredPoints2[i].size() !=   _boardSize.width*_boardSize.height) {
			filteredPoints1.erase(filteredPoints1.begin() + i);
			filteredPoints2.erase(filteredPoints2.begin() + i);
			i--;
		}
	}
	
	
	vector<vector<Point3f> > objectPoints(1);
	for (int i = 0; i < _boardSize.height; i++) {
		for (int j = 0; j < _boardSize.width; j++) {
			objectPoints[0].push_back(Point3f(float(j*_squareSize), float(i*_squareSize), 0));
		}
	}
	objectPoints.resize(filteredPoints1.size(),objectPoints[0]);

	stereoCalibrate(objectPoints, filteredPoints1, filteredPoints2, mCamera1.matrix(), mCamera1.distCoeffs(), mCamera2.matrix(), mCamera2.distCoeffs(),_imageSize, mR, mT, mE, mF);
}

//---------------------------------------------------------------------------------------------------------------------
void StereoCameras::computeFeatures(const Mat &_frame, vector<Point2i> &_features){
	goodFeaturesToTrack(_frame, _features, 5000, 0.001,0.5);
}

//---------------------------------------------------------------------------------------------------------------------
void StereoCameras::computeEpipoarLines(const vector<Point2i> &_points, vector<Vec3f> &_epilines){
	computeCorrespondEpilines(_points, 1, fundamentalMatrix(), _epilines);
}

//---------------------------------------------------------------------------------------------------------------------
vector<Point3f> StereoCameras::triangulate(const vector<Point2i> &_points1, const vector<Point2i> &_points2) {
	Mat pnts3D	(4,_points1.size(),CV_64F);
	Mat cam1pnts(2,_points1.size(),CV_64F);
	Mat cam2pnts(2,_points1.size(),CV_64F);

	for (unsigned i = 0; i < _points1.size(); i++) {
		cam1pnts.at<double>(0,i) = _points1[i].x;
		cam1pnts.at<double>(1,i) = _points1[i].y;
		cam2pnts.at<double>(0,i) = _points2[i].x;
		cam2pnts.at<double>(1,i) = _points2[i].y;
	}

	Mat I = Mat::eye(3,4, CV_64F);
	Mat extrinsicMatrix(3,4, CV_64F);
	mR.copyTo(extrinsicMatrix.rowRange(0,3).colRange(0,3));
	mT.copyTo(extrinsicMatrix.rowRange(0,3).col(3));

	triangulatePoints(mCamera1.matrix()*I, mCamera2.matrix()*extrinsicMatrix, cam1pnts, cam2pnts, pnts3D);

	vector<Point3f> points3d;
	for (int i = 0 ; i < pnts3D.cols ; i++) {
		float w = (float) pnts3D.at<double>(3,i);
		float x = (float) pnts3D.at<double>(0,i)/w;
		float y = (float) pnts3D.at<double>(1,i)/w;
		float z = (float) pnts3D.at<double>(2,i)/w;
		points3d.push_back(Point3f(x,y,z));
	}

	return points3d;
}

//---------------------------------------------------------------------------------------------------------------------
vector<Point3f> StereoCameras::filterPoints(const Mat &_frame1, const Mat &_frame2, const vector<Point2i> &_points1, const vector<Point2i> &_points2, const vector<Point3f> &_points3d, int _maxReprojectionError){
	vector<Point2f> reprojection1, reprojection2;
	projectPoints(_points3d, Mat::eye(3, 3, CV_64F), Mat::zeros(3, 1, CV_64F), mCamera1.matrix(), mCamera1.distCoeffs(), reprojection1);
	projectPoints(_points3d, rotation(), translation(), mCamera2.matrix(), mCamera2.distCoeffs(), reprojection2);

	vector<Point3f> filteredPoints3d;
	for (unsigned i = 0; i < _points3d.size(); i++) {
		double rError1 = sqrt(pow(reprojection1[i].x - _points1[i].x, 2)
							+ pow(reprojection1[i].y - _points1[i].y, 2));
		double rError2 = sqrt(pow(reprojection2[i].x - _points2[i].x, 2)
							+ pow(reprojection2[i].y - _points2[i].y, 2));

		if (rError1 < _maxReprojectionError && rError2 < _maxReprojectionError) {
			filteredPoints3d.push_back(_points3d[i]);
		}
	}

	return filteredPoints3d;
}

//---------------------------------------------------------------------------------------------------------------------
