//
//
//
//
//

#include "StereoCameras.h"

using namespace cv;
using namespace std;

//---------------------------------------------------------------------------------------------------------------------
StereoCameras::StereoCameras(unsigned _indexCamera1, unsigned _indexCamera2): mCamera1(_indexCamera1), mCamera2(_indexCamera2) {

}

StereoCameras::StereoCameras(std::string _pattern1, std::string _pattern2): mCamera1(_pattern1), mCamera2(_pattern2) {
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
		std::cout << rmap[0][0] << std::endl;
		std::cout << rmap[0][1] << std::endl;
		std::cout << rmap[1][0] << std::endl;
		std::cout << rmap[1][1] << std::endl;

		remap(_frame1, _frame1, rmap[0][0], rmap[0][1], INTER_LINEAR);
		remap(_frame2, _frame2, rmap[1][0], rmap[1][1], INTER_LINEAR);
		break;
		}
	}
}

Mat StereoCameras::disparity(const Mat & _frame1, const Mat & _frame2, unsigned _disparityRange, unsigned _blockSize) {
	Ptr<StereoSGBM> disparityMaker = StereoSGBM::create(0, _disparityRange, _blockSize);

	Mat disparity;
	disparityMaker->compute(_frame1, _frame2, disparity);

	double minVal,maxVal;

	minMaxLoc( disparity, &minVal, &maxVal );
	disparity.convertTo(disparity, CV_8UC1, 255/(maxVal - minVal));

	return disparity;
}

vector<Point3f> StereoCameras::triangulate(const vector<Point2i> &_points1, const vector<Point2i> &_points2) {
	cv::Mat pnts3D	(4,_points1.size(),CV_64F);
	cv::Mat cam1pnts(2,_points1.size(),CV_64F);
	cv::Mat cam2pnts(2,_points1.size(),CV_64F);

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
	for (unsigned i = 0 ; i < pnts3D.cols ; i++) {
		float w = (float) pnts3D.at<double>(3,i);
		float x = (float) pnts3D.at<double>(0,i)/w;
		float y = (float) pnts3D.at<double>(1,i)/w;
		float z = (float) pnts3D.at<double>(2,i)/w;
		points3d.push_back(Point3f(x,y,z));
	}

	return points3d;
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
Mat StereoCameras::rotation(unsigned _index){
	if(_index == 0){
		return Mat::eye(3,3, CV_64F);
	}else{
		return mR;
	}
}

//---------------------------------------------------------------------------------------------------------------------
Mat StereoCameras::translation(unsigned _index){
	if(_index == 0){
		return Mat::zeros(3,1,CV_64F);
	}else{
		return mT;
	}
}

//---------------------------------------------------------------------------------------------------------------------
bool StereoCameras::isCalibrated() const {
	return mCalibrated;
}

void StereoCameras::save(std::string _filePath) {
	mCamera1.saveParams(_filePath + "_cam1");
	mCamera2.saveParams(_filePath + "_cam2");
	
	FileStorage fs(_filePath + "_stero.yml", FileStorage::WRITE);
	fs << "mR" << mR;
	fs << "mT" << mT;
	fs << "mE" << mE;
	fs << "mF" << mF;
}

void StereoCameras::load(std::string _filePath) {
	mCamera1.params(_filePath + "_cam1");
	mCamera2.params(_filePath + "_cam2");

	FileStorage fs(_filePath + "_stero.yml", FileStorage::READ);
	fs["mR"] >> mR;
	fs["mT"] >> mT;
	fs["mE"] >> mE;
	fs["mF"] >> mF;

	mCalibrated = true;
}

//---------------------------------------------------------------------------------------------------------------------
void StereoCameras::calibrateStereo(const vector<vector<Point2f>> &_imagePoints1, const vector<vector<Point2f>> &_imagePoints2, Size _imageSize, Size _boardSize, float _squareSize) {
	vector<vector<Point2f>> filteredPoints1 = _imagePoints1;
	vector<vector<Point2f>> filteredPoints2 = _imagePoints2;

	for (unsigned i = 0; i < filteredPoints1.size(); i++) {
		if (filteredPoints1[i].size() ==  0 || filteredPoints2[i].size() ==  0) {
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
