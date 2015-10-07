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

	calibrateStereo(imagePoints1, imagePoints2, Size(_calibrationImages1[0].rows, _calibrationImages1[0].cols), _boardSize, _squareSize);
	mCalibrated = true;
}

//---------------------------------------------------------------------------------------------------------------------
void StereoCameras::frames(Mat & _frame1, Mat & _frame2, bool _undistortAndRectificate) {
	_frame1 = mCamera1.frame();
	_frame2 = mCamera2.frame();
	if(_frame1.rows == 0 || _frame2.rows == 0)
		return;

	if (_undistortAndRectificate && mCalibrated) {
		Size imageSize = Size(_frame1.rows, _frame1.cols);
		Mat R1, R2, P1, P2, Q;
		Rect validRoi[2];
		stereoRectify(mCamera1.matrix(), mCamera1.distCoeffs(), mCamera2.matrix(), mCamera2.distCoeffs(), imageSize, mR, mT, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

		Mat rmap[2][2];

		initUndistortRectifyMap(mCamera1.matrix(), mCamera1.distCoeffs(), R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
		initUndistortRectifyMap(mCamera2.matrix(), mCamera2.distCoeffs(), R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

		Mat rectifiedFrame1, rectifiedFrame2;
		remap(_frame1, _frame1, rmap[0][0], rmap[0][1], INTER_LINEAR);
		remap(_frame2, _frame2, rmap[1][0], rmap[1][1], INTER_LINEAR);
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
	vector<Point3f> points3d;
	triangulatePoints(mCamera1.matrix(), mCamera2.matrix(), _points1, _points2, points3d);
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
