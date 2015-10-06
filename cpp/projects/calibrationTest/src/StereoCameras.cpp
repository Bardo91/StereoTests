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

//---------------------------------------------------------------------------------------------------------------------
void StereoCameras::calibrate(const vector<Mat> &_calibrationImages1, const vector<Mat> &_calibrationImages2, Size _boardSize, float _squareSize) {
	vector<vector<Point2f>> imagePoints1, imagePoints2;

	mCamera1.calibrate(_calibrationImages1, _boardSize, _squareSize, imagePoints1);
	mCamera2.calibrate(_calibrationImages2, _boardSize, _squareSize, imagePoints2);

	calibrateStereo(imagePoints1, imagePoints2, Size(_calibrationImages1[0].rows, _calibrationImages1[0].cols), _boardSize, _squareSize);
}

//---------------------------------------------------------------------------------------------------------------------
void StereoCameras::frames(Mat & _frame1, Mat & _frame2) {
	_frame1 = mCamera1.frame();
	_frame2 = mCamera2.frame();
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
	vector<vector<Point3f> > objectPoints(1);
	for (int i = 0; i < _boardSize.height; i++) {
		for (int j = 0; j < _boardSize.width; j++) {
			objectPoints[0].push_back(Point3f(float(j*_squareSize), float(i*_squareSize), 0));
		}
	}
	objectPoints.resize(_imagePoints1.size(),objectPoints[0]);

	stereoCalibrate(objectPoints, _imagePoints1, _imagePoints2, mCamera1.matrix(), mCamera1.distCoeffs(), mCamera2.matrix(), mCamera2.distCoeffs(),_imageSize, mR, mT, mE, mF);
}
