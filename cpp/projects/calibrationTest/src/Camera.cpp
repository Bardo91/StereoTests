#include "Camera.h"

using namespace cv;
using namespace std;

//---------------------------------------------------------------------------------------------------------------------
Camera::Camera(unsigned _camIndex): mDriver(_camIndex){
}

Camera::Camera(std::string _pattern): mDriver(_pattern) {
}

//---------------------------------------------------------------------------------------------------------------------
bool Camera::calibrate(const vector<Mat> &_arrayFrames, Size _boardSize, float _squareSize, vector<vector<Point2f>> &_imagePoints) {
	_imagePoints.resize(0);

	for (Mat frame : _arrayFrames) {
		vector<Point2f> pointBuf;
		Mat frameGray;
		cvtColor(frame, frameGray, COLOR_BGR2GRAY);
		bool found = findChessboardCorners( frame, _boardSize, pointBuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
		if (found) {
			cornerSubPix(frameGray, pointBuf, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
			_imagePoints.push_back(pointBuf);
		}
		else {
			continue;
		}
	}

	if (_imagePoints.size() != 0) {
		calcParams(_imagePoints, Size(_arrayFrames[0].rows, _arrayFrames[0].cols),_boardSize, _squareSize);
	}
	else {
		return false;
	}
	mCalibrated = true;
	return true;
}

//---------------------------------------------------------------------------------------------------------------------
void Camera::params(cv::Mat _matrix, cv::Mat _distCoefs) {
	mMatrix = _matrix;
	mDistCoeffs = _distCoefs;
	mCalibrated = true;
}

//---------------------------------------------------------------------------------------------------------------------
void Camera::params(std::string _paramFile) {
	FileStorage fs(_paramFile + ".yml", FileStorage::READ);
	fs["Matrix"] >> mMatrix;
	fs["DistCoeffs"] >> mDistCoeffs;

	int nRotVectors;
	fs["nRotVectors"] >> nRotVectors;
	mRotVectors.resize(nRotVectors);
	for (int i = 0; i < (int) nRotVectors; i++) {
		fs["RotVector" +to_string(i)] >> mRotVectors[i];
	}

	int nTransVectors;
	fs["nTransVectors"] >> nTransVectors;
	mRotVectors.resize(nRotVectors);
	for (int i = 0; i < (int) nTransVectors; i++) {
		fs["TransVector" +to_string(i)] >> mTransVectors[i];
	}
	mCalibrated = true;
}

//---------------------------------------------------------------------------------------------------------------------
void Camera::saveParams(std::string _paramFile) {
	FileStorage fs(_paramFile + ".yml", FileStorage::WRITE);
	fs << "Matrix" << mMatrix;
	fs << "DistCoeffs" << mDistCoeffs;

	fs << "nRotVectors" << (int) mRotVectors.size();
	for (int i = 0; i < (int) mRotVectors.size(); i++) {
		fs << "RotVector" +to_string(i) << mRotVectors[i];
	}

	fs << "nTransVectors" << (int) mTransVectors.size();
	for (int i = 0; i < (int) mTransVectors.size(); i++) {
		fs << "TransVector" + to_string(i) << mTransVectors[i];
	}
}

//---------------------------------------------------------------------------------------------------------------------
Mat Camera::matrix() const {
	return mMatrix;
}

//---------------------------------------------------------------------------------------------------------------------
Mat Camera::distCoeffs() const {
	return mDistCoeffs;
}

//---------------------------------------------------------------------------------------------------------------------
vector<Mat> Camera::rotVectors() const {
	return mRotVectors;
}

//---------------------------------------------------------------------------------------------------------------------
vector<Mat> Camera::transVectors() const {
	return mTransVectors;
}

//---------------------------------------------------------------------------------------------------------------------
Mat Camera::frame(bool _undistort) {
	Mat frame;
	mDriver >> frame;

	if (_undistort && mCalibrated) {
		undistort(frame, frame, mMatrix, mDistCoeffs);
	}

	return frame;
}

//---------------------------------------------------------------------------------------------------------------------
void Camera::calcParams(const vector<vector<Point2f>> &_imagePoints, Size _imageSize, Size _boardSize, float _squareSize) {
	mMatrix = Mat::eye(3, 3, CV_64F);

	mDistCoeffs = Mat::zeros(8, 1, CV_64F);

	vector<vector<Point3f> > objectPoints(1);
	for (int i = 0; i < _boardSize.height; i++) {
		for (int j = 0; j < _boardSize.width; j++) {
			objectPoints[0].push_back(Point3f(float(j*_squareSize), float(i*_squareSize), 0));
		}
	}

	objectPoints.resize(_imagePoints.size(), objectPoints[0]);
	double rms = calibrateCamera(objectPoints, _imagePoints, _imageSize, mMatrix, mDistCoeffs, mRotVectors, mTransVectors, CALIB_FIX_K4|CALIB_FIX_K5);

}

