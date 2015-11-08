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
			_imagePoints.push_back(pointBuf);
			continue;
		}
	}

	if (_imagePoints.size() != 0) {
		calcParams(_imagePoints, _arrayFrames[0].size(),_boardSize, _squareSize);
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
	mTransVectors.resize(nTransVectors);
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
	Mat frame, undistorted;
	mDriver >> frame;

	if (frame.rows != 0 && _undistort && mCalibrated) {
		undistorted = undistort(frame);
		return undistorted;
	}

	return frame;
}

//---------------------------------------------------------------------------------------------------------------------
cv::Mat Camera::undistort(const cv::Mat & _frame) {
	Mat undistorted;
	cv::undistort(_frame, undistorted, mMatrix, mDistCoeffs);;
	return undistorted;
}

//---------------------------------------------------------------------------------------------------------------------
bool Camera::isCalibrated() const {
	return mCalibrated;
}

//---------------------------------------------------------------------------------------------------------------------
void Camera::calcParams(const vector<vector<Point2f>> &_imagePoints, Size _imageSize, Size _boardSize, float _squareSize) {
	mMatrix = Mat::eye(3, 3, CV_64F);

	mDistCoeffs = Mat::zeros(8, 1, CV_64F);

	// Filter good points.
	vector<vector<Point2f>> filteredPoints = _imagePoints;
	for (unsigned i = 0; i < filteredPoints.size(); i++) {
		if (filteredPoints[i].size() !=  _boardSize.width*_boardSize.height) {
			filteredPoints.erase(filteredPoints.begin() + i);
			i--;
		}
	}

	vector<vector<Point3f> > objectPoints(1);
	for (int i = 0; i < _boardSize.height; i++) {
		for (int j = 0; j < _boardSize.width; j++) {
			objectPoints[0].push_back(Point3f(float(j*_squareSize), float(i*_squareSize), 0));
		}
	}

	objectPoints.resize(filteredPoints.size(), objectPoints[0]);
	double rms = calibrateCamera(objectPoints, filteredPoints, _imageSize, mMatrix, mDistCoeffs, mRotVectors, mTransVectors, CALIB_FIX_K4|CALIB_FIX_K5);

}

