#include "Camera.h"

using namespace cv;
using namespace std;

Camera::Camera(unsigned _camIndex) {
}

bool Camera::calibrate(vector<Mat> _arrayFrames, Size _boardSize, double _squareSize) {
	vector<vector<Point2f>> imagePoints;
	for (Mat frame : _arrayFrames) {
		vector<Point2f> pointBuf;
		Mat frameGray;
		cvtColor(frame, frameGray, COLOR_BGR2GRAY);
		bool found = findChessboardCorners( frame, _boardSize, pointBuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
		if (found) {
			cornerSubPix(frameGray, pointBuf, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
			imagePoints.push_back(pointBuf);
		}
		else {
			continue;
		}
	}

	if (imagePoints.size() != 0) {
		calcParams(imagePoints, Size(_arrayFrames[0].rows, _arrayFrames[0].cols),_boardSize, _squareSize);
	}
	else {
		return false;
	}
}

void Camera::params(cv::Mat _matrix, cv::Mat _distCoefs, cv::Mat _rotVectors, cv::Mat _transVectors) {

}

void Camera::params(std::string _paramFile) {

}

Mat Camera::matrix() {
	return mMatrix;
}

Mat Camera::distCoeffs() {
	return mDistCoeffs;
}

Mat Camera::rotVectors() {
	return mRotVectors;
}

Mat Camera::transVectors() {
	return mTransVectors;
}

Mat Camera::frame() {
	Mat frame;
	mDriver >> frame;
	return frame;
}

void Camera::calcParams(vector<vector<Point2f>> _imagePoints, Size _imageSize, Size _boardSize, float _squareSize) {
	mMatrix = Mat::eye(3, 3, CV_64F);

	mDistCoeffs = Mat::zeros(8, 1, CV_64F);

	vector<vector<Point3f> > objectPoints(1);
	
	vector<Point3f> corners;
	vector<vector<Point3f> > objectPoints(1);
	for (int i = 0; i < _boardSize.height; i++) {
		for (int j = 0; j < _boardSize.width; j++) {
			corners.push_back(Point3f(float(j*_squareSize), float(i*_squareSize), 0));
		}
	}

	objectPoints.resize(_imagePoints.size(), objectPoints[0]);
	double rms = calibrateCamera(objectPoints, _imagePoints, _imageSize, mMatrix, mDistCoeffs, mRotVectors, mTransVectors, CALIB_FIX_K4|CALIB_FIX_K5);

}

