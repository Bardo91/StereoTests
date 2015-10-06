//
//
//
//
//

#include "Camera.h"

#include <opencv2/opencv.hpp>
#include <vector>

class StereoCameras {
public:
	StereoCameras(unsigned _indexCamera1, unsigned _indexCamera2);
	StereoCameras(std::string _pattern1, std::string _pattern2);

	void calibrate(const std::vector<cv::Mat> &_calibrationImages1, const std::vector<cv::Mat> &_calibrationImages2, cv::Size _boardSize, float _squareSize);

	void frames(cv::Mat &_frame1, cv::Mat &_frame2, bool _undistortAndRectificate = false);

	Camera & camera(unsigned _index);

private:
	void calibrateStereo(const std::vector<std::vector<cv::Point2f>> &_imagePoints1, const std::vector<std::vector<cv::Point2f>> &_imagePoints2, cv::Size _imageSize, cv::Size _boardSize, float _squareSize);

private:
	Camera mCamera1, mCamera2;
	cv::Mat mR, mT, mE, mF;

	bool mCalibrated = false;
};