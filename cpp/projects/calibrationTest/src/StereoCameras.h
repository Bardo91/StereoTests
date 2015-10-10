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
	enum eFrameFixing {None, Undistort, UndistortAndRectify};

	/// Create an instance of pairs of cameras using OS indexes.
	/// \params _indexCamera1: index of first camera.
	/// \params _indexCamera2: index of second camera.
	StereoCameras(unsigned _indexCamera1, unsigned _indexCamera2);

	/// Create an instance of face cameras using a pattern for image's paths.
	/// \params _pattern1: Example ---> "/home/user/Desktop/images/img_%d.jpg".
	/// \params _pattern2: Example ---> "/home/user/Desktop/images/img_%d.jpg".
	StereoCameras(std::string _pattern1, std::string _pattern2);

	/// Calibrate pair of cameras and get intrinsic parameters.
	/// \params _calibrationImages1: array of images for calibration.
	/// \params _calibrationImages2: array of images for calibration.
	/// \params _boardSize:	Number of corners in the board.
	/// \params _squareSize: Size of each square in board.
	void calibrate(const std::vector<cv::Mat> &_calibrationImages1, const std::vector<cv::Mat> &_calibrationImages2, cv::Size _boardSize, float _squareSize);

	/// Get pair of frames
	/// \param _frame1: Matrix were first image will be stored.
	/// \param _frame2: Matrix were second image will be stored.
	/// \param _undistortAndRectificate: 
	void frames(cv::Mat &_frame1, cv::Mat &_frame2, eFrameFixing _fixes = eFrameFixing::None);
	
	/// Compute disparity map from given images
	/// \params _frame1: first image.
	/// \params _frame2: second image.
	/// \params _disparityRange: max separation of points.
	/// \params _blockSize: block size.
	cv::Mat disparity(const cv::Mat &_frame1, const cv::Mat &_frame2, unsigned _disparityRange, unsigned _blockSize);

	/// Calculate 3d points from given projections on frames.
	/// \params _points1: projection of points on first image.
	/// \params _points2: projection of points on second image.
	/// \return array of 3d points
	std::vector<cv::Point3f> triangulate(const std::vector<cv::Point2i> &_points1, const std::vector<cv::Point2i> &_points2);

	/// Get one of the cameras
	/// \params _index: 0 for first camera; 1 for the second one.
	/// \return reference to one of the cameras.
	Camera & camera(unsigned _index);

	cv::Mat rotation(unsigned _index);

	cv::Mat traslation(unsigned _index);

	/// Return true if everything is calibrated.
	bool isCalibrated() const;

	/// Save CameraParameters
	/// \params _filePath: path of the file
	void save(std::string _filePath);

	/// Load Camera parameters
	/// \params _filePath: path of the file
	void load(std::string _filePath);

private:
	void calibrateStereo(const std::vector<std::vector<cv::Point2f>> &_imagePoints1, const std::vector<std::vector<cv::Point2f>> &_imagePoints2, cv::Size _imageSize, cv::Size _boardSize, float _squareSize);

private:
	Camera mCamera1, mCamera2;
	cv::Mat mR, mT, mE, mF;

	bool mCalibrated = false;
};
