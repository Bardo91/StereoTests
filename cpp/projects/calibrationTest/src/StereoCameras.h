//
//
//
//
//


#ifndef STEREOCAMERAS_H_
#define STEREOCAMERAS_H_

#include "Camera.h"

#include <opencv2/opencv.hpp>
#include <vector>

class StereoCameras {
public:
	enum eFrameFixing {None, Undistort, UndistortAndRectify};

	/// Create an instance of pairs of cameras using OS indexes.
	/// \param _indexCamera1: index of first camera.
	/// \param _indexCamera2: index of second camera.
	StereoCameras(unsigned _indexCamera1, unsigned _indexCamera2);

	/// Create an instance of face cameras using a pattern for image's paths.
	/// \param _pattern1: Example ---> "/home/user/Desktop/images/img_%d.jpg".
	/// \param _pattern2: Example ---> "/home/user/Desktop/images/img_%d.jpg".
	StereoCameras(std::string _pattern1, std::string _pattern2);

	/// Calibrate pair of cameras and get intrinsic parameters.
	/// \param _calibrationImages1: array of images for calibration.
	/// \param _calibrationImages2: array of images for calibration.
	/// \param _boardSize:	Number of corners in the board.
	/// \param _squareSize: Size of each square in board.
	void calibrate(const std::vector<cv::Mat> &_calibrationImages1, const std::vector<cv::Mat> &_calibrationImages2, cv::Size _boardSize, float _squareSize);

	/// Get pair of frames
	/// \param _frame1: Matrix were first image will be stored.
	/// \param _frame2: Matrix were second image will be stored.
	/// \param _undistortAndRectificate: 
	void frames(cv::Mat &_frame1, cv::Mat &_frame2, eFrameFixing _fixes = eFrameFixing::None);
	
	/// Compute disparity map from given images
	/// \param _frame1: first image.
	/// \param _frame2: second image.
	/// \param _disparityRange: max separation of points.
	/// \param _blockSize: block size.
	cv::Mat disparity(const cv::Mat &_frame1, const cv::Mat &_frame2, unsigned _disparityRange, unsigned _blockSize);


	/// Calculate 3d points from given projections on frames.
	/// \param _frame1: first image of stereo pair
	/// \param _frame2: second image of stereo pair
	/// \return array of 3d points
	std::vector<cv::Point3f> pointCloud(const cv::Mat &_frame1, const cv::Mat &_frame2);

	/// Get one of the cameras.
	/// \param _index: 0 for first camera; 1 for the second one.
	/// \return reference to one of the cameras.
	Camera & camera(unsigned _index);

	/// Give rotation matrix between first and second camera.
	/// \return Rotation matrix between first and second camera.
	cv::Mat rotation() const;

	/// Give translation matrix between first and second camera.
	/// \return Translation matrix between first and second camera.
	cv::Mat translation() const;

	/// Get essential matrix.
	/// \return essential matrix.
	cv::Mat essentialMatrix() const;

	/// Get fundamental matrix.
	/// \return fundamental matrix.
	cv::Mat fundamentalMatrix() const;

	/// Method to check if cameras are calibrated.
	/// \return True if all cameras and stereo parameters are calculated, false otherwise.
	bool isCalibrated() const;

	/// Save Camera parameters.
	/// \param _filePath: path of the file.
	void save(std::string _filePath);

	/// Load Camera parameters.
	/// \param _filePath: path of the file.
	void load(std::string _filePath);

private:
	void calibrateStereo(const std::vector<std::vector<cv::Point2f>> &_imagePoints1, const std::vector<std::vector<cv::Point2f>> &_imagePoints2, cv::Size _imageSize, cv::Size _boardSize, float _squareSize);

	void computeFeatures(const cv::Mat &_frame, std::vector<cv::Point2i> &_features);
	void computeEpipoarLines(const std::vector<cv::Point2i> &_points, std::vector<cv::Vec3f> &_epilines);

	cv::Point2i findMatch(const cv::Mat &_frame1, const cv::Mat &_frame2, const cv::Point2i &_point, const cv::Vec3f &_epiline, const int _maxDisparity, const int _squareSize = 11);

	std::vector<cv::Point3f> triangulate(const std::vector<cv::Point2i> &_points1, const std::vector<cv::Point2i> &_points2);

	std::vector<cv::Point3f> filterPoints(const cv::Mat &_frame1, const cv::Mat &_frame2, const std::vector<cv::Point2i> &_points1, const std::vector<cv::Point2i> &_points2, const std::vector<cv::Point3f> &_points3d, int _maxReprojectionError);
private:
	Camera mCamera1, mCamera2;
	cv::Mat mR, mT, mE, mF;

	bool mCalibrated = false;
};


#endif	//	STEREOCAMERAS_H_
