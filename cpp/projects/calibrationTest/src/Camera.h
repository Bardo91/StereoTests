//
//
//
//
//

#include <opencv2/opencv.hpp>
#include <vector>

class Camera {
public:
	Camera(unsigned _camIndex);

	bool calibrate(std::vector<cv::Mat> _arrayFrames, cv::Size _boardSize, double _squareSize);
	void params(cv::Mat _matrix, cv::Mat _distCoefs, cv::Mat _rotVectors, cv::Mat _transVectors);
	void params(std::string _paramFile);

	cv::Mat matrix();
	cv::Mat distCoeffs();
	cv::Mat rotVectors();
	cv::Mat transVectors();

	cv::Mat frame();

private:
	void calcParams(std::vector<std::vector<cv::Point2f>> _imagePoints, cv::Size _imageSize, cv::Size _boardSize, float _squareSize);

private:
	cv::VideoCapture mDriver;
	cv::Mat mMatrix, mDistCoeffs, mRotVectors, mTransVectors;

};