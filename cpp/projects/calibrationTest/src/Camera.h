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

	bool calibrate(const std::vector<cv::Mat> &_arrayFrames, cv::Size _boardSize, float _squareSize);
	void params(cv::Mat _matrix, cv::Mat _distCoefs, cv::Mat _rotVectors, cv::Mat _transVectors);
	void params(std::string _paramFile);
	void saveParams(std::string _paramFile);

	cv::Mat matrix()		const;
	cv::Mat distCoeffs()	const;
	std::vector<cv::Mat> rotVectors()	const;
	std::vector<cv::Mat> transVectors()	const;

	cv::Mat frame();

private:
	void calcParams(const std::vector<std::vector<cv::Point2f>> &_imagePoints, cv::Size _imageSize, cv::Size _boardSize, float _squareSize);

private:
	cv::VideoCapture mDriver;
	cv::Mat mMatrix, mDistCoeffs;
	std::vector<cv::Mat> mRotVectors, mTransVectors;

};