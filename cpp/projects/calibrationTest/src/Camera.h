//
//
//
//
//

#include <opencv2/opencv.hpp>
#include <vector>

class Camera {
public:
	/// Create an instance of the camera
	/// \params _camIndex: index of the camera on the current OS.
	Camera(unsigned _camIndex);

	/// Calculate intrinsic parameters of the camera
	/// \params _arrayFrames: Array of frames that will be used for calibration.
	/// \params _boardSize:	Number of corners in the board
	/// \params _squareSize: Size of each square in board
	/// \params _imagePoints: If needed, output variable for using externally the detected matrix of points.
	bool calibrate(const std::vector<cv::Mat> &_arrayFrames, cv::Size _boardSize, float _squareSize, std::vector<std::vector<cv::Point2f>> &_imagePoints = std::vector<std::vector<cv::Point2f>>());

	/// Set intrinsic params of the camera
	/// \params _matrix: 3x3 floating point camera matrix
	/// \params _vector: of distorsion coefficients
	void params(cv::Mat _matrix, cv::Mat _distCoefs);

	/// Load intrinsic params of the camera from a file
	/// \param _paramFile: path to the file that contains parameters (do not include file extension, ex. "params.yml" --> _paramFile = "params";
	void params(std::string _paramFile);

	/// Save current intrinsic params into a file
	/// \param _paramFile: path where the params will be stored.
	void saveParams(std::string _paramFile);

	/// Get current camera matrix
	cv::Mat matrix()		const;

	/// Get current distorsion coefficients
	cv::Mat distCoeffs()	const;

	/// Get rotation vectors estimated for each pattern view.
	std::vector<cv::Mat> rotVectors()	const;

	/// Get translation vectors estimated for each pattern view.
	std::vector<cv::Mat> transVectors()	const;

	/// Get a new frame from the camera
	cv::Mat frame();

private:
	void calcParams(const std::vector<std::vector<cv::Point2f>> &_imagePoints, cv::Size _imageSize, cv::Size _boardSize, float _squareSize);

private:
	cv::VideoCapture mDriver;
	cv::Mat mMatrix, mDistCoeffs;
	std::vector<cv::Mat> mRotVectors, mTransVectors;

};