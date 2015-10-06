///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-10-01
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int _argc, char** _argv){
	Mat view = imread("C:/Users/Pablo RS/ownCloud/Datasets/StereoTesting/CalibrationImages/cam1/img_cam1_0.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	Mat pointBuf;

	bool found = findChessboardCorners( view, Size(8,6), pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
	
	if (found) {
		std::cout << "great" << std::endl;
		Mat viewGray;
		view.copyTo(viewGray);
		cornerSubPix( viewGray, pointBuf, Size(11,11), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
		drawChessboardCorners( view, Size(8,6), Mat(pointBuf), found );
		imshow("points", view);
		waitKey(1);
	}
	else {
		std::cout << "fail!" << std::endl;
	}

	system("PAUSE");

}
