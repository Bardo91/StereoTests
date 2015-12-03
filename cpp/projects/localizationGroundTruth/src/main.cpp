#include "../../objectsMap/src/vision/StereoCameras.h"

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <fstream>

using namespace std;
using namespace cv;



int main(int _argc, char ** _argv) {

	float squareSize = 0.0223;
	Size cornerCount(15, 10); 
	string leftPath = "C:/Users/GRVC/Desktop/experiment_1356999938/cam1_%d.jpg";
	string rightPath = "C:/Users/GRVC/Desktop/experiment_1356999938/cam2_%d.jpg";
	string savePath = "groundTruthPath";
	StereoCameras cameras(leftPath,rightPath );
	cameras.load("C:/Cprograms/grvc/Code git repo/StereoTests/cpp/build/projects/calibrationApp/calib");
	string folderName = "GroundTruthFiles/";
	std::ofstream file(folderName + "gtPath.txt");


	namedWindow("Image View", 1);

	Mat rvec(3,1,CV_32F,Scalar(0)), tvec(3,1, CV_32F, Scalar(0));


	int i = 0;
	for (;;)
	{
		Mat left, right, leftGray;
		cameras.frames(left, right);
		if (left.cols == 0)
			break;

		cout << i << endl;
		i++;
		vector<Point2f> pointBuf;
		cvtColor(left, leftGray, COLOR_BGR2GRAY);
		bool found = findChessboardCorners(leftGray, cornerCount, pointBuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
		if (!found) {
			file << i << ", NaN, NaN, NaN, NaN, NaN, NaN" << endl;
			continue;
		}


		cornerSubPix(leftGray, pointBuf, Size(3, 3), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));

		vector<Point3f> objectPoints;
		for (int i = 0; i < cornerCount.height; i++) {
			for (int j = 0; j < cornerCount.width; j++) {
				objectPoints.push_back(Point3f(float(j*squareSize), float(i*squareSize), 0));
			}
		}

		solvePnP(objectPoints, pointBuf, cameras.camera(0).matrix(), cameras.camera(0).distCoeffs(), rvec, tvec);
		cout << rvec << " " << tvec << endl;
		file << i << ", " << 
			rvec.at<double>(0, 0) << ", " <<
			rvec.at<double>(1, 0) << ", " <<
			rvec.at<double>(2, 0) << ", " <<
			tvec.at<double>(0, 0) << ", " <<
			tvec.at<double>(1, 0) << ", " <<
			tvec.at<double>(2, 0) << ", " << endl;


		drawChessboardCorners(left, cornerCount, Mat(pointBuf), found);
		//imshow("Image View", left);




	}


	return 1;
}