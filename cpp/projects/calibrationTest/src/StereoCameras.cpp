//
//
//
//
//

#include "StereoCameras.h"

using namespace cv;
using namespace std;

//---------------------------------------------------------------------------------------------------------------------
StereoCameras::StereoCameras(unsigned _indexCamera1, unsigned _indexCamera2): mCamera1(_indexCamera1), mCamera2(_indexCamera2) {

}

StereoCameras::StereoCameras(string _pattern1, string _pattern2): mCamera1(_pattern1), mCamera2(_pattern2) {

}

//---------------------------------------------------------------------------------------------------------------------
void StereoCameras::calibrate(const vector<Mat> &_calibrationImages1, const vector<Mat> &_calibrationImages2, Size _boardSize, float _squareSize) {
	vector<vector<Point2f>> imagePoints1, imagePoints2;

	mCamera1.calibrate(_calibrationImages1, _boardSize, _squareSize, imagePoints1);
	mCamera2.calibrate(_calibrationImages2, _boardSize, _squareSize, imagePoints2);

	calibrateStereo(imagePoints1, imagePoints2, _calibrationImages1[0].size(), _boardSize, _squareSize);
	mCalibrated = true;
}

//---------------------------------------------------------------------------------------------------------------------
void StereoCameras::frames(Mat & _frame1, Mat & _frame2,  eFrameFixing _fixes) {
	switch(_fixes){
	case eFrameFixing::None:
		_frame1 = mCamera1.frame();
		_frame2 = mCamera2.frame();
		break;
	case eFrameFixing::Undistort:
		_frame1 = mCamera1.frame(true);
		_frame2 = mCamera2.frame(true);
		break;
	case eFrameFixing::UndistortAndRectify:{
		_frame1 = mCamera1.frame(true);
		_frame2 = mCamera2.frame(true);
		Size imageSize = _frame1.size();
		Mat R1, R2, P1, P2, Q;
		Rect validRoi[2];
		stereoRectify(mCamera1.matrix(), mCamera1.distCoeffs(), mCamera2.matrix(), mCamera2.distCoeffs(), imageSize, mR, mT, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, imageSize, &validRoi[0], &validRoi[1]);

		Mat rmap[2][2];

		// Calculate remap functions
		initUndistortRectifyMap(mCamera1.matrix(), mCamera1.distCoeffs(), R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
		initUndistortRectifyMap(mCamera2.matrix(), mCamera2.distCoeffs(), R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

		// Undirstort and rectify images
		Mat rectifiedFrame1, rectifiedFrame2;
		cout << rmap[0][0] << endl;
		cout << rmap[0][1] << endl;
		cout << rmap[1][0] << endl;
		cout << rmap[1][1] << endl;

		remap(_frame1, _frame1, rmap[0][0], rmap[0][1], INTER_LINEAR);
		remap(_frame2, _frame2, rmap[1][0], rmap[1][1], INTER_LINEAR);
		break;
		}
	}
}

//---------------------------------------------------------------------------------------------------------------------
Mat StereoCameras::disparity(const Mat & _frame1, const Mat & _frame2, unsigned _disparityRange, unsigned _blockSize) {
	Ptr<StereoSGBM> disparityMaker = StereoSGBM::create(0, _disparityRange, _blockSize);

	Mat disparity;
	disparityMaker->compute(_frame1, _frame2, disparity);

	double minVal,maxVal;

	minMaxLoc( disparity, &minVal, &maxVal );
	disparity.convertTo(disparity, CV_8UC1, 255/(maxVal - minVal));

	return disparity;
}

//---------------------------------------------------------------------------------------------------------------------
vector<Point3f> StereoCameras::pointCloud(const cv::Mat &_frame1, const cv::Mat &_frame2) {
	// Compute keypoint only in first image
	vector<Point2i> keypoints;
	computeFeatures(_frame1, keypoints);

	std::cout << "Features computed in frame1: " << keypoints.size() << std::endl;

	// Compute projection of epipolar lines into second image.
	std::vector<cv::Vec3f> epilines;
	computeEpipoarLines(keypoints, epilines);

	// For each epipolar line calculate equivalent feature by template matching.
	Rect validRegion(30, 30, _frame2.cols -30*2, _frame2.rows - 30*2);	// 666 maybe... use ROIs computed in calibration?
	vector<Point2i> points1, points2;
	for (unsigned i = 0; i < epilines.size(); i++){
		if(!validRegion.contains(keypoints[i]))	// Ignore keypoint if it is outside valid region.
			continue;

		// Calculate matching and add points
		Point2i matchedPoint = findMatch(_frame1, _frame2, keypoints[i], epilines[i], 640);
		if(!validRegion.contains(matchedPoint))
			continue;

		points1.push_back(keypoints[i]);
		points2.push_back(matchedPoint);
	}
	std::cout << "Features matched: " << points1.size() << std::endl;
	// Triangulate points using features in both images.
	vector<Point3f> points3d = triangulate(points1, points2);
	std::cout << "Points triangulated: " << points3d.size() << std::endl;
	// Filter points using reprojection.
	vector<Point3f> points3dFiltered = filterPoints(_frame1, _frame2, points1, points2, points3d, 3);
	std::cout << "Points Filtered: " << points3dFiltered.size() << std::endl;

	return points3dFiltered;
}

//---------------------------------------------------------------------------------------------------------------------
Camera & StereoCameras::camera(unsigned _index) {
	assert(_index < 2);
	if(_index == 0)
		return mCamera1;
	else {
		return mCamera2;
	}
}

//---------------------------------------------------------------------------------------------------------------------
Mat StereoCameras::rotation() const{
	return mR;
}

//---------------------------------------------------------------------------------------------------------------------
Mat StereoCameras::translation() const{
	return mT;
}

//---------------------------------------------------------------------------------------------------------------------
Mat StereoCameras::essentialMatrix() const{
	return mE;
}

//---------------------------------------------------------------------------------------------------------------------
Mat StereoCameras::fundamentalMatrix() const{
	return mF;
}


//---------------------------------------------------------------------------------------------------------------------
bool StereoCameras::isCalibrated() const {
	return mCalibrated;
}

void StereoCameras::save(string _filePath) {
	mCamera1.saveParams(_filePath + "_cam1");
	mCamera2.saveParams(_filePath + "_cam2");
	
	FileStorage fs(_filePath + "_stero.yml", FileStorage::WRITE);
	fs << "mR" << mR;
	fs << "mT" << mT;
	fs << "mE" << mE;
	fs << "mF" << mF;
}

void StereoCameras::load(string _filePath) {
	mCamera1.params(_filePath + "_cam1");
	mCamera2.params(_filePath + "_cam2");

	FileStorage fs(_filePath + "_stero.yml", FileStorage::READ);
	fs["mR"] >> mR;
	fs["mT"] >> mT;
	fs["mE"] >> mE;
	fs["mF"] >> mF;

	mCalibrated = true;
}

//---------------------------------------------------------------------------------------------------------------------
//		Private methods
//---------------------------------------------------------------------------------------------------------------------
void StereoCameras::calibrateStereo(const vector<vector<Point2f>> &_imagePoints1, const vector<vector<Point2f>> &_imagePoints2, Size _imageSize, Size _boardSize, float _squareSize) {
	vector<vector<Point2f>> filteredPoints1 = _imagePoints1;
	vector<vector<Point2f>> filteredPoints2 = _imagePoints2;

	for (unsigned i = 0; i < filteredPoints1.size(); i++) {
		if (filteredPoints1[i].size() !=   _boardSize.width*_boardSize.height || filteredPoints2[i].size() !=   _boardSize.width*_boardSize.height) {
			filteredPoints1.erase(filteredPoints1.begin() + i);
			filteredPoints2.erase(filteredPoints2.begin() + i);
			i--;
		}
	}
	
	
	vector<vector<Point3f> > objectPoints(1);
	for (int i = 0; i < _boardSize.height; i++) {
		for (int j = 0; j < _boardSize.width; j++) {
			objectPoints[0].push_back(Point3f(float(j*_squareSize), float(i*_squareSize), 0));
		}
	}
	objectPoints.resize(filteredPoints1.size(),objectPoints[0]);

	stereoCalibrate(objectPoints, filteredPoints1, filteredPoints2, mCamera1.matrix(), mCamera1.distCoeffs(), mCamera2.matrix(), mCamera2.distCoeffs(),_imageSize, mR, mT, mE, mF);
}

//---------------------------------------------------------------------------------------------------------------------
void StereoCameras::computeFeatures(const Mat &_frame, vector<Point2i> &_features){
	goodFeaturesToTrack(_frame, _features, 5000, 0.001,0.5);
}

//---------------------------------------------------------------------------------------------------------------------
void StereoCameras::computeEpipoarLines(const vector<Point2i> &_points, vector<Vec3f> &_epilines){
	computeCorrespondEpilines(_points, 1, fundamentalMatrix(), _epilines);
}

//---------------------------------------------------------------------------------------------------------------------
cv::Point2i StereoCameras::findMatch(const Mat &_frame1, const Mat &_frame2, const Point2i &_point, const Vec3f &_epiline, const int _maxDisparity, const int _squareSize){
	// Compute template matching over the epipolar line

	// Get template from first image.
	Mat imgTemplate = _frame1(Rect(	Point2i(_point.x - _squareSize/2, _point.y - _squareSize/2),
									Point2i(_point.x + _squareSize/2, _point.y + _squareSize/2)));
	uchar * imgTemplateData = imgTemplate.data;

	double minVal = _squareSize*_squareSize*255*255;
	Point2i maxLoc, p1, sp1, sp2;
	Mat subImage;

	int minX = _point.x - _maxDisparity;
	minX = minX < 50 ? 50 : minX;
	int maxX = _point.x > (_frame2.cols - 50) ? (_frame2.cols -50 -1 ):_point.x;
	std::vector<double> corrValues;

	/*Mat dis1, dis2, dis22;
	cvtColor(_frame1, dis1, CV_GRAY2BGR);
	cvtColor(_frame2, dis2, CV_GRAY2BGR);
	Point2i p1e(0, -_epiline[2] / _epiline[1]);
	Point2i p2e(_frame2.cols,-(_epiline[2] + _epiline[0] * _frame2.cols) / _epiline[1]);
	line(dis2, p1e, p2e, Scalar(0,0,255));
	circle(dis1, _point, 3, Scalar(0,0,255));
	rectangle(dis1, Rect(	Point2i(_point.x - _squareSize/2, _point.y - _squareSize/2), Point2i(_point.x + _squareSize/2, _point.y + _squareSize/2)), Scalar(255,0,0));
	imshow("Frame1", dis1);
	imshow("Frame2", dis2);
	waitKey();

	namedWindow("template", CV_WINDOW_FREERATIO);
	namedWindow("subImage", CV_WINDOW_FREERATIO);
	namedWindow("difference", CV_WINDOW_FREERATIO);*/
	for(unsigned i = minX ; i < maxX ;i++){
		// Compute point over epiline
		p1.x = i;
		p1.y = -1*(_epiline[2] + _epiline[0] * i)/_epiline[1];
		sp1 = p1 - Point2i(_squareSize/2, _squareSize/2);
		sp2 = p1 + Point2i(_squareSize/2, _squareSize/2);

		/*dis2.copyTo(dis22);
		circle(dis22, p1, 4, Scalar(0,255,0));
		rectangle(dis22, sp1, sp2, Scalar(0,255,0));
		imshow("dis22",dis22);*/

		// Get subimage from image 2;
		subImage = _frame2(Rect(sp1, sp2));
		uchar * subImageData = subImage.data;
		// Compute correlation
		/*imshow("subimage2", subImage);*/

		Mat dif = imgTemplate.clone();
		double val = 0;
		for(unsigned row = 0; row < subImage.rows; row++){
			for(unsigned col = 0; col < subImage.cols; col++){
				double aux = abs(double(subImage.at<uchar>(row, col)) - double(imgTemplate.at<uchar>(row, col)));
				val += aux;
				dif.at<uchar>(row, col) = aux > 255 ? 255: aux;
			}
		}

		corrValues.push_back(val);
		/*std::cout << "Current Max: " << minVal << ". Current Val: " << val <<std::endl;*/

		/*imshow("difference", dif);
		imshow("template", imgTemplate);
		imshow("subImage", subImage);*/

		if(val < minVal){
			minVal = val;
			maxLoc = p1;
			/*std::cout << "New minimum" << std::endl;*/
		}

		/*waitKey();*/
	}



	/*Mat matchDis;
	hconcat(_frame1, _frame2, matchDis);
	cvtColor(matchDis,matchDis, CV_GRAY2BGR);
	circle(matchDis, _point, 3, Scalar(0,255,0));
	circle(matchDis, maxLoc + Point2i(_frame2.cols,0), 3, Scalar(0,255,0));
	//line(matchDis, _point, maxLoc+Point2i(_frame2.cols,0), Scalar(0,255,0));

	Mat plot = Mat::zeros(300,corrValues.size(), CV_8UC1);
	double maxVal = *std::max_element(corrValues.begin(), corrValues.end());
	for(unsigned i = 0; i < corrValues.size(); i++){
		plot.at<uchar>(unsigned(300*corrValues[i]/(maxVal)),i) = 255;
	}

	cvtColor(plot,plot,CV_GRAY2BGR);
	line(plot, Point(0,0), Point(0,50), Scalar(0,0,255));
	line(plot, Point(0,0), Point(50,0), Scalar(0,255,0));
	imshow("plot", plot);


	Point2i ep1(minX, -1*(_epiline[2] + _epiline[0] * minX)/_epiline[1]);
	Point2i ep2(maxX, -1*(_epiline[2] + _epiline[0] * maxX)/_epiline[1]);
	line(matchDis, ep1+ Point2i(_frame2.cols,0), ep2+ Point2i(_frame2.cols,0), Scalar(0,0,255));
	imshow("matches", matchDis);
	waitKey();*/

	return maxLoc;
}

//---------------------------------------------------------------------------------------------------------------------
vector<Point3f> StereoCameras::triangulate(const vector<Point2i> &_points1, const vector<Point2i> &_points2) {
	Mat pnts3D	(4,_points1.size(),CV_64F);
	Mat cam1pnts(2,_points1.size(),CV_64F);
	Mat cam2pnts(2,_points1.size(),CV_64F);

	for (unsigned i = 0; i < _points1.size(); i++) {
		cam1pnts.at<double>(0,i) = _points1[i].x;
		cam1pnts.at<double>(1,i) = _points1[i].y;
		cam2pnts.at<double>(0,i) = _points2[i].x;
		cam2pnts.at<double>(1,i) = _points2[i].y;
	}

	Mat I = Mat::eye(3,4, CV_64F);
	Mat extrinsicMatrix(3,4, CV_64F);
	mR.copyTo(extrinsicMatrix.rowRange(0,3).colRange(0,3));
	mT.copyTo(extrinsicMatrix.rowRange(0,3).col(3));

	triangulatePoints(mCamera1.matrix()*I, mCamera2.matrix()*extrinsicMatrix, cam1pnts, cam2pnts, pnts3D);

	vector<Point3f> points3d;
	for (unsigned i = 0 ; i < pnts3D.cols ; i++) {
		float w = (float) pnts3D.at<double>(3,i);
		float x = (float) pnts3D.at<double>(0,i)/w;
		float y = (float) pnts3D.at<double>(1,i)/w;
		float z = (float) pnts3D.at<double>(2,i)/w;
		points3d.push_back(Point3f(x,y,z));
	}

	return points3d;
}

//---------------------------------------------------------------------------------------------------------------------
vector<Point3f> StereoCameras::filterPoints(const Mat &_frame1, const Mat &_frame2, const vector<Point2i> &_points1, const vector<Point2i> &_points2, const vector<Point3f> &_points3d, int _maxReprojectionError){
	vector<Point2f> reprojection1, reprojection2;
	projectPoints(_points3d, Mat::eye(3, 3, CV_64F), Mat::zeros(3, 1, CV_64F), mCamera1.matrix(), mCamera1.distCoeffs(), reprojection1);
	projectPoints(_points3d, rotation(), translation(), mCamera2.matrix(), mCamera2.distCoeffs(), reprojection2);

	vector<Point3f> filteredPoints3d;
	for (unsigned i = 0; i < _points3d.size(); i++) {
		double rError1 = sqrt(pow(reprojection1[i].x - _points1[i].x, 2)
							+ pow(reprojection1[i].y - _points1[i].y, 2));
		double rError2 = sqrt(pow(reprojection2[i].x - _points2[i].x, 2)
							+ pow(reprojection2[i].y - _points2[i].y, 2));

		if (rError1 < _maxReprojectionError && rError2 < _maxReprojectionError) {
			filteredPoints3d.push_back(_points3d[i]);
		}

		/*Mat matchDis;
		hconcat(_frame1, _frame2, matchDis);
		cvtColor(matchDis,matchDis, CV_GRAY2BGR);
		circle(matchDis, _points1[i], 3, Scalar(0,255,0));
		circle(matchDis, _points2[i]+ Point2i(_frame2.cols,0), 3, Scalar(0,255,0));
		line(matchDis, _points1[i], _points2[i]+Point2i(_frame2.cols,0), Scalar(0,255,0));

		circle(matchDis, reprojection1[i], 3, Scalar(0,0,255));
		circle(matchDis, Point2i(reprojection2[i].x, reprojection2[i].y) + Point2i(_frame2.cols,0), 3, Scalar(0,0,255));

		imshow("matches", matchDis);
		waitKey();*/

	}

	return filteredPoints3d;
}

//---------------------------------------------------------------------------------------------------------------------
