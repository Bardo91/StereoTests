//
//
//
//
#include "Gui.h"

#include <string>
#include <cassert>
#include <pcl/common/transforms.h>


using namespace cv;
using namespace std;
using namespace pcl;
using namespace pcl::visualization;
using namespace Eigen;
//---------------------------------------------------------------------------------------------------------------------
// Static initialization
Gui* Gui::mInstance = nullptr;

//---------------------------------------------------------------------------------------------------------------------
void Gui::init(string _name, StereoCameras& _stereoCameras) {
	assert(mInstance == nullptr);
	mInstance = new Gui(_name, _stereoCameras);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::end() {
	assert(mInstance != nullptr);
	delete mInstance;
}

//---------------------------------------------------------------------------------------------------------------------
Gui * Gui::get() {
	return mInstance;
}

//---------------------------------------------------------------------------------------------------------------------
// Public interface
//---------------------------------------------------------------------------------------------------------------------
void Gui::drawMap(const PointCloud<PointXYZ>::Ptr & _map) {
	m3dViewer->addPointCloud<PointXYZRGB>(colorizePointCloud(_map, 255,0,0), "map", mViewPortMapViewer);	// Not efficient but fast implementation	
	m3dViewer->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 2, "map",mViewPortMapViewer);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::drawPlane(const pcl::ModelCoefficients &_plane) {
	m3dViewer->addPlane(_plane, "Plane_"+to_string(mPcCounter++),mViewPortMapViewer);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::drawPlane(const pcl::ModelCoefficients & _plane, double _x, double _y, double _z) {
	m3dViewer->addPlane(_plane,_x, _y, _z, "Plane_"+to_string(mPcCounter++),mViewPortMapViewer);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::drawLine(const pcl::PointXYZ & _p1, const pcl::PointXYZ & _p2, unsigned _r, unsigned _g, unsigned _b) {
	m3dViewer->addLine<PointXYZ, PointXYZ>(_p1, _p2, _r, _g, _b,"Line"+to_string(mPcCounter++),mViewPortMapViewer);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::drawCamera(const Eigen::Matrix3f & _orientation, const Eigen::Vector4f & _position) {
	// Create a pointcloud vertically oriented in the origin
	pcl::PointCloud<pcl::PointXYZ> camera;
	camera.push_back(PointXYZ(0.05, 0.05, 0));
	camera.push_back(PointXYZ(-0.05, 0.05, 0));
	camera.push_back(PointXYZ(-0.05, -0.05, 0));
	camera.push_back(PointXYZ(0.05, -0.05, 0));
	camera.push_back(PointXYZ(0.1, 0.1, 0.2));
	camera.push_back(PointXYZ(-0.1, 0.1, 0.2));
	camera.push_back(PointXYZ(-0.1, -0.1, 0.2));
	camera.push_back(PointXYZ(0.1, -0.1, 0.2));

	// Rotate and move camera to the desired position and orientation.
	Matrix4f transformation = Matrix4f::Zero();
	transformation << _orientation;
	transformation.col(3) << _position(0), _position(1), _position(2), 1;

	//std::cout << transformation << std::endl;

	pcl::PointCloud<pcl::PointXYZ> cameraRotated;
	transformPointCloud(camera, cameraRotated, transformation);

	// Draw camera
	PointXYZ p1 = cameraRotated[3];
	for (unsigned i = 0; i < 4; i++) {
		PointXYZ p2 = cameraRotated[i];
		drawLine(p1, p2);
		p1 = p2;
	}

	p1 = cameraRotated[4];
	for (unsigned i = 4; i < 8; i++) {
		PointXYZ p2 = cameraRotated[i];
		drawLine(p1, p2);
		p1 = p2;
	}

	for (unsigned i = 0; i < 4; i++) {
		PointXYZ p1 = cameraRotated[i];
		PointXYZ p2 = cameraRotated[i+4];
		drawLine(p1, p2);
	}


}

//---------------------------------------------------------------------------------------------------------------------
void Gui::clearMap() {
	m3dViewer->removeAllPointClouds(mViewPortMapViewer);
	m3dViewer->removeAllShapes(mViewPortMapViewer);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::drawCandidate(const ObjectCandidate & _candidate) {
	addCluster(_candidate.cloud(), 4, _candidate.R(), _candidate.G(), _candidate.B());
	reprojectCloud(_candidate.cloud(), _candidate.R(), _candidate.G(), _candidate.B());
	drawCathegory(_candidate);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::addCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr & _cluster, unsigned _pointSize, unsigned _r, unsigned _g, unsigned _b) {
	mPcCounter++;
	m3dViewer->addPointCloud<PointXYZRGB>(colorizePointCloud(_cluster, _r, _g, _b),"Cluster_"+to_string(mPcCounter),mViewPortMapViewer);
	m3dViewer->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, _pointSize, "Cluster_"+to_string(mPcCounter),mViewPortMapViewer);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::addPointToPcViewer(const PointCloud<PointXYZ>::Ptr & _cloud, unsigned _pointSize, unsigned _r, unsigned _g, unsigned _b) {
	mPcCounter++;
	m3dViewer->addPointCloud<PointXYZRGB>(colorizePointCloud(_cloud, _r, _g, _b),"Cloud"+to_string(mPcCounter),mViewportPcViewer);
	m3dViewer->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, _pointSize, "Cloud"+to_string(mPcCounter),mViewportPcViewer);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::clearPcViewer() {
	m3dViewer->removeAllPointClouds();
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::updateStereoImages(const Mat & _left, const Mat & _right) {
	mLeftImage = _left; 
	mRightImage = _right;
	hconcat(mLeftImage, mRightImage, mPairStereo);

	imshow(mName + "_StereoViewer", mPairStereo);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::putBlurry(bool _left) {
	Point2i startPoint;
	if(_left)
		startPoint = Point2i(20, 30);
	else
		startPoint = Point2i(20 + mLeftImage.cols, 30);

	putText(mPairStereo, "Blurry Image", startPoint, FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255),4);
	imshow(mName + "_StereoViewer", mPairStereo);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::drawPoints(const vector<Point2f>& _points, bool _isLeft, unsigned _r, unsigned _g, unsigned _b) {
	Scalar color = Scalar(_b, _g, _r);
	Point2i offset(_isLeft?0:mLeftImage.cols, 0);
	
	Rect validRegion(_isLeft? 0:mLeftImage.cols, 0, mLeftImage.cols, mLeftImage.rows);
	//cout << validRegion.x << ", " << validRegion.y << ", " << validRegion.width << ", " << validRegion.height << std::endl;
	for (Point2i point : _points) {
		if(!validRegion.contains(point+offset))
			continue;

		circle(mPairStereo, point+offset, 3, color);
	}
	imshow(mName + "_StereoViewer", mPairStereo);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::drawBoundBoxes(const vector<Rect>& _boxes, bool _isLeft, unsigned _r, unsigned _g, unsigned _b) {
	Scalar color = Scalar(_b, _g, _r);
	int offset = _isLeft?0:mLeftImage.cols;

	Rect validFrame(0,offset,mLeftImage.cols, mLeftImage.rows);

	for (Rect box : _boxes) {
		box.x += offset;
		rectangle(mPairStereo, box&validFrame, color);
	}
	imshow(mName + "_StereoViewer", mPairStereo);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::drawBox(const cv::Rect & _box, bool _isLeft, unsigned _r, unsigned _g, unsigned _b) {
	Scalar color = Scalar(_b, _g, _r);

	int offset = _isLeft?0:mLeftImage.cols;
	Rect validFrame(offset,0,mLeftImage.cols, mLeftImage.rows);
	Rect box = _box;
	box.x += offset;
	rectangle(mPairStereo, box&validFrame, color);

	imshow(mName + "_StereoViewer", mPairStereo);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::drawPolygon(const std::vector<cv::Point2f>& _polygon, bool _isLeft, unsigned _r, unsigned _g, unsigned _b) {
	Scalar color = Scalar(_b, _g, _r);
	Point2f offset(_isLeft?0:mLeftImage.cols, 0);	

	Rect validRegion(_isLeft? 0:mLeftImage.cols, 0, mLeftImage.cols, mLeftImage.rows);

	std::function<void (Point2f _p1, Point2f _p2, unsigned _iter)> checkAndDraw = [&](Point2f _p1, Point2f _p2, unsigned _iter) {
		if (_iter > 10) {
			line(mPairStereo, _p1, _p2, color, 3);
			return;
		}

		if (validRegion.contains(_p1)) {
			if (validRegion.contains(_p2)) {
				line(mPairStereo, _p1, _p2, color,3);
			}
			else {
				_p2  = _p1 + (_p2 - _p1)/2;
				checkAndDraw(_p1,_p2,_iter++);
			}
		} else {
			if (validRegion.contains(_p2)) {
				_p1  = _p2 + (_p1 - _p2)/2;
				checkAndDraw(_p1,_p2,_iter++);
			}
		}		
	};

	// Draw all lines except between last and initial point.
	for (unsigned i = 0; i < _polygon.size() - 1;i++) {
		Point2f p1 = _polygon[i] + offset;
		Point2f p2 = _polygon[i+1] + offset;
		checkAndDraw(p1, p2, 0);
	}
	
	// Draw last line
	Point2f p1 = _polygon[0] + offset;
	Point2f p2 = _polygon[_polygon.size()-1] + offset;
	checkAndDraw(p1, p2, 0);

	imshow(mName + "_StereoViewer", mPairStereo);
}

//---------------------------------------------------------------------------------------------------------------------
Rect boundRect(vector<Point2f> _points2d) {
	int minX=99999, minY=999999, maxX=0, maxY=0;
	for (Point2f point : _points2d) {
		if(point.x < minX)
			minX = point.x;
		if(point.y < minY)
			minY = point.y;
		if(point.x > maxX)
			maxX = point.x;
		if(point.y > maxY)
			maxY = point.y;
	}

	return Rect(minX, minY, maxX-minX, maxY-minY);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::reprojectCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud, unsigned _r, unsigned _g, unsigned _b)
{
	vector<Point3f> points3d;
	for (const PointXYZ point : *_cloud)
		points3d.push_back(Point3f(point.x, point.y, point.z));
	vector<Point2f> reprojection1 = mStereoCameras.project3dPointsWCS(points3d, true);
	vector<Point2f> reprojection2 = mStereoCameras.project3dPointsWCS(points3d, false);
	drawPoints(reprojection1, true, _r, _g, _b);
	drawPoints(reprojection2, false, _r, _g, _b);
	// Calculate convexHull
	std::vector<Point2f> convexHull1, convexHull2;
	convexHull(reprojection1, convexHull1);
	convexHull(reprojection2, convexHull2);

	drawPolygon(convexHull1, true, _r, _g, _b);
	drawPolygon(convexHull2, false, _r, _g, _b);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::drawCathegory(const ObjectCandidate & _candidate) {
	vector<Point3f> points3d;
	for (const PointXYZ point : *_candidate.cloud())
		points3d.push_back(Point3f(point.x, point.y, point.z));

	vector<Point2f> reprojection1 = mStereoCameras.project3dPointsWCS(points3d, true);
	vector<Point2f> reprojection2 = mStereoCameras.project3dPointsWCS(points3d, false);

	Rect validFrame(0,0,mLeftImage.cols, mLeftImage.rows);
	Rect bb1 = boundRect(reprojection1)&validFrame;
	Rect bb2 = boundRect(reprojection2)&validFrame;
	drawBox(bb1, true, _candidate.R() ,_candidate.G(), _candidate.B());
	drawBox(bb2, false,  _candidate.R() ,_candidate.G(), _candidate.B());
	
	
	Point2i startPointLeft = Point2i(bb1.x, bb1.y);
	Point2i startPointRight = Point2i(bb2.x + mLeftImage.cols, bb2.y);

	std::pair<int, double> cathegory = _candidate.cathegory();

	string text = to_string(cathegory.first) + ": " + to_string(cathegory.second);
	putText(mPairStereo, text, startPointLeft, FONT_HERSHEY_PLAIN, 1, Scalar(_candidate.B() ,_candidate.G(), _candidate.R()),1);
	putText(mPairStereo, text, startPointRight, FONT_HERSHEY_PLAIN, 1, Scalar(_candidate.B() ,_candidate.G(), _candidate.R()),1);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::spinOnce() {
	m3dViewer->spinOnce();
}

//---------------------------------------------------------------------------------------------------------------------
Gui::Gui(string _name, StereoCameras& _stereoCameras): mName(_name), m3dViewer(new PCLVisualizer (mName)), mStereoCameras(_stereoCameras) {
	m3dViewer->initCameraParameters ();
	m3dViewer->addCoordinateSystem (0.25);
	// Set up mapViewer
	m3dViewer->createViewPort (0.0, 0.0, 0.5, 1.0, mViewPortMapViewer);
	m3dViewer->setBackgroundColor (0, 0, 0, mViewPortMapViewer);
	m3dViewer->addCoordinateSystem (0.25,"XYZ_map", mViewPortMapViewer);
	PointCloud<PointXYZ>::Ptr emptyCloud(new PointCloud<PointXYZ>);
	m3dViewer->addPointCloud(emptyCloud, "map",mViewPortMapViewer);
	m3dViewer->addText ("Map Viewer", 10, 10, "MapViewer text", mViewPortMapViewer);
	// Set up pc viewer
	m3dViewer->createViewPort (0.5, 0.0, 1.0, 1.0, mViewportPcViewer);
	m3dViewer->setBackgroundColor (0.1, 0.1, 0.1, mViewportPcViewer);
	m3dViewer->addCoordinateSystem (0.25,"XYZ_pc",mViewportPcViewer);
	m3dViewer->addText ("Cloud Viewer", 10, 10, "PcViewer text", mViewportPcViewer);

	// Set up stereo viewer
	namedWindow(mName + "_StereoViewer", CV_WINDOW_AUTOSIZE);
}
//---------------------------------------------------------------------------------------------------------------------
PointCloud<PointXYZRGB>::Ptr Gui::colorizePointCloud(const PointCloud<PointXYZ>::Ptr & _cloud, int _r, int _g, int _b) {
	PointCloud<PointXYZRGB>::Ptr colorizedCloud(new PointCloud<PointXYZRGB>);
	for (PointXYZ point : *_cloud) {
		PointXYZRGB p;
		p.x = point.x;
		p.y = point.y;
		p.z = point.z;
		p.r = _r;
		p.g = _g;
		p.b = _b;
		colorizedCloud->push_back(p);
	}
	return colorizedCloud;
}

//---------------------------------------------------------------------------------------------------------------------