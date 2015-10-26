//
//
//
//

#include <string>
#include <cassert>

#include "Gui.h"

using namespace cv;
using namespace std;
using namespace pcl;
using namespace pcl::visualization;

//---------------------------------------------------------------------------------------------------------------------
// Static initialization
Gui* Gui::mInstance = nullptr;

//---------------------------------------------------------------------------------------------------------------------
void Gui::init(string _name) {
	assert(mInstance == nullptr);
	mInstance = new Gui(_name);
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
	mMapViewer->addPointCloud<PointXYZ>(_map, "map");	// Not efficient but fast implementation
	mMapViewer->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 1, "map");
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::clearMap() {
	mMapViewer->removeAllPointClouds();
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::addCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr & _cluster, unsigned _pointSize, unsigned _r, unsigned _g, unsigned _b) {
	mMapViewer->addPointCloud<PointXYZRGB>(colorizePointCloud(_cluster, _r, _g, _b));
	mMapViewer->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 1, "Cluster_"+to_string(mPcCounter++));
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::addPointToPcViewer(const PointCloud<PointXYZ>::Ptr & _cloud, unsigned _pointSize, unsigned _r, unsigned _g, unsigned _b) {
	mPcViewer->addPointCloud<PointXYZRGB>(colorizePointCloud(_cloud, _r, _g, _b));
	mPcViewer->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, _pointSize, "Cloud"+to_string(mPcCounter++));
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::clearPcViewer() {
	mPcViewer->removeAllPointClouds();
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::updateStereoImages(const Mat & _left, const Mat & _right) {
	cvtColor(_left, mLeftImage, CV_GRAY2BGR);
	cvtColor(_right, mRightImage, CV_GRAY2BGR);
	hconcat(mLeftImage, mRightImage, mPairStereo);

	imshow(mName + "_StereoViewer", mPairStereo);
}

void Gui::putBlurry(bool _left) {
	Point2i startPoint;
	if(left)
		Point2i(20, 30);
	else
		Point2i(20 + mLeftImage.cols, 30);

	putText(mPairStereo, "Blurry Image", startPoint, FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255), 4);
	imshow(mName + "_StereoViewer", mPairStereo);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::drawPoints(const vector<Point2i>& _points, bool _isLeft, unsigned _r, unsigned _g, unsigned _b) {
	Scalar color = Scalar(_b, _g, _r);
	Point2i offset(_isLeft?0:mLeftImage.cols, 0);

	for (Point2i point : _points) {
		circle(mPairStereo, point+offset, 3, color);
	}
	imshow(mName + "_StereoViewer", mPairStereo);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::drawBoundBoxes(const vector<Rect>& _boxes, bool _isLeft, unsigned _r, unsigned _g, unsigned _b) {
	Scalar color = Scalar(_b, _g, _r);
	int offset = _isLeft?0:mLeftImage.cols;

	for (Rect box : _boxes) {
		box.x += offset;
		rectangle(mPairStereo, box, color);
	}
	imshow(mName + "_StereoViewer", mPairStereo);
}


//---------------------------------------------------------------------------------------------------------------------


//---------------------------------------------------------------------------------------------------------------------
// Private methods
//---------------------------------------------------------------------------------------------------------------------

Gui::Gui(string _name): mName(_name), mMapViewer(new PCLVisualizer (mName+"_MapViewer")), mPcViewer(new PCLVisualizer (mName+"_PcViewer")) {

	// Set up mapViewer
	mMapViewer->setBackgroundColor (0, 0, 0);
	mMapViewer->addCoordinateSystem (1.0);
	mMapViewer->initCameraParameters ();

	// Set up point cloud Viewer
	mPcViewer->setBackgroundColor (0, 0, 0);
	mPcViewer->addCoordinateSystem (1.0);
	mPcViewer->initCameraParameters ();

	// Set up stereo viewer
	namedWindow(mName + "_StereoViewer", CV_WINDOW_NORMAL);
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