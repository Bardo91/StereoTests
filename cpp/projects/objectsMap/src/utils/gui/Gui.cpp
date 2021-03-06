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
void Gui::drawPlane(const pcl::ModelCoefficients &_plane, std::string _tag) {
	if (_tag == "") {
		_tag =  "Plane_"+to_string(mPcCounter++);
	}else {
		if (!isTagAllowed(_tag)) {
			return;
		}
	}
	m3dViewer->addPlane(_plane,_tag,mViewPortMapViewer);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::drawPlane(const pcl::ModelCoefficients & _plane, double _x, double _y, double _z, std::string _tag) {
	if (_tag == "") {
		_tag =  "Plane_"+to_string(mPcCounter++);
	}else {
		if (!isTagAllowed(_tag)) {
			return;
		}
	}
	m3dViewer->addPlane(_plane,_x, _y, _z,_tag ,mViewPortMapViewer);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::drawLine(const pcl::PointXYZ & _p1, const pcl::PointXYZ & _p2, unsigned _r, unsigned _g, unsigned _b, std::string _tag) {
	if (_tag == "") {
		_tag =  "Line" + to_string(mPcCounter++);
	}else {
		if (!isTagAllowed(_tag)) {
			return;
		}
	}
	m3dViewer->addLine<PointXYZ, PointXYZ>(_p1, _p2, _r, _g, _b,_tag,mViewPortMapViewer);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::drawCamera(const Eigen::Matrix3f & _orientation, const Eigen::Vector4f & _position, unsigned _r , unsigned _g , unsigned _b, std::string _tag) {
	// Create a pointcloud vertically oriented in the origin
	pcl::PointCloud<pcl::PointXYZ> camera;
	camera.push_back(PointXYZ(0.07, 0.05, 0));
	camera.push_back(PointXYZ(-0.07, 0.05, 0));
	camera.push_back(PointXYZ(-0.07, -0.05, 0));
	camera.push_back(PointXYZ(0.07, -0.05, 0));
	camera.push_back(PointXYZ(0.12, 0.1, 0.2));
	camera.push_back(PointXYZ(-0.12, 0.1, 0.2));
	camera.push_back(PointXYZ(-0.12, -0.1, 0.2));
	camera.push_back(PointXYZ(0.12, -0.1, 0.2));

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
		drawLine(p1,p2, _r, _g, _b);
		p1 = p2;
	}

	p1 = cameraRotated[4];
	for (unsigned i = 4; i < 8; i++) {
		PointXYZ p2 = cameraRotated[i];
		drawLine(p1, p2, _r, _g, _b);
		p1 = p2;
	}

	for (unsigned i = 0; i < 4; i++) {
		PointXYZ p1 = cameraRotated[i];
		PointXYZ p2 = cameraRotated[i+4];
		drawLine(p1, p2, _r, _g, _b);
	}

	// Marker on top
	ModelCoefficients coef;
	coef.values.push_back((cameraRotated[0].x + cameraRotated[1].x)/2);
	coef.values.push_back((cameraRotated[0].y + cameraRotated[1].y)/2);
	coef.values.push_back((cameraRotated[0].z + cameraRotated[1].z)/2);
	coef.values.push_back(0.01);
	m3dViewer->addSphere(coef,"cameraTop"+to_string(mPcCounter++), mViewPortMapViewer);

}

//---------------------------------------------------------------------------------------------------------------------
void Gui::clearMap() {
	m3dViewer->removeAllPointClouds(mViewPortMapViewer);
	m3dViewer->removeAllShapes(mViewPortMapViewer);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::drawCandidate(const ObjectCandidate & _candidate, const Eigen::Vector4f &_position, const Eigen::Quaternionf &_orientation) {
	addCloudToMapViewer(_candidate.cloud(), 4, _candidate.R(), _candidate.G(), _candidate.B(), "candidate_"+to_string(mPcCounter++));
	reprojectCloud(_candidate.cloud(),_position, _orientation, _candidate.R(), _candidate.G(), _candidate.B());
	drawCathegory(_candidate,_position, _orientation);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::addCloudToMapViewer(const pcl::PointCloud<pcl::PointXYZ>::Ptr & _cluster, unsigned _pointSize, unsigned _r, unsigned _g, unsigned _b, std::string _tag) {
	if (_tag == "") {
		mPcCounter++;
		_tag = "Cluster_"+to_string(mPcCounter);
	}else {
		if (!isTagAllowed(_tag)) {
			return;
		}
	}
	m3dViewer->addPointCloud<PointXYZRGB>(colorizePointCloud(_cluster, _r, _g, _b),_tag,mViewPortMapViewer);
	m3dViewer->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, _pointSize, _tag,mViewPortMapViewer);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::addCloudToPcViewer(const PointCloud<PointXYZ>::Ptr & _cloud, unsigned _pointSize, unsigned _r, unsigned _g, unsigned _b, std::string _tag) {
	if (_tag == "") {
		mPcCounter++;
		_tag = "Cloud_"+to_string(mPcCounter);
	} else {
		if (!isTagAllowed(_tag)) {
			return;
		}
	}
	m3dViewer->addPointCloud<PointXYZRGB>(colorizePointCloud(_cloud, _r, _g, _b),_tag, mViewportPcViewer);
	m3dViewer->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, _pointSize, _tag, mViewportPcViewer);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::clearPcViewer() {
	m3dViewer->removeAllPointClouds();
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::updateStereoImages(const Mat & _left, const Mat & _right) {
	mLinesOfText = 0;
	mLeftImage = _left; 
	mRightImage = _right;
	hconcat(mLeftImage, mRightImage, mPairStereo);
	imshow(mName + "_StereoViewer", mPairStereo);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::putBlurry(bool _left) {
	Point2i startPoint;
	if(_left)
		startPoint = Point2i(mLeftImage.cols/2-50, mLeftImage.rows/2);
	else
		startPoint = Point2i(mLeftImage.cols/2 + mLeftImage.cols-50, mLeftImage.rows/2);

	putText(mPairStereo, "Blurry Image", startPoint, FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 0, 255),4);
	imshow(mName + "_StereoViewer", mPairStereo);
}

//---------------------------------------------------------------------------------------------------------------------
void Gui::addText(string _text, unsigned char _r, unsigned char _g, unsigned char _b) {
	Point2i startPoint(20,30+mLinesOfText++*30);
	putText(mPairStereo, _text, startPoint, FONT_HERSHEY_SIMPLEX, 0.75, Scalar(_b, _g, _r),2);
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
void Gui::reprojectCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud, const Eigen::Vector4f &_position, const Eigen::Quaternionf &_orientation, unsigned _r, unsigned _g, unsigned _b) {
	vector<Point3f> points3d;
	for (const PointXYZ point : *_cloud)
		points3d.push_back(Point3f(point.x, point.y, point.z));
	vector<Point2f> reprojection1 = mStereoCameras.project3dPoints(points3d, true, _position, _orientation);
	vector<Point2f> reprojection2 = mStereoCameras.project3dPoints(points3d, false, _position, _orientation);
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
void Gui::drawCathegory(const ObjectCandidate & _candidate, const Eigen::Vector4f &_position, const Eigen::Quaternionf &_orientation) {
	vector<Point3f> points3d;
	for (const PointXYZ point : *_candidate.cloud())
		points3d.push_back(Point3f(point.x, point.y, point.z));

	vector<Point2f> reprojection1 = mStereoCameras.project3dPoints(points3d, true, _position, _orientation);
	vector<Point2f> reprojection2 = mStereoCameras.project3dPoints(points3d, false, _position, _orientation);

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
Gui::Gui(string _name, StereoCameras& _stereoCameras) : mName(_name), m3dViewer(new PCLVisualizer(mName)), mStereoCameras(_stereoCameras) {
	m3dViewer->createViewPort(0.0, 0.0, 0.5, 1.0, mViewPortMapViewer);
	m3dViewer->createViewPort(0.5, 0.0, 1.0, 1.0, mViewportPcViewer);
	m3dViewer->createViewPortCamera(mViewPortMapViewer);
	m3dViewer->setCameraFieldOfView(M_PI/180*40, mViewPortMapViewer);
	m3dViewer->createViewPortCamera(mViewportPcViewer);
	m3dViewer->setCameraFieldOfView(M_PI / 180 * 65, mViewportPcViewer);
	m3dViewer->setCameraPosition(0.0, 0.0, 0.05, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, mViewportPcViewer);
	m3dViewer->setCameraPosition(0.0, -0.2, -0.75,0.0,0.0,1.0, 0.0, -1.0, 0.1, mViewPortMapViewer);
	//m3dViewer->initCameraParameters();
	// Set up mapViewer
	m3dViewer->setBackgroundColor(0, 0, 0, mViewPortMapViewer);
	m3dViewer->addCoordinateSystem(0.25, "XYZ_map", mViewPortMapViewer);
	PointCloud<PointXYZ>::Ptr emptyCloud(new PointCloud<PointXYZ>);
	m3dViewer->addPointCloud(emptyCloud, "map", mViewPortMapViewer);
	m3dViewer->addText("Map Viewer", 10, 10, "MapViewer text", mViewPortMapViewer);
	// Set up pc viewer
	m3dViewer->setBackgroundColor(0.1, 0.1, 0.1, mViewportPcViewer);
	m3dViewer->addCoordinateSystem(0.05, "XYZ_pc", mViewportPcViewer);
	m3dViewer->addText("Cloud Viewer", 10, 10, "PcViewer text", mViewportPcViewer);
	//connect keyboard event
	m3dViewer->registerKeyboardCallback<Gui>(&Gui::keyboardEventOccurred, *this, (void*)&m3dViewer);

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
void Gui::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &_event, void* _viewer_void){
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (_viewer_void);
	std::cout << _event.getKeySym() << " was pressed";
	if ((_event.getKeySym() == "1") && _event.keyUp()) {
		std::cout << " => guess visualization toggled" << std::endl;
		mShowGuess = !mShowGuess;
	}
	if ((_event.getKeySym() == "2") && _event.keyUp()) {
		std::cout << " => icp result visualization toggled" << std::endl;
		mShowIcpResult = !mShowIcpResult;
	}
	if ((_event.getKeySym() == "3") && _event.keyUp()) {
		std::cout << " => candidate visualization toggled" << std::endl;
		mShowCandidates = !mShowCandidates;
	} else {
		std::cout << std::endl;
	}
}

bool Gui::isTagAllowed(const std::string & _tag) {
	if (_tag == "guess") {
		return mShowGuess;
	}
	else if (_tag == "icpResult"){
		return mShowIcpResult;
	}
	else if (_tag.find("candidate") != string::npos) {
		return mShowCandidates;
	}
	else {
		return false;
	}
	
}
