//
//
//
//


#ifndef GUI_H_
#define GUI_H_

#include <boost/thread/thread.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <StereoLib/StereoCameras.h>
#include <StereoLib/ObjectCandidate.h>


class Gui {
public:		// Static interface
	static void	init(std::string _name, StereoCameras& _stereoCameras);
	static void	end();
	static Gui*	get();

public:		// Public interface
	/// Draw the given pointcloud to the map viewer.
	/// \param _map: pcl point cloud that is wanted to be drawn.
	void drawMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_map);

	/// Draw a plane into de map.
	void drawPlane(const pcl::ModelCoefficients &_plane, std::string _tag = "");
	void drawPlane(const pcl::ModelCoefficients &_plane, double _x, double _y, double _z, std::string _tag = "");

	/// Draw a plane into de map.
	void drawLine(const pcl::PointXYZ &_p1, const pcl::PointXYZ &_p2, unsigned _r = 255, unsigned _g =255, unsigned _b = 255, std::string _tag = "");

	/// Draw a camera shape on the map.
	/// \param _pos: Camera position.
	///	\param _ori: Camera orientation.
	void drawCamera(const Eigen::Matrix3f &_orientation, const Eigen::Vector4f &_position, unsigned _r = 255, unsigned _g =255, unsigned _b = 255, std::string _tag = "");

	/// Clear map viewer.
	void clearMap();

	/// Draw the object candidate point cloud and the image reprojection
	void drawCandidate(const ObjectCandidate & _candidate, const Eigen::Vector4f &_position, const Eigen::Quaternionf &_orientation);

	/// Add cluster of single object in the point cloud;
	void addCloudToMapViewer(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cluster,  unsigned _pointSize, unsigned _r, unsigned _g, unsigned _b, std::string _tag = "");

	/// This viewer is a general purpose viewer for displaying point cloud.
	/// \param _cloud: Point cloud to be displayed
	/// \param _pointSize: Wanted size of point for display
	/// \param _r: Red (0-255) channel of the final desired color for the _cloud
	/// \param _g: green (0-255) channel of the final desired color for the _cloud
	/// \param _b: blue (0-255) channel of the final desired color for the _cloud
	void addCloudToPcViewer(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, unsigned _pointSize = 1, unsigned _r=255, unsigned _g=255, unsigned _b=255, std::string _tag = "");
	
	/// Clear pc viewer
	void clearPcViewer();

	/// Set a new pair of images to display in stereo viewer
	/// \param: _left: left image if the stereo pair
	/// \param: _right: right image if the stereo pair
	void updateStereoImages(const cv::Mat &_left, const cv::Mat &_right);
	
	/// Mark image as blurry
	/// \param _left: true to mark left image, false to right
	void putBlurry(bool _left);

	/// Add Text to pair of images
	/// \param _text: text to be added
	void addText(std::string _text, unsigned char _r=255, unsigned char _g=255, unsigned char _b=255);

	/// Draw points over the stereo pair of images
	/// \param _points: Points to be drawn relative to single image coordinates
	/// \param _isLeft: True if points belong to left image, false if belong to right image.
	/// \param _r: Red (0-255) channel of the final desired color for the _points
	/// \param _g: green (0-255) channel of the final desired color for the _points
	/// \param _b: blue (0-255) channel of the final desired color for the _points
	void drawPoints(const std::vector<cv::Point2f> &_points, bool _isLeft, unsigned _r=255, unsigned _g=255, unsigned _b=255);

	/// Draw boxes over the stereo pair of images
	/// \param _boxes: Boxes to be drawn
	/// \param _isLeft: True if points belong to left image, false if belong to right image.
	/// \param _r: Red (0-255) channel of the final desired color for the _boxes
	/// \param _g: green (0-255) channel of the final desired color for the _boxes
	/// \param _b: blue (0-255) channel of the final desired color for the _boxes
	void drawBoundBoxes(const std::vector<cv::Rect> &_boxes, bool _isLeft, unsigned _r=255, unsigned _g=255, unsigned _b=255);

	/// Draw box on the stereo pair of images
	/// \param _boxes: Box to be drawn
	/// \param _isLeft: True if points belong to left image, false if belong to right image.
	/// \param _r: Red (0-255) channel of the final desired color for the _boxes
	/// \param _g: green (0-255) channel of the final desired color for the _boxes
	/// \param _b: blue (0-255) channel of the final desired color for the _boxes
	void drawBox(const cv::Rect &_box, bool _isLeft, unsigned _r=255, unsigned _g=255, unsigned _b=255);


	/// Draw Poligon over the stereo pair of images
	/// \param _boxes: Points to be drawn relative to single image coordinates
	/// \param _isLeft: True if points belong to left image, false if belong to right image.
	/// \param _r: Red (0-255) channel of the final desired color for the _boxes
	/// \param _g: green (0-255) channel of the final desired color for the _boxes
	/// \param _b: blue (0-255) channel of the final desired color for the _boxes
	void drawPolygon(const std::vector<cv::Point2f> &_polygon, bool _isLeft, unsigned _r=255, unsigned _g=255, unsigned _b=255);

	/// Reproject the pointcloud using the stereoParamteres, that are updated by the environmentmap
	/// \param _cloud: The cloud to be projected	
	/// \param _r: Red (0-255) channel of the final desired color for the _boxes
	/// \param _g: green (0-255) channel of the final desired color for the _boxes
	/// \param _b: blue (0-255) channel of the final desired color for the _boxes
	void reprojectCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud, const Eigen::Vector4f &_position, const Eigen::Quaternionf &_orientation, unsigned _r = 255, unsigned _g = 255, unsigned _b = 255);

	void drawCathegory(const ObjectCandidate &_candidate, const Eigen::Vector4f &_position, const Eigen::Quaternionf &_orientation);

	/// needed to update the viewer
	void spinOnce();

private:	// Private methods
	Gui(std::string _name, StereoCameras& _stereoCameras);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, int _r, int _g, int _b);
	void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &_event, void* _viewer_void);

	bool isTagAllowed(const std::string &_tag);

private:	// Members
	static Gui	*mInstance;
	
	std::string	mName;	// Base name for all windows.
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> m3dViewer;
	int mViewPortMapViewer = 0, mViewportPcViewer = 1;

	cv::Mat mLeftImage, mRightImage, mPairStereo;
	unsigned mLinesOfText = 0;

	unsigned mPcCounter = 0;	// This variable is used to generate different names between pointcloud inside the vizualizer.
								// 666 TODO: check it.
	StereoCameras& mStereoCameras;
	bool mShowGuess = true;
	bool mShowIcpResult = true;
	bool mShowCandidates = true;
};	//	class Gui

#endif	//	GUI_H_