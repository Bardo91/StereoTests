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


class Gui {
public:		// Static interface
	static void	init(std::string _name);
	static void	end();
	static Gui*	get();

public:		// Public interface
	/// Draw the given pointcloud to the map viewer.
	/// \param _map: pcl point cloud that is wanted to be drawn.
	void drawMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_map);
	
	/// Clear map viewer.
	void clearMap();

	/// This viewer is a general purpose viewer for displaying point cloud.
	/// \param _cloud: Point cloud to be displayed
	/// \param _pointSize: Wanted size of point for display
	/// \param _r: Red (0-255) channel of the final desired color for the _cloud
	/// \param _g: green (0-255) channel of the final desired color for the _cloud
	/// \param _b: blue (0-255) channel of the final desired color for the _cloud
	void addPointToPcViewer(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, unsigned _pointSize = 1, unsigned _r=255, unsigned _g=255, unsigned _b=255);
	
	/// Clear pc viewer
	void clearPcViewer();

	/// Set a new pair of images to display in stereo viewer
	/// \param: _left: left image if the stereo pair
	/// \param: _right: right image if the stereo pair
	void updateStereoImages(const cv::Mat &_left, const cv::Mat &_right);
	
	/// Draw points over the stereo pair of images
	/// \param _points: Points to be drawn relative to single image coordinates
	/// \param _isLeft: True if points belong to left image, false if belong to right image.
	/// \param _r: Red (0-255) channel of the final desired color for the _points
	/// \param _g: green (0-255) channel of the final desired color for the _points
	/// \param _b: blue (0-255) channel of the final desired color for the _points
	void drawPoints(const std::vector<cv::Point2i> &_points, bool _isLeft, unsigned _r=255, unsigned _g=255, unsigned _b=255);

	/// Draw boxes over the stereo pair of images
	/// \param _boxes: Points to be drawn relative to single image coordinates
	/// \param _isLeft: True if points belong to left image, false if belong to right image.
	/// \param _r: Red (0-255) channel of the final desired color for the _boxes
	/// \param _g: green (0-255) channel of the final desired color for the _boxes
	/// \param _b: blue (0-255) channel of the final desired color for the _boxes
	void drawBoundBoxes(const std::vector<cv::Rect> &_boxes, bool _isLeft, unsigned _r=255, unsigned _g=255, unsigned _b=255);

private:	// Private methods
	Gui(std::string _name);

private:	// Members
	static Gui	*mInstance;
	
	std::string	mName;	// Base name for all windows.
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> mMapViewer;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> mPcViewer;

	cv::Mat mLeftImage, mRightImage;

};	//	class Gui

#endif	//	GUI_H_