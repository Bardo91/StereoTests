//
//
//
//
//


#ifndef ENVIRONMENTMAP_H_
#define ENVIRONMENTMAP_H_

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class EnvironmentMap {
public:		// Public interface
	/// Basic constructor. Initialize an empty map
	EnvironmentMap();

	/// Constructor. Initialize map with an initial set of points
	/// \param _firstCloud: Initial set of points.
	EnvironmentMap(pcl::PointCloud<pcl::PointXYZ> &_firstCloud);

	/// Remove internal pointcloud
	void clear();

	/// Filter internal pointcloud
	void filter();

	/// Add points into internal cloud.
	/// \param _cloud:
	void addPoints(const pcl::PointCloud< pcl::PointXYZ> &_cloud);

	/// Cluster internal point cloud and returns vector with clusters
	/// \return  
	std::vector<pcl::PointCloud<pcl::PointXYZ>> clusterCloud();

	/// Get point cloud
	pcl::PointCloud<pcl::PointXYZ> cloud();

private:	// Private methods
	pcl::PointCloud<pcl::PointXYZ> concatenatePointClouds(const pcl::PointCloud< pcl::PointXYZ> &_cloud1, const pcl::PointCloud< pcl::PointXYZ> &_cloud2);
	pcl::PointCloud<pcl::PointNormal> computeNormals(const pcl::PointCloud<pcl::PointXYZ> &_pointCloud);

private:	// Members
	pcl::PointCloud<pcl::PointXYZ> mCloud;

};	// class EnvironmentMap

#endif	//	ENVIRONMENTMAP_H_