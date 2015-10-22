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
#include <pcl/filters/voxel_grid.h>

class EnvironmentMap {
public:		// Public interface
	/// Basic constructor. Initialize an empty map
	EnvironmentMap(float _voxelSize);

	/// Remove internal pointcloud
	void clear();

	/// Filter internal pointcloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud);

	/// Add points into internal cloud.
	/// \param _cloud:
	void addPoints(const pcl::PointCloud< pcl::PointXYZ>::Ptr &_cloud);

	/// voxelate current map/pointcloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr voxel(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud);

	/// Cluster internal point cloud and returns vector with clusters
	/// \return  
	std::vector<pcl::PointCloud<pcl::PointXYZ>> clusterCloud();

	/// Get point cloud
	pcl::PointCloud<pcl::PointXYZ> cloud();

private:	// Private methods
	pcl::PointCloud<pcl::PointXYZ> concatenatePointClouds(const pcl::PointCloud< pcl::PointXYZ> &_newCloud, const pcl::PointCloud< pcl::PointXYZ> &_fixedCloud);
	pcl::PointCloud<pcl::PointNormal> computeNormals(const pcl::PointCloud<pcl::PointXYZ> &_pointCloud);

private:	// Members
	pcl::PointCloud<pcl::PointXYZ> mCloud;
	pcl::VoxelGrid<pcl::PointXYZ> mVoxelGrid;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> mOutlierRemoval;

};	// class EnvironmentMap

#endif	//	ENVIRONMENTMAP_H_