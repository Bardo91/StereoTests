//
//
//
//
//


#ifndef ENVIRONMENTMAP_H_
#define ENVIRONMENTMAP_H_

// 666 TODO: clean includes

#include <vector>


#include <boost/make_shared.hpp>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>

#include "TimeTools.h"



class EnvironmentMap {
public:		// Public interface
	/// Internal structure to configure algorithms
	struct Params {
		// Voxeling parameters.
		float	voxelSize;

		// Filter outlier removal.
		int		outlierMeanK;
		float	outlierStdDev;
		bool	outlierSetNegative;

		// ICP-NL
		double	icpMaxTransformationEpsilon;
		double	icpEuclideanEpsilon;
		int		icpMaxIcpIterations;
		float	icpMaxCorrespondenceDistance;
		float	icpMaxCorrDistDownStep;
		int		icpMaxCorrDistDownStepIterations;

		// Pointcloud history filtering
		unsigned	historySize;

		// Euclidean clustering
		double	clusterTolerance;
		int		minClusterSize;
		int		maxClusterSize;

		// Floor extractor
		double			floorCameraMinAngle;
		double			floorCameraMaxAngle;
		double			floorDistanceThreshold;
		unsigned		floorMaxIters;
	};

	/// Basic constructor. Initialize an empty map
	EnvironmentMap(Params _params);

	/// Remove internal pointcloud
	void clear();

	/// Add points into internal cloud.
	/// \param _cloud:
	void addPoints(const pcl::PointCloud< pcl::PointXYZ>::Ptr &_cloud);

	/// Cluster internal point cloud and returns vector with clusters
	/// \return  
	std::vector<pcl::PointIndices> clusterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud);
	std::vector<pcl::PointIndices> clusterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &_clusters);

	/// Get point cloud
	pcl::PointCloud<pcl::PointXYZ> cloud();

	pcl::PointCloud<pcl::PointXYZ>::Ptr lastJoinedCloud();

	/// Look for planes in the given pointcloud.
	pcl::ModelCoefficients  extractFloor(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud);

	/// Crop the map using a plane
	void cropMap(pcl::ModelCoefficients _plane, bool _upperSide = true);

private:	// Private methods
	// Filter internal pointcloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud);
	// voxelate current map/pointcloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr voxel(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud);

	// Calculate transformation between two point cloud using ICP-NL algorithm.
	Eigen::Matrix4f getTransformationBetweenPcs(const pcl::PointCloud<pcl::PointXYZ> &_newCloud, const pcl::PointCloud< pcl::PointXYZ> &_fixedCloud);
	
	// This method use the internal voxel grid to downsample the input clouds. 
	// Then operate a 1x1 convolution between both: 
	//			exists(cloud1(x,y,z)) && exists(cloud2(x,y,z)) ? Point(x,y,z) : null
	pcl::PointCloud<pcl::PointXYZ> convoluteCloudsOnGrid(const pcl::PointCloud<pcl::PointXYZ> &_cloud1, const pcl::PointCloud<pcl::PointXYZ> &_cloud2);

	bool validTransformation(const Eigen::Matrix4f &_transformation, double _maxAngle, double _maxTranslation);
	
private:	// Members
	Params	mParams;

	std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr>		mCloudHistory;
	pcl::PointCloud<pcl::PointXYZ>						mCloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr					mLastJoinedCloud;
	pcl::VoxelGrid<pcl::PointXYZ>						mVoxelGrid;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ>		mOutlierRemoval;
	pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ>	mPcJoiner;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> mEuclideanClusterExtraction;


	const double cMaxAngle			= M_PI/180*1;	// 1º
	const double cMaxTranslation	= 5;			// 10 mm 

	Eigen::Matrix4f mPreviousCloud2MapTransformation = Eigen::Matrix4f::Identity();
};	// class EnvironmentMap

#endif	//	ENVIRONMENTMAP_H_