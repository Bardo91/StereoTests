//
//
//
//
//


#ifndef OBJECTCANDIDATE_H_
#define OBJECTCANDIDATE_H_

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

class ObjectCandidate
{
public:
	ObjectCandidate(pcl::PointIndices _pointIndeces, pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud, bool _copyCloudPoints);
	~ObjectCandidate();

	/// Get point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud() const;

	unsigned R() const;

	unsigned G() const;

	unsigned B() const;



private:
	pcl::PointIndices mPointIndices;
	pcl::PointCloud<pcl::PointXYZ>::Ptr mCloud;
	unsigned mR, mG, mB;
};

#endif
