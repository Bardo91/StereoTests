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
	EnvironmentMap();
	EnvironmentMap(pcl::PointCloud<pcl::PointXYZ> &_firstCloud);

	void clear();
	void clean();
	bool update(const pcl::PointCloud< pcl::PointXYZ> &_cloud);

	std::vector<pcl::PointCloud<pcl::PointXYZ>> clusterCloud();

private:	// Private methods

private:	// Members
	pcl::PointCloud<pcl::PointXYZ> mCloud;

};	// class EnvironmentMap

#endif	//	ENVIRONMENTMAP_H_