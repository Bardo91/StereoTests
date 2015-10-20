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
public:
	EnvironmentMap();
	EnvironmentMap(pcl::PointCloud<pcl::PointXYZRGB> _firstCloud);

	void clear();
	void clean();
	bool update(pcl::PointCloud<pcl::PointXYZRGB> _cloud);

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>> clusterCloud();

private:
};	// class EnvironmentMap

#endif	//	ENVIRONMENTMAP_H_