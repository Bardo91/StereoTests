//
//
//
//
//

#include "EnvironmentMap.h"

EnvironmentMap::EnvironmentMap() {
}

EnvironmentMap::EnvironmentMap(pcl::PointCloud<pcl::PointXYZRGB> _firstCloud) {
}

void EnvironmentMap::clear() {
}

void EnvironmentMap::clean() {
} 

bool EnvironmentMap::update(pcl::PointCloud<pcl::PointXYZRGB> _cloud) {
	return false;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>> EnvironmentMap::clusterCloud() {
	return std::vector<pcl::PointCloud<pcl::PointXYZRGB>>();
}
