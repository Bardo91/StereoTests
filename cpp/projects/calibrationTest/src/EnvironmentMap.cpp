//
//
//
//
//

#include "EnvironmentMap.h"

// Clustering includes
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace pcl;
using namespace std;

EnvironmentMap::EnvironmentMap() {
}

EnvironmentMap::EnvironmentMap(PointCloud<PointXYZ> &_firstCloud) {
	mCloud += _firstCloud;
}

void EnvironmentMap::clear() {
}

void EnvironmentMap::clean() {
} 

bool EnvironmentMap::update(const PointCloud<PointXYZ> &_cloud) {
	return false;
}

vector<PointCloud<PointXYZ>> EnvironmentMap::clusterCloud() {
	return vector<PointCloud<PointXYZ>>();
}
