//
//
//
//
//

#include "EnvironmentMap.h"

// Joining and alignement
#include <boost/make_shared.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_representation.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>


// Point cloud filter
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>


using namespace pcl;
using namespace std;

//---------------------------------------------------------------------------------------------------------------------
// Define a new point representation for < x, y, z, curvature >
class PointXYZC : public PointRepresentation<PointNormal> {
	using PointRepresentation<PointNormal>::nr_dimensions_;
public:
	PointXYZC (){
		nr_dimensions_ = 4;
	}

	virtual void copyToFloatArray (const PointNormal  &p, float * out) const {
		out[0] = p.x;
		out[1] = p.y;
		out[2] = p.z;
		out[3] = p.curvature;
	}
};

//---------------------------------------------------------------------------------------------------------------------
EnvironmentMap::EnvironmentMap() {

}

//---------------------------------------------------------------------------------------------------------------------
EnvironmentMap::EnvironmentMap(PointCloud<PointXYZ> &_firstCloud) {
	mCloud += _firstCloud;
}

//---------------------------------------------------------------------------------------------------------------------
void EnvironmentMap::clear() {
	mCloud.clear();
}

//---------------------------------------------------------------------------------------------------------------------
PointCloud<PointXYZ> EnvironmentMap::filter(PointCloud<PointXYZ> &_cloud) {
	StatisticalOutlierRemoval<PointXYZ> sor;
	PointCloud<PointXYZ> filteredCloud;
	sor.setInputCloud(_cloud.makeShared());
	sor.setMeanK(10);
	sor.setStddevMulThresh(0.1);
	ApproximateVoxelGrid<PointXYZ> avg;
	avg.setLeafSize(30, 30, 30);
	avg.setInputCloud(filteredCloud.makeShared());
	sor.setNegative(false);
	sor.filter(filteredCloud);
	return filteredCloud;
}

//---------------------------------------------------------------------------------------------------------------------
void EnvironmentMap::addPoints(const PointCloud<PointXYZ>& _cloud) {
	if (mCloud.size() == 0) {
		mCloud += _cloud;
		return;
	}
	else {
		mCloud = concatenatePointClouds(_cloud, mCloud);
	}
}

//---------------------------------------------------------------------------------------------------------------------
vector<PointCloud<PointXYZ>> EnvironmentMap::clusterCloud() {
	return vector<PointCloud<PointXYZ>>();
}

//---------------------------------------------------------------------------------------------------------------------
PointCloud<PointXYZ> EnvironmentMap::cloud() {
	return mCloud;
}

//---------------------------------------------------------------------------------------------------------------------
PointCloud<PointXYZ> EnvironmentMap::concatenatePointClouds(const PointCloud<PointXYZ>& _cloud1, const PointCloud<PointXYZ>& _cloud2) {
	// Compute surface normals and curvature
	PointCloud<PointNormal> cloudAndNormals1 = computeNormals(_cloud1);
	PointCloud<PointNormal> cloudAndNormals2 = computeNormals(_cloud2);


	IterativeClosestPointNonLinear<PointNormal, PointNormal> reg;
	reg.setTransformationEpsilon (1e-6);
	reg.setMaxCorrespondenceDistance (100);  
	reg.setMaximumIterations (2);
	
	PointXYZC point_representation;
	float alpha[4] = {1.0, 1.0, 1.0, 1.0};
	point_representation.setRescaleValues (alpha);
	reg.setPointRepresentation (boost::make_shared<const PointXYZC> (point_representation));

	reg.setInputSource (cloudAndNormals1.makeShared());
	reg.setInputTarget (cloudAndNormals2.makeShared());

	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
	PointCloud<PointNormal> alignedCloud1 = cloudAndNormals1;
	for (int i = 0; i < 30; ++i) {
		cloudAndNormals1 = alignedCloud1;

		reg.setInputSource (cloudAndNormals1.makeShared());
		reg.align (alignedCloud1);

		//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one is smaller than the threshold,  refine the process by reducing the maximal correspondence distance
		if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
			reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

		prev = reg.getLastIncrementalTransformation ();
	}

	// Get the transformation from target to source
	targetToSource = Ti.inverse();
	PointCloud<PointXYZ> output;
	transformPointCloud (mCloud, output, targetToSource);

	//add the source to the transformed target
	output += _cloud1;

	return output;
}

//---------------------------------------------------------------------------------------------------------------------
PointCloud<PointNormal> EnvironmentMap::computeNormals(const PointCloud<PointXYZ>& _pointCloud) {
	NormalEstimation<PointXYZ, PointNormal> estimator;
	search::KdTree<PointXYZ>::Ptr tree (new search::KdTree<PointXYZ> ());
	estimator.setSearchMethod (tree);
	estimator.setKSearch (30);

	PointCloud<PointNormal> cloudAndNormals;
	estimator.setInputCloud (_pointCloud.makeShared());
	estimator.compute (cloudAndNormals);
	copyPointCloud (_pointCloud, cloudAndNormals);
	
	return cloudAndNormals;
}
