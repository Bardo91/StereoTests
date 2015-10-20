//
//
//
//
//

#include "EnvironmentMap.h"

// Joining and alignement
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>


using namespace pcl;
using namespace std;

//---------------------------------------------------------------------------------------------------------------------
// Define a new point representation for < x, y, z, curvature >
class PointXYZC : public pcl::PointRepresentation<PointNormal> {
	using pcl::PointRepresentation<PointNormal>::nr_dimensions_;
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
}

//---------------------------------------------------------------------------------------------------------------------
void EnvironmentMap::filter() {
}

//---------------------------------------------------------------------------------------------------------------------
void EnvironmentMap::addPoints(const pcl::PointCloud<pcl::PointXYZ>& _cloud) {
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
pcl::PointCloud<pcl::PointXYZ> EnvironmentMap::cloud() {
	return mCloud;
}

//---------------------------------------------------------------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZ> EnvironmentMap::concatenatePointClouds(const pcl::PointCloud<pcl::PointXYZ>& _cloud1, const pcl::PointCloud<pcl::PointXYZ>& _cloud2) {
	// Compute surface normals and curvature
	pcl::PointCloud<PointNormal> cloudAndNormals1 = computeNormals(_cloud1);
	pcl::PointCloud<PointNormal> cloudAndNormals2 = computeNormals(_cloud2);


	pcl::IterativeClosestPointNonLinear<PointNormal, PointNormal> reg;
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
	pcl::PointCloud<PointNormal> alignedCloud1 = cloudAndNormals1;
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
	pcl::transformPointCloud (mCloud, output, targetToSource);

	//add the source to the transformed target
	output += _cloud1;

	return output;
}

//---------------------------------------------------------------------------------------------------------------------
pcl::PointCloud<pcl::PointNormal> EnvironmentMap::computeNormals(const PointCloud<PointXYZ>& _pointCloud) {
	pcl::NormalEstimation<PointXYZ, PointNormal> estimator;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	estimator.setSearchMethod (tree);
	estimator.setKSearch (30);

	pcl::PointCloud<PointNormal> cloudAndNormals;
	estimator.setInputCloud (_pointCloud.makeShared());
	estimator.compute (cloudAndNormals);
	pcl::copyPointCloud (_pointCloud, cloudAndNormals);
	
	return cloudAndNormals;
}
