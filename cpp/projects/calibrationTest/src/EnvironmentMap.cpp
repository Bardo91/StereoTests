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

// INTERNAL STRUCTURES FOR JOINING POINT CLOUDS
//convenient structure to handle our pointclouds
struct PCD
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	std::string f_name;

	PCD() : cloud (new pcl::PointCloud<pcl::PointXYZ>) {};
};

struct PCDComparator
{
	bool operator () (const PCD& p1, const PCD& p2)
	{
		return (p1.f_name < p2.f_name);
	}
};


// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation<PointNormal> {
	using pcl::PointRepresentation<PointNormal>::nr_dimensions_;
public:
	MyPointRepresentation ()
	{
		// Define the number of dimensions
		nr_dimensions_ = 4;
	}

	// Override the copyToFloatArray method to define our feature vector
	virtual void copyToFloatArray (const PointNormal  &p, float * out) const
	{
		// < x, y, z, curvature >
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
		concatenatePointClouds(_cloud, mCloud);
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
void EnvironmentMap::concatenatePointClouds(const pcl::PointCloud<pcl::PointXYZ>& _cloud1, const pcl::PointCloud<pcl::PointXYZ>& _cloud2) {
	pcl::VoxelGrid<PointXYZ> grid;

	// Compute surface normals and curvature
	pcl::PointCloud<PointNormal> cloudAndNormals1 = computeNormals(_cloud1);
	pcl::PointCloud<PointNormal> cloudAndNormals2 = computeNormals(_cloud2);

	// Instantiate our custom point representation (defined above) ...
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[4] = {1.0, 1.0, 1.0, 1.0};
	point_representation.setRescaleValues (alpha);

	// Align
	pcl::IterativeClosestPointNonLinear<PointNormal, PointNormal> reg;
	reg.setTransformationEpsilon (1e-6);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance (100);  
	// Set the point representation
	reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

	reg.setInputSource (cloudAndNormals1.makeShared());
	reg.setInputTarget (cloudAndNormals2.makeShared());

	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
	pcl::PointCloud<PointNormal>::Ptr reg_result = cloudAndNormals1.makeShared();
	reg.setMaximumIterations (2);
	for (int i = 0; i < 30; ++i) {
		// save cloud for visualization purpose
		cloudAndNormals1 = *reg_result;

		// Estimate
		reg.setInputSource (cloudAndNormals1.makeShared());
		reg.align (*reg_result);

		//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one is smaller than the threshold, 
		//refine the process by reducing the maximal correspondence distance
		if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
			reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

		prev = reg.getLastIncrementalTransformation ();
	}

	//
	// Get the transformation from target to source
	targetToSource = Ti.inverse();

	PointCloud<PointXYZ> output;
	//
	// Transform target back in source frame
	pcl::transformPointCloud (mCloud, output, targetToSource);

	//add the source to the transformed target
	output += _cloud1;

	mCloud = output;

	Eigen::Matrix4f final_transform = targetToSource;

}

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
