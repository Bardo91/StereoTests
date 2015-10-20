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
using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
//our visualizer
pcl::visualization::PCLVisualizer *p;
//its left and right viewports
int vp_1, vp_2;	
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


void showCloudsLeft(const PointCloud<PointXYZ>::Ptr cloud_target, const PointCloud<PointXYZ>::Ptr cloud_source)
{
	p->removePointCloud ("vp1_target");
	p->removePointCloud ("vp1_source");

	PointCloudColorHandlerCustom<PointXYZ> tgt_h (cloud_target, 0, 255, 0);
	PointCloudColorHandlerCustom<PointXYZ> src_h (cloud_source, 255, 0, 0);
	p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
	p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

	PCL_INFO ("Press q to begin the registration.\n");
	p-> spin();
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Display source and target on the second viewport of the visualizer
*
*/
void showCloudsRight(const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_target, const pcl::PointCloud<pcl::PointNormal>::Ptr cloud_source)
{
	p->removePointCloud ("source");
	p->removePointCloud ("target");


	PointCloudColorHandlerGenericField<pcl::PointNormal> tgt_color_handler (cloud_target, "curvature");
	if (!tgt_color_handler.isCapable ())
		PCL_WARN ("Cannot create curvature color handler!");

	PointCloudColorHandlerGenericField<pcl::PointNormal> src_color_handler (cloud_source, "curvature");
	if (!src_color_handler.isCapable ())
		PCL_WARN ("Cannot create curvature color handler!");


	p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
	p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

	p->spinOnce();
}

//---------------------------------------------------------------------------------------------------------------------
EnvironmentMap::EnvironmentMap() {
	p = new pcl::visualization::PCLVisualizer("Pairwise Incremental Registration example");
	p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
	p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);

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
	// Downsample for consistency and speed
	// \note enable this for large datasets
	pcl::PointCloud<PointXYZ>::Ptr src (new pcl::PointCloud<PointXYZ>);
	pcl::PointCloud<PointXYZ>::Ptr tgt (new pcl::PointCloud<PointXYZ>);
	pcl::VoxelGrid<PointXYZ> grid;
	bool downsample = false;
	if (downsample)
	{
		grid.setLeafSize (0.05, 0.05, 0.05);
		grid.setInputCloud (_cloud.makeShared());
		grid.filter (*src);

		grid.setInputCloud (mCloud.makeShared());
		grid.filter (*tgt);
	}
	else
	{
		src = _cloud.makeShared();
		tgt = mCloud.makeShared();
	}


	// Compute surface normals and curvature
	pcl::PointCloud<PointNormal>::Ptr points_with_normals_src (new pcl::PointCloud<PointNormal>);
	pcl::PointCloud<PointNormal>::Ptr points_with_normals_tgt (new pcl::PointCloud<PointNormal>);

	pcl::NormalEstimation<PointXYZ, PointNormal> norm_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	norm_est.setSearchMethod (tree);
	norm_est.setKSearch (30);

	norm_est.setInputCloud (src);
	norm_est.compute (*points_with_normals_src);
	pcl::copyPointCloud (*src, *points_with_normals_src);

	norm_est.setInputCloud (tgt);
	norm_est.compute (*points_with_normals_tgt);
	pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

	//
	// Instantiate our custom point representation (defined above) ...
	MyPointRepresentation point_representation;
	// ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[4] = {1.0, 1.0, 1.0, 1.0};
	point_representation.setRescaleValues (alpha);

	//
	// Align
	pcl::IterativeClosestPointNonLinear<PointNormal, PointNormal> reg;
	reg.setTransformationEpsilon (1e-6);
	// Set the maximum distance between two correspondences (src<->tgt) to 10cm
	// Note: adjust this based on the size of your datasets
	reg.setMaxCorrespondenceDistance (100);  
	// Set the point representation
	reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation> (point_representation));

	reg.setInputSource (points_with_normals_src);
	reg.setInputTarget (points_with_normals_tgt);



	//
	// Run the same optimization in a loop and visualize the results
	Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
	pcl::PointCloud<PointNormal>::Ptr reg_result = points_with_normals_src;
	reg.setMaximumIterations (2);
	for (int i = 0; i < 30; ++i)
	{
		PCL_INFO ("Iteration Nr. %d.\n", i);

		// save cloud for visualization purpose
		points_with_normals_src = reg_result;

		// Estimate
		reg.setInputSource (points_with_normals_src);
		reg.align (*reg_result);

		//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one
		//is smaller than the threshold, refine the process by reducing
		//the maximal correspondence distance
		if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
			reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);

		prev = reg.getLastIncrementalTransformation ();

		// visualize current state
		showCloudsRight(points_with_normals_tgt, points_with_normals_src);
	}

	//
	// Get the transformation from target to source
	targetToSource = Ti.inverse();
	
	PointCloud<PointXYZ> output;
	//
	// Transform target back in source frame
	pcl::transformPointCloud (mCloud, output, targetToSource);

	//add the source to the transformed target
	output += _cloud;

	mCloud = output;

	Eigen::Matrix4f final_transform = targetToSource;
}

//---------------------------------------------------------------------------------------------------------------------
vector<PointCloud<PointXYZ>> EnvironmentMap::clusterCloud() {
	return vector<PointCloud<PointXYZ>>();
}

pcl::PointCloud<pcl::PointXYZ> EnvironmentMap::cloud()
{
	return mCloud;
}
