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



#include <pcl/visualization/cloud_viewer.h>


using namespace pcl;
using namespace std;
using namespace Eigen;

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
EnvironmentMap::EnvironmentMap(float _voxelSize) {
	mVoxelGrid.setLeafSize(_voxelSize, _voxelSize, _voxelSize);
	mOutlierRemoval.setMeanK(10);
	mOutlierRemoval.setStddevMulThresh(0.05);
	mOutlierRemoval.setNegative(false);
}

//---------------------------------------------------------------------------------------------------------------------
void EnvironmentMap::clear() {
	mCloud.clear();
}

//---------------------------------------------------------------------------------------------------------------------
PointCloud<PointXYZ>::Ptr EnvironmentMap::filter(const PointCloud<PointXYZ>::Ptr &_cloud) {
	PointCloud<PointXYZ>::Ptr filteredCloud(new PointCloud<PointXYZ>);
	mOutlierRemoval.setInputCloud(_cloud);
	mOutlierRemoval.filter(*filteredCloud);
	return filteredCloud;
}

PointCloud<PointXYZRGB>::Ptr colorizePointCloud(PointCloud<PointXYZ>::Ptr _cloud, int _r, int _g, int _b) {
	PointCloud<PointXYZRGB>::Ptr colorizedCloud(new PointCloud<PointXYZRGB>);
	for (PointXYZ point : *_cloud) {
		PointXYZRGB p;
		p.x = point.x;
		p.y = point.y;
		p.z = point.z;
		p.r = _r;
		p.g = _g;
		p.b = _b;
		colorizedCloud->push_back(p);
	}
	return colorizedCloud;
}
//---------------------------------------------------------------------------------------------------------------------
void EnvironmentMap::addPoints(const PointCloud<PointXYZ>::Ptr & _cloud) {
	if (mCloud.size() == 0) {
		mCloud += *voxel(filter(_cloud));
	} else {
		//visualization::CloudViewer viewer("Debug cloud");
		//viewer.showCloud(mCloud.makeShared(), "Ori");
		//system("PAUSE");
		//viewer.showCloud(colorizePointCloud(_cloud, 255, 0,0), "Input");
		//system("PAUSE");
		PointCloud<PointXYZ>::Ptr filteredCloud(new PointCloud<PointXYZ>);
		filteredCloud = filter(_cloud);
		//viewer.showCloud(colorizePointCloud(filteredCloud, 0, 255,0), "Filtered");
		//system("PAUSE");
		PointCloud<PointXYZ>::Ptr voxeledCloud = voxel(filteredCloud);
		//viewer.showCloud(colorizePointCloud(filteredCloud, 0, 0,255), "Voxeled");
		//system("PAUSE");
		Matrix4f transformation = getTransformationBetweenPcs(*voxeledCloud, mCloud);
		PointCloud<PointXYZ> transformedCloud;
		transformPointCloud(*voxeledCloud, transformedCloud, transformation);
		//viewer.showCloud(colorizePointCloud(transformedCloud.makeShared(), 0, 255,255), "Transformed");
		//system("PAUSE");

		//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")); 
		//viewer->setBackgroundColor (0, 0, 0); 
		//viewer->addPointCloud<pcl::PointXYZ> (mCloud.makeShared(),  "Ori"); 
		//viewer->addPointCloud<pcl::PointXYZRGB> (colorizePointCloud(_cloud, 0, 255,0),  "Input"); 
		//viewer->addPointCloud<pcl::PointXYZRGB> (colorizePointCloud(voxeledCloud, 0, 255,0),  "Filtered"); 
		//viewer->addPointCloud<pcl::PointXYZRGB> (colorizePointCloud(filteredCloud, 0, 0,255),  "Voxeled"); 
		//viewer->addPointCloud<pcl::PointXYZRGB> (colorizePointCloud(transformedCloud.makeShared(), 255, 0,0),  "Transformed"); 
		//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "Ori"); 
		//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "Input"); 
		//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "Filtered"); 
		//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "Voxeled"); 
		//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "Transformed"); 
		//viewer->addCoordinateSystem (1.0); 

		
		mCloud += transformedCloud;
		mCloud = *voxel(mCloud.makeShared());
	}
}

//---------------------------------------------------------------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr EnvironmentMap::voxel(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud) {
	mVoxelGrid.setInputCloud(_cloud);
	PointCloud<PointXYZ>::Ptr voxeled(new PointCloud<PointXYZ>);
	mVoxelGrid.filter(*voxeled);
	return voxeled;
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
Matrix4f EnvironmentMap::getTransformationBetweenPcs(const PointCloud<PointXYZ>& _newCloud, const PointCloud<PointXYZ>& _fixedCloud) {
	// Compute surface normals and curvature
	PointCloud<PointNormal> cloudAndNormals1 = computeNormals(_newCloud);
	PointCloud<PointNormal> cloudAndNormals2 = computeNormals(_fixedCloud);

	IterativeClosestPointNonLinear<PointNormal, PointNormal> reg;
	reg.setTransformationEpsilon (1e-2);
	reg.setMaxCorrespondenceDistance (0.01);  
	reg.setMaximumIterations (3);
	
	PointXYZC point_representation;
	float alpha[4] = {1.0, 1.0, 1.0, 1.0};
	point_representation.setRescaleValues (alpha);
	reg.setPointRepresentation (boost::make_shared<const PointXYZC> (point_representation));

	reg.setInputSource (cloudAndNormals1.makeShared());
	reg.setInputTarget (cloudAndNormals2.makeShared());

	Matrix4f Ti = Matrix4f::Identity (), prev, targetToSource;
	PointCloud<PointNormal> alignedCloud1 = cloudAndNormals1;
	for (int i = 0; i < 30; ++i) {
		cloudAndNormals1 = alignedCloud1;

		reg.setInputSource (cloudAndNormals1.makeShared());
		reg.align (alignedCloud1);

		//accumulate transformation between each Iteration
		Ti = reg.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one is smaller than the threshold,  refine the process by reducing the maximal correspondence distance
		if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
			reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 1);

		prev = reg.getLastIncrementalTransformation ();
	}

	// Get the transformation from target to source
	targetToSource = Ti;//.inverse();
	return targetToSource;
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

//---------------------------------------------------------------------------------------------------------------------
bool EnvironmentMap::validTransformation(const Matrix4f & _transformation, double _maxAngle, double _maxTranslation) {
	Matrix3f rotation = _transformation.block<3,3>(0,0);
	Vector3f translation = _transformation.block<3,1>(0,3);
	
	Affine3f aff(Affine3f::Identity());
	aff = aff*rotation;
	
	float roll, pitch, yaw;
	getEulerAngles(aff, roll, pitch, yaw);

	std::cout << "Rotations: " << roll << ", " << pitch << ", " << yaw << std::endl;
	std::cout << "Translations: " << translation(0) << ", " << translation(1) << ", " << translation(2) << std::endl;

	if (abs(roll) < cMaxAngle && abs(pitch) < cMaxAngle && abs(yaw) < cMaxAngle  &&
		abs(translation(0)) < cMaxTranslation && abs(translation(1)) < cMaxTranslation && abs(translation(2)) < cMaxTranslation) {
		std::cout << "Valid point cloud rotation" << std::endl;
		return true;
	} else {
		std::cout << "Invalid point cloud rotation" << std::endl;
		return false;
	}
}
