//
//
//
//
//

#include "EnvironmentMap.h"
#include <opencv2/opencv.hpp>

using namespace pcl;
using namespace std;
using namespace Eigen;

//---------------------------------------------------------------------------------------------------------------------
EnvironmentMap::EnvironmentMap(EnvironmentMap::Params _params) {
	mParams = _params;

	// Initialize members
	// Init voxel class
	mVoxelGrid.setLeafSize(_params.voxelSize, _params.voxelSize, _params.voxelSize);

	// Init filtering class
	mOutlierRemoval.setMeanK(_params.outlierMeanK);
	mOutlierRemoval.setStddevMulThresh(_params.outlierStdDev);
	mOutlierRemoval.setNegative(_params.outlierSetNegative);

	// Init ICP-NL class
	mPcJoiner.setTransformationEpsilon (_params.icpMaxTransformationEpsilon);
	mPcJoiner.setMaxCorrespondenceDistance (_params.icpMaxCorrespondenceDistance);  
	mPcJoiner.setMaximumIterations (_params.icpMaxIcpIterations);
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

//---------------------------------------------------------------------------------------------------------------------
void EnvironmentMap::addPoints(const PointCloud<PointXYZ>::Ptr & _cloud) {
	// Store First cloud as reference
	if (mCloud.size() == 0) {
		mCloud += *voxel(filter(_cloud));
	}

	// Storing and processing history of point clouds.
	unsigned cHistorySize = 2;
	mCloudHistory.push_back(_cloud);
		
	if (mCloudHistory.size() >= cHistorySize) {
		// Now we consider only 2 clouds on history, if want to increase it, need to define points with probabilities.
		PointCloud<PointXYZ>::Ptr cloud1 = voxel(filter(mCloudHistory[0]));
		PointCloud<PointXYZ>::Ptr cloud2 = voxel(filter(mCloudHistory[1]));

		PointCloud<PointXYZ> transformedCloud1;
		PointCloud<PointXYZ> transformedCloud2;

		Matrix4f transformation = getTransformationBetweenPcs(*cloud1, mCloud);
		transformPointCloud(*cloud1, transformedCloud1, transformation);
		transformation = getTransformationBetweenPcs(*cloud2, mCloud);
		transformPointCloud(*cloud2, transformedCloud2, transformation);


		PointCloud<PointXYZ> andCloud = convoluteCloudsOnGrid(transformedCloud1, transformedCloud2);
		mCloud += andCloud;
		// Finally discart oldest cloud
		mCloudHistory.pop_front();
	}
	//if (mCloud.size() == 0) {
	//	mCloud += *voxel(filter(_cloud));
	//} else {
	//	PointCloud<PointXYZ>::Ptr filteredCloud(new PointCloud<PointXYZ>);
	//	filteredCloud = filter(_cloud);
	//	PointCloud<PointXYZ>::Ptr voxeledCloud = voxel(filteredCloud);
	//	Matrix4f transformation = getTransformationBetweenPcs(*voxeledCloud, mCloud);
	//	PointCloud<PointXYZ> transformedCloud;
	//	transformPointCloud(*filteredCloud, transformedCloud, transformation);
	//	
	//	mCloud += transformedCloud;
	//	mCloud = *voxel(mCloud.makeShared());
	//}
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
	PointCloud<PointXYZ> cloud1 = _newCloud;
	PointCloud<PointXYZ> cloud2 = _fixedCloud;

	mPcJoiner.setInputSource (cloud1.makeShared());
	mPcJoiner.setInputTarget (cloud2.makeShared());

	Matrix4f Ti = Matrix4f::Identity (), prev, targetToSource;
	PointCloud<PointXYZ> alignedCloud1 = cloud1;
	for (int i = 0; i < mParams.icpMaxCorrDistDownStepIterations; ++i) {
		cloud1 = alignedCloud1;

		mPcJoiner.setInputSource (cloud1.makeShared());
		mPcJoiner.align (alignedCloud1);

		//accumulate transformation between each Iteration
		Ti = mPcJoiner.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one is smaller than the threshold,  refine the process by reducing the maximal correspondence distance
		if (fabs ((mPcJoiner.getLastIncrementalTransformation () - prev).sum ()) < mPcJoiner.getTransformationEpsilon ())
			mPcJoiner.setMaxCorrespondenceDistance (mPcJoiner.getMaxCorrespondenceDistance () - mParams.icpMaxCorrDistDownStep);

		prev = mPcJoiner.getLastIncrementalTransformation ();
	}

	// Get the transformation from target to source
	targetToSource = Ti;//.inverse();
	return targetToSource;
}

//---------------------------------------------------------------------------------------------------------------------
PointCloud<PointXYZRGB>::Ptr colorizePointCloud(const PointCloud<PointXYZ>::Ptr &_cloud, int _r, int _g, int _b) {
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
PointCloud<PointXYZ> EnvironmentMap::convoluteCloudsOnGrid(const PointCloud<PointXYZ>& _cloud1, const PointCloud<PointXYZ>& _cloud2) {
	PointCloud<PointXYZ> outCloud;
	bool isFirstLarge = _cloud1.size() > _cloud2.size() ? true:false;

	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")); 
	//viewer->setBackgroundColor (0, 0, 0); 
	//viewer->addCoordinateSystem (1.0); 
	//viewer->addPointCloud<pcl::PointXYZ> (mCloud.makeShared(),  "Ori"); 
	//viewer->addPointCloud<pcl::PointXYZRGB> (colorizePointCloud(_cloud1.makeShared(), 255,0 ,0),  "Input1"); 
	//viewer->addPointCloud<pcl::PointXYZRGB> (colorizePointCloud(_cloud2.makeShared(), 0, 255,0),  "Input2"); 
	//viewer->addPointCloud<pcl::PointXYZRGB> (colorizePointCloud(filteredCloud, 0, 0,255),  "Voxeled"); 
	//viewer->addPointCloud<pcl::PointXYZRGB> (colorizePointCloud(transformedCloud.makeShared(), 255, 0,0),  "Transformed"); 
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Ori"); 
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Input1"); 
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "Input2"); 
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "Voxeled"); 
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "Transformed"); 

	//cv::waitKey();

	// Put larger structure into VoxelGrid.
	mVoxelGrid.setSaveLeafLayout(true);
	if (isFirstLarge) 
		voxel(_cloud1.makeShared());
	else
		voxel(_cloud2.makeShared());

	// Iterate over the smaller cloud
	for (PointXYZ point : isFirstLarge? _cloud2 : _cloud1) {
		Eigen::Vector3i voxelCoord = mVoxelGrid.getGridCoordinates(point.x, point.y, point.z);
		int index = mVoxelGrid.getCentroidIndexAt(voxelCoord);
		if (index != -1) {
			outCloud.push_back(point);	// If have more than 2 clouds on history, needed probabilities. 666 TODO
			//std::cout << "--> Got  match! <--" << std::endl;
		}
	}
	std::cout << "Size of first: " << _cloud1.size() << std::endl;
	std::cout << "Size of second: " << _cloud2.size() << std::endl;
	std::cout << "Size of result: " << outCloud.size() << std::endl;
	return outCloud;
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
