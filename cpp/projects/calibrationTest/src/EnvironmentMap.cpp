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
	mPcJoiner.setEuclideanFitnessEpsilon(_params.icpEuclideanEpsilon);

	// Init Euclidean Extraction
	mEuclideanClusterExtraction.setClusterTolerance(_params.clusterTolerance);
	mEuclideanClusterExtraction.setMinClusterSize(_params.minClusterSize);
	mEuclideanClusterExtraction.setMaxClusterSize(_params.maxClusterSize);
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
	// 666 we store the actual first cloud instead of considering history, this means we accept the noise from the first cloud
	if (mCloud.size() == 0) {
		mCloud += *voxel(filter(_cloud));
		mLastJoinedCloud = mCloud.makeShared();
	}

	// Storing and processing history of point clouds.
	mCloudHistory.push_back(_cloud);
	
	if (mCloudHistory.size() >= mParams.historySize) {
		// Get first pointcloud
		PointCloud<PointXYZ>::Ptr cloud1 = voxel(filter(mCloudHistory[0]));
		PointCloud<PointXYZ> transformedCloud1;
		Matrix4f transformation = getTransformationBetweenPcs(*cloud1, mCloud);
		transformPointCloud(*cloud1, transformedCloud1, transformation);
		

		for (unsigned i = 1; i < mParams.historySize; i++) {
			// Now we consider only 2 clouds on history, if want to increase it, need to define points with probabilities.
			PointCloud<PointXYZ>::Ptr cloud2 = voxel(filter(mCloudHistory[i]));

			PointCloud<PointXYZ> transformedCloud2;
			transformation = getTransformationBetweenPcs(*cloud2, mCloud);
			transformPointCloud(*cloud2, transformedCloud2, transformation);

			transformedCloud1 = convoluteCloudsOnGrid(transformedCloud1, transformedCloud2);
		}
		mLastJoinedCloud = transformedCloud1.makeShared();
		mLastView2MapTransformation = transformation; //needed for gui reprojection of map points
		mCloud += transformedCloud1;
		mCloud = *voxel(mCloud.makeShared());
		// Finally discart oldest cloud
		mCloudHistory.pop_front();
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
vector<PointIndices> EnvironmentMap::clusterCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud) {
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(_cloud);
	//vector of indices of each cluster
	std::vector<pcl::PointIndices> clusterIndices;
	mEuclideanClusterExtraction.setSearchMethod(tree);
	mEuclideanClusterExtraction.setInputCloud(_cloud);
	mEuclideanClusterExtraction.extract(clusterIndices);

	return clusterIndices;
}

//---------------------------------------------------------------------------------------------------------------------
PointCloud<PointXYZ> EnvironmentMap::cloud() {
	return mCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr EnvironmentMap::lastJoinedCloud()
{
	return mLastJoinedCloud;
}

Eigen::Matrix4f EnvironmentMap::lastView2MapTransformation()
{
	return mLastView2MapTransformation;
}

//---------------------------------------------------------------------------------------------------------------------
Matrix4f EnvironmentMap::getTransformationBetweenPcs(const PointCloud<PointXYZ>& _newCloud, const PointCloud<PointXYZ>& _fixedCloud) {
	// 666 we use the transformation guess in this function, so it is not as general for any input as we like, since
	// this general guess is in regards to the entire map (mCloud)
	//PointCloud<PointXYZ> cloud1 = _newCloud;
	//PointCloud<PointXYZ> cloud2 = _fixedCloud;
	BOViL::STime *timer = BOViL::STime::get();
	double t;

	mPcJoiner.setInputSource (_newCloud.makeShared());
	mPcJoiner.setInputTarget (_fixedCloud.makeShared());

	Matrix4f Ti = Matrix4f::Identity (), prev, targetToSource;
	PointCloud<PointXYZ> alignedCloud1;// = cloud1;
	for (int i = 0; i < mParams.icpMaxCorrDistDownStepIterations; ++i) {
		//cloud1 = alignedCloud1;
		double t0 = timer->getTime();
		//mPcJoiner.setInputSource (cloud1.makeShared());
		mPcJoiner.align (alignedCloud1, mPreviousCloud2MapTransformation);
		t = timer->getTime() - t0;
		//accumulate transformation between each Iteration
		Ti = mPcJoiner.getFinalTransformation () * Ti;

		//if the difference between this transformation and the previous one is smaller than the threshold,  refine the process by reducing the maximal correspondence distance
		if (fabs ((mPcJoiner.getLastIncrementalTransformation () - prev).sum ()) < mPcJoiner.getTransformationEpsilon ())
			mPcJoiner.setMaxCorrespondenceDistance (mPcJoiner.getMaxCorrespondenceDistance () - mParams.icpMaxCorrDistDownStep);

		prev = mPcJoiner.getLastIncrementalTransformation ();
	}

	cout << "Time for alignment " << t << endl;

	cout << "Fitness score " << mPcJoiner.getFitnessScore() << "   Has conveged? " << mPcJoiner.hasConverged() << endl;

	// Get the transformation from target to source
	targetToSource = Ti;
	mPreviousCloud2MapTransformation = targetToSource;
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
