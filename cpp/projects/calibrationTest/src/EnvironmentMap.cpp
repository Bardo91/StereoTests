//
//
//
//
//

#include "EnvironmentMap.h"
#include "Gui.h"

#include <opencv2/opencv.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common.h>

using namespace pcl;
using namespace std;
using namespace Eigen;

//---------------------------------------------------------------------------------------------------------------------
EnvironmentMap::EnvironmentMap() {

}

//---------------------------------------------------------------------------------------------------------------------
EnvironmentMap::EnvironmentMap(EnvironmentMap::Params _params) {
	mParams = _params;
	params(mParams);
}

//---------------------------------------------------------------------------------------------------------------------
void EnvironmentMap::params(EnvironmentMap::Params _params) {
	mParams = _params;

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
EnvironmentMap::Params EnvironmentMap::params() const {
	return mParams;
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
// you can only use one type of history calculaton in the application, because they use the same members which need to be caluclated correctly in previous steps
void EnvironmentMap::addPoints(const PointCloud<PointXYZ>::Ptr &_cloud, enum eHistoryCalculation _calculation) {
	switch (_calculation) {
		case eHistoryCalculation::Simple:
			addPointsSimple(_cloud);
			break;
		case eHistoryCalculation::Accurate:
			addPointsAccurate(_cloud);
			break;
		case eHistoryCalculation::Sequential:
			addPointsSequential(_cloud);
			break;
		default:
			cout << "Wrong argument for addPoints()" << endl;
			break;
	}
}

//---------------------------------------------------------------------------------------------------------------------
void EnvironmentMap::addPointsSimple(const PointCloud<PointXYZ>::Ptr & _cloud) {
	if (mCloud.size() == 0) {
		if (mCloudHistory.size() == 0) {
			// Store First cloud as reference
			cout << "This is the first point cloud, no map yet, adding to history" << endl;
			PointCloud<PointXYZ>::Ptr firstCloud = voxel(filter(_cloud));
			firstCloud->sensor_origin_ = Vector4f(0,0,0,1);
			firstCloud->sensor_orientation_ = Quaternionf::Identity();
			mCloudHistory.push_back(firstCloud);
		}
		// transform clouds to history until there are enough to make first map from history
		else if (mCloudHistory.size() < mParams.historySize) {
			printf("This is point cloud Nr. %d of %d needed for map.\n", mCloudHistory.size() + 1, mParams.historySize);
			transformCloudtoTargetCloudAndAddToHistory(_cloud, mCloudHistory[0], transformationFromSensor(mCloudHistory.back()));
		}
	}
	else {
		// Storing and processing history of point clouds.
		//temporary cleaned cloud for calculation of the transformation. We do not want to voxel in the camera coordinate system
		//because we lose some points when rotating it to the map and voxeling there. That's why we rotate the original cloud
		transformCloudtoTargetCloudAndAddToHistory(_cloud, mCloud.makeShared(), transformationFromSensor(mCloudHistory.back()));
	}

	addOrientationAndOriginDataToMap(mCloudHistory.back());

	if (mCloudHistory.size() >= mParams.historySize) {
		cout << "Map extended" << endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr	lastJoinedCloud;
		lastJoinedCloud = convoluteCloudsInQueue(mCloudHistory).makeShared();
		mCloud += *lastJoinedCloud;
		mCloud = *voxel(mCloud.makeShared());
		// Finally discard oldest cloud
		mCloudHistory.pop_front();
		// drawing of the last point cloud that has been added to the map.
		PointCloud<PointXYZ> PointCloudCameraCS;
		transformPointCloud(*lastJoinedCloud, PointCloudCameraCS, originInverse(mCloud.makeShared()),sensorInverse(mCloud.makeShared()));
		Gui::get()->addPointToPcViewer(PointCloudCameraCS.makeShared(), 3, 255, 10, 10);	
	}
}
//---------------------------------------------------------------------------------------------------------------------
void EnvironmentMap::addPointsSequential(const PointCloud<PointXYZ>::Ptr & _cloud) {
	// Store First cloud as reference
	// 666 we store the actual first cloud instead of considering history, this means we accept the noise from the first cloud
	if (mCloud.size() == 0) {
		mCloud += *voxel(filter(_cloud));
		//mCloud.sensor_origin_ = Vector4f(0, 0, 0);
		//mCloud.sensor_orientation_ = Quaternionf(1, 0, 0, 0);
	}

	// Storing and processing history of point clouds.
	//temporary cleaned cloud for calculation of the transformation. We do not want to voxel in the camera coordinate system
	//becase we lose some points when rotating it to the map and voxeling there. That's why we rotate the original cloud
	PointCloud<PointXYZ>::Ptr filtered_cloud = filter(_cloud);
	PointCloud<PointXYZ> filtered_cloudWCS;
	Matrix4f transformation = getTransformationBetweenPcs(*voxel(filtered_cloud), mCloud, transformationFromSensor(mCloud.makeShared())); //666 mPreviousCloud2MapTransformation needs to be from cloudHistory.orientation
	transformPointCloud(*filtered_cloud, filtered_cloudWCS, transformation);
	PointCloud<PointXYZ>::Ptr voxeledFiltered_cloudWCS = voxel(filtered_cloudWCS.makeShared());
	voxeledFiltered_cloudWCS->sensor_orientation_ = Quaternionf(transformation.block<3, 3>(0, 0));
	voxeledFiltered_cloudWCS->sensor_origin_ = transformation.col(3);
	mCloudHistory.push_back(voxeledFiltered_cloudWCS);
	//666 fix this, probably not needed anymore, because it should be in mCloudHistory


	if (mCloudHistory.size() >= mParams.historySize) {
		// Get first pointcloud
		PointCloud<PointXYZ> convolutedSum = *mCloudHistory[0];

		for (unsigned i = 1; i < mParams.historySize; i++)
			// Now we consider only 2 clouds on history, if want to increase it, need to define points with probabilities.
			convolutedSum = convoluteCloudsOnGrid(convolutedSum, *mCloudHistory[i]);

		mCloud += convolutedSum;
		mCloud = *voxel(mCloud.makeShared());
		// Finally discart oldest cloud
		mCloudHistory.pop_front();
	}
}

//---------------------------------------------------------------------------------------------------------------------
void EnvironmentMap::addPointsAccurate(const PointCloud<PointXYZ>::Ptr & _cloud) {
	// Store First cloud as reference
	// 666 we store the actual first cloud instead of considering history, this means we accept the noise from the first cloud
	if (mCloud.size() == 0) {
		mCloud += *voxel(filter(_cloud));
		//mCloud.sensor_origin_ = Vector4f(0, 0, 0);
		//mCloud.sensor_orientation_ = Quaternionf(1, 0, 0, 0);
	}

	// Storing and processing history of point clouds.
	//temporary cleaned cloud for calculation of the transformation. We do not want to voxel in the camera coordinate system
	//becase we lose some points when rotating it to the map and voxeling there. That's why we rotate the original cloud
	PointCloud<PointXYZ>::Ptr filtered_cloud = filter(_cloud);
	PointCloud<PointXYZ> filtered_cloudWCS;
	Matrix4f transformation = getTransformationBetweenPcs(*voxel(filtered_cloud), mCloud, transformationFromSensor(mCloud.makeShared())); //666 mPreviousCloud2MapTransformation needs to be from cloudHistory.orientation
	transformPointCloud(*filtered_cloud, filtered_cloudWCS, transformation);
	PointCloud<PointXYZ>::Ptr voxeledFiltered_cloudWCS = voxel(filtered_cloudWCS.makeShared());
	voxeledFiltered_cloudWCS->sensor_orientation_ = Quaternionf(transformation.block<3, 3>(0, 0));
	voxeledFiltered_cloudWCS->sensor_origin_ = transformation.col(3);
	mCloudHistory.push_back(voxeledFiltered_cloudWCS);
	//666 fix this, probably not needed anymore, because it should be in mCloudHistory


	if (mCloudHistory.size() >= mParams.historySize) {
		// Get first pointcloud
		PointCloud<PointXYZ> convolutedSum = *mCloudHistory[0];

		for (unsigned i = 1; i < mParams.historySize; i++)
			// Now we consider only 2 clouds on history, if want to increase it, need to define points with probabilities.
			convolutedSum = convoluteCloudsOnGrid(convolutedSum, *mCloudHistory[i]);



		mCloud += convolutedSum;
		mCloud = *voxel(mCloud.makeShared());
		
		// Store last position of the camera.
		mCloud.sensor_orientation_ = Quaternionf(transformation.block<3, 3>(0, 0));
		mCloud.sensor_origin_ = transformation.col(3);

		// Finally discart oldest cloud
		mCloudHistory.pop_front();
	}
}

void EnvironmentMap::transformCloudtoTargetCloudAndAddToHistory(const PointCloud<PointXYZ>::Ptr & _cloud, const PointCloud<PointXYZ>::Ptr & _target, const Matrix4f &_guess)
{
	PointCloud<PointXYZ>::Ptr filtered_cloud = filter(_cloud);
	PointCloud<PointXYZ> filtered_cloudWCS;
	Matrix4f transformation = getTransformationBetweenPcs(*voxel(filtered_cloud), *_target, _guess); //666 mPreviousCloud2MapTransformation needs to be from cloudHistory.orientation
	transformPointCloud(*filtered_cloud, filtered_cloudWCS, transformation);
	PointCloud<PointXYZ>::Ptr voxeledFiltered_cloudWCS = voxel(filtered_cloudWCS.makeShared());
	voxeledFiltered_cloudWCS->sensor_orientation_ = Quaternionf(transformation.block<3, 3>(0, 0));
	voxeledFiltered_cloudWCS->sensor_origin_ = transformation.col(3);
	mCloudHistory.push_back(voxeledFiltered_cloudWCS);
}

pcl::PointCloud<pcl::PointXYZ> EnvironmentMap::convoluteCloudsInQueue(std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> _cloudQueue)
{
	PointCloud<PointXYZ> convolutedSum = *_cloudQueue[0];
	for (unsigned i = 1; i < _cloudQueue.size(); i++)
		convolutedSum = convoluteCloudsOnGrid(convolutedSum, *_cloudQueue[i]);
	return convolutedSum;
}

void EnvironmentMap::addOrientationAndOriginDataToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr & _cloud)
{
	mCloud.sensor_orientation_ = _cloud->sensor_orientation_;
	mCloud.sensor_origin_ = _cloud->sensor_origin_;
}

Eigen::Vector3f EnvironmentMap::originInverse(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud)
{
	//Vector4f output = -(_cloud->sensor_orientation_.inverse()*_cloud->sensor_origin_);
	Vector3f output = -(_cloud->sensor_orientation_.inverse()*_cloud->sensor_origin_.block<3, 1>(0, 0));
	return output;
}

Eigen::Quaternionf EnvironmentMap::sensorInverse(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud)
{
	//return _cloud->sensor_orientation_.inverse();
	return _cloud->sensor_orientation_.conjugate();
}

//---------------------------------------------------------------------------------------------------------------------
PointCloud<PointXYZ>::Ptr EnvironmentMap::voxel(const PointCloud<PointXYZ>::Ptr &_cloud) {
	mVoxelGrid.setInputCloud(_cloud);
	PointCloud<PointXYZ>::Ptr voxeled(new PointCloud<PointXYZ>);
	mVoxelGrid.filter(*voxeled);
	return voxeled;
}

Eigen::Matrix4f EnvironmentMap::transformationFromSensor(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud)
{
	Matrix4f output = Matrix4f::Identity();
	output.col(3) = _cloud->sensor_origin_;
	output.block<3, 3>(0, 0) = _cloud->sensor_orientation_.matrix();
	return output;
}

//---------------------------------------------------------------------------------------------------------------------
vector<PointIndices> EnvironmentMap::clusterCloud(const PointCloud<PointXYZ>::Ptr &_cloud) {
	// Creating the KdTree object for the search method of the extraction
	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
	tree->setInputCloud(_cloud);
	//vector of indices of each cluster
	vector<PointIndices> clusterIndices;
	mEuclideanClusterExtraction.setSearchMethod(tree);
	mEuclideanClusterExtraction.setInputCloud(_cloud);
	mEuclideanClusterExtraction.extract(clusterIndices);
	return clusterIndices;
}

//---------------------------------------------------------------------------------------------------------------------
vector<PointIndices> EnvironmentMap::clusterCloud(const PointCloud<PointXYZ>::Ptr &_cloud, vector<PointCloud<PointXYZ>::Ptr> &_clusters) {
	vector<PointIndices> clusterIndices = clusterCloud(_cloud);
	for (vector<PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
		PointCloud<PointXYZ>::Ptr cloudCluster(new PointCloud<PointXYZ>);
		for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
			cloudCluster->points.push_back(_cloud->points[*pit]);
		}
		cloudCluster->width = cloudCluster->points.size();
		cloudCluster->height = 1;
		cloudCluster->is_dense = true;
		_clusters.push_back(cloudCluster);
	}
	return clusterIndices;
}

//---------------------------------------------------------------------------------------------------------------------
PointCloud<PointXYZ> EnvironmentMap::cloud() {
	return mCloud;
}

//---------------------------------------------------------------------------------------------------------------------
ModelCoefficients  EnvironmentMap::extractFloor(const PointCloud<PointXYZ>::Ptr &_cloud) {
	vector<PointCloud<PointXYZ>::Ptr> clusters;
	clusterCloud(_cloud, clusters);

	if(clusters.size() < 4)
		return ModelCoefficients();

	PointCloud<PointXYZ> farthestPoints;
	Eigen::Vector4f pivotPt;
	pivotPt << 0,0,0,1;
	for (PointCloud<PointXYZ>::Ptr cluster: clusters) {
		Eigen::Vector4f maxPt;
		getMaxDistance(*cluster, pivotPt, maxPt);
		PointXYZ point(maxPt(0), maxPt(1), maxPt(2));
		farthestPoints.push_back(point);
	}

	ModelCoefficients::Ptr coefficients (new ModelCoefficients);
	PointIndices::Ptr inliers (new PointIndices);
	SACSegmentation<PointXYZ> seg;
	seg.setOptimizeCoefficients (true);
	seg.setModelType (SACMODEL_PLANE);
	seg.setMethodType (SAC_RANSAC);
	seg.setDistanceThreshold (0.1);
	seg.setInputCloud (farthestPoints.makeShared());
	seg.segment (*inliers, *coefficients);

	return *coefficients;
}

//---------------------------------------------------------------------------------------------------------------------
double EnvironmentMap::distanceToPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &_cloud, const pcl::ModelCoefficients &_plane) {
	
	double minDist = 999999;
	for (PointXYZ point : *_cloud) {
		double dist =	abs(_plane.values[0]*point.x +
							_plane.values[1]*point.y +
							_plane.values[2]*point.z +
							_plane.values[3]) / 
							sqrt(	pow(_plane.values[0], 2) +
									pow(_plane.values[1], 2) +
									pow(_plane.values[2], 2));
				
		if(dist < minDist)
			minDist = dist;
	}
	
	return minDist;
}

//---------------------------------------------------------------------------------------------------------------------
void EnvironmentMap::cropCloud(PointCloud<PointXYZ>::Ptr &_cloud, ModelCoefficients _plane, bool _upperSide) {
	if (_cloud->size() == 0 || _plane.values.size() != 4)
		return;

	auto predicate = [&](const PointXYZ &_point) {
		double val = (-_plane.values[0] * _point.x - _plane.values[1] * _point.y - _plane.values[3])/_plane.values[2];

		if(_upperSide)
			return _point.z > val? true:false;
		else
			return _point.z > val? false:true;
	};

	_cloud->erase( std::remove_if(_cloud->begin(), _cloud->end(), predicate ), _cloud->end());
}

//---------------------------------------------------------------------------------------------------------------------
Matrix4f EnvironmentMap::getTransformationBetweenPcs(const PointCloud<PointXYZ>& _newCloud, const PointCloud<PointXYZ>& _targetCloud, const Eigen::Matrix4f &_initialGuess, PointCloud<PointXYZ> &_alignedCloud) {
	BOViL::STime *timer = BOViL::STime::get();
	double t;

	mPcJoiner.setInputSource(_newCloud.makeShared());
	mPcJoiner.setInputTarget(_targetCloud.makeShared());

	double t0 = timer->getTime();
	mPcJoiner.align(_alignedCloud, _initialGuess);
	t = timer->getTime() - t0;

	cout << "Time for alignment " << t << endl;
	cout << "Fitness score " << mPcJoiner.getFitnessScore() << "   Has conveged? " << mPcJoiner.hasConverged() << endl;

	return mPcJoiner.getFinalTransformation();
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
			outCloud.push_back(point);
		}
	}
	cout << "Size of first: " << _cloud1.size() << endl;
	cout << "Size of second: " << _cloud2.size() << endl;
	cout << "Size of result: " << outCloud.size() << endl;
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

	cout << "Rotations: " << roll << ", " << pitch << ", " << yaw << endl;
	cout << "Translations: " << translation(0) << ", " << translation(1) << ", " << translation(2) << endl;

	if (abs(roll) < cMaxAngle && abs(pitch) < cMaxAngle && abs(yaw) < cMaxAngle  &&
		abs(translation(0)) < cMaxTranslation && abs(translation(1)) < cMaxTranslation && abs(translation(2)) < cMaxTranslation) {
		cout << "Valid point cloud rotation" << endl;
		return true;
	} else {
		cout << "Invalid point cloud rotation" << endl;
		return false;
	}
}

