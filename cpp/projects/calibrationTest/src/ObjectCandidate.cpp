//
//
//
//
//

#include "ObjectCandidate.h"


using namespace pcl;
using namespace cv;
using namespace std;

ObjectCandidate::ObjectCandidate(PointIndices _pointIndices, PointCloud<PointXYZ>::Ptr _cloud, bool _copyCloudPoints = false)
{
	mPointIndices = _pointIndices;
	mCloud =  PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
	if (_copyCloudPoints) {
		for (int index : _pointIndices.indices)
			mCloud->push_back(_cloud->points[index]);
		mCloud->width = mCloud->points.size();
		mCloud->height = 1;
		mCloud->is_dense = true;
	}
	mR = rand() * 255 / RAND_MAX; 
	mG = rand() * 255 / RAND_MAX; 
	mB = rand() * 255 / RAND_MAX;

}


ObjectCandidate::~ObjectCandidate()
{
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ObjectCandidate::cloud() const
{
	return mCloud;
}

unsigned ObjectCandidate::R() const { return mR; }

unsigned ObjectCandidate::G() const { return mG; }

unsigned ObjectCandidate::B() const { return mB; }
