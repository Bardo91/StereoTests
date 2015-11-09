//
//
//
//
//

#include "ObjectCandidate.h"


using namespace pcl;
using namespace cv;
using namespace std;

//---------------------------------------------------------------------------------------------------------------------
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

//---------------------------------------------------------------------------------------------------------------------
ObjectCandidate::~ObjectCandidate() {

}

//---------------------------------------------------------------------------------------------------------------------
void ObjectCandidate::addView(cv::Mat _view) {
	mViewHistory.push_back(_view);
}

//---------------------------------------------------------------------------------------------------------------------
pcl::PointCloud<pcl::PointXYZ>::Ptr ObjectCandidate::cloud() const {
	return mCloud;
}

//---------------------------------------------------------------------------------------------------------------------
unsigned ObjectCandidate::R() const { return mR; }

//---------------------------------------------------------------------------------------------------------------------
unsigned ObjectCandidate::G() const { return mG; }

//---------------------------------------------------------------------------------------------------------------------
unsigned ObjectCandidate::B() const { return mB; }

//---------------------------------------------------------------------------------------------------------------------
void ObjectCandidate::addView(cv::Mat _view, std::vector<std::pair<unsigned, float>> _cathegories) {
	mViewHistory.push_back(_view);
	mCathegoryHistory.push_back(_cathegories);
}

//---------------------------------------------------------------------------------------------------------------------
std::pair<unsigned, float> ObjectCandidate::cathegory() const {
	if (mCathegoryHistory.size() == 0) {
		return pair<unsigned, float>(9999,0);
	}

	vector<pair<unsigned,float>> lastData = mCathegoryHistory[mCathegoryHistory.size()-1];

	double maxProb=0;
	int maxIndex;
	for (unsigned i = 0; i < lastData.size(); i++) {
		if (lastData[i].second > maxProb) {
			maxIndex = i;
			maxProb = lastData[i].second;
		}
	}

	return lastData[maxIndex];
}
