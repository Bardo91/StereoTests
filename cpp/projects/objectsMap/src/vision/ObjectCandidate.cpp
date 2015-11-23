//
//
//
//
//

#include "ObjectCandidate.h"
#include <numeric>

using namespace pcl;
using namespace cv;
using namespace std;

//---------------------------------------------------------------------------------------------------------------------
// Public Interface
//---------------------------------------------------------------------------------------------------------------------
ObjectCandidate::ObjectCandidate(PointIndices _pointIndices, PointCloud<PointXYZ>::Ptr _cloud, bool _copyCloudPoints = false) {
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
void ObjectCandidate::addView(cv::Mat _view, std::vector<double> _probs) {
	mViewHistory.push_back(_view);
	
	if (mLabelsHistory.size() == 0) {
		mLabelsHistory.resize(_probs.size());
	} 

	for (unsigned i = 0 ; i < mLabelsHistory.size() ; i++) {
		mLabelsHistory[i].push_back(_probs[i]);
	}
	
	calculateLabelsStatistics();
}

//---------------------------------------------------------------------------------------------------------------------
std::pair<int, double> ObjectCandidate::cathegory() const {
	if (mLabelsHistory.size() == 0) {
		return pair<int, double>(9999,0);
	}

	int maxIndex;
	double maxProb = 0;
	for (unsigned i = 0; i < mLabelsStatistics.size(); i++) {
		if (mLabelsStatistics[i].first > maxProb) {
			maxIndex = i;
			maxProb = mLabelsStatistics[i].first;
		}
	}

	return pair<unsigned, float>(maxIndex, maxProb);
}

//---------------------------------------------------------------------------------------------------------------------
// Private Interface
//---------------------------------------------------------------------------------------------------------------------
void ObjectCandidate::calculateLabelsStatistics() {
	if (mLabelsStatistics.size() == 0) {
		mLabelsStatistics.resize(mLabelsHistory.size());
	}

	for (unsigned i = 0; i < mLabelsHistory.size(); i++) {
		int nSamples = mLabelsHistory[i].size();
		double mean = 0;
		for (unsigned j = 0; j < mLabelsHistory[i].size();j++) {
			mean += mLabelsHistory[i][j];
		}
		mean /= mLabelsHistory[i].size();
		double stdDev = 0;
		for (unsigned j = 0; j < mLabelsHistory[i].size(); j++) {
			double diff = mLabelsHistory[i][j] - mean;
			stdDev += pow(diff, 2);
		}
		stdDev = sqrt(stdDev/nSamples);

		mLabelsStatistics[i].first = mean;
		mLabelsStatistics[i].second = stdDev;
	}
}