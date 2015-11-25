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
ObjectCandidate::ObjectCandidate(PointIndices _pointIndices, PointCloud<PointXYZ>::Ptr _cloud)
{
	mPointIndices = _pointIndices;
	mCloud = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
	for (int index : _pointIndices.indices)
		mCloud->push_back(_cloud->points[index]);
	mCloud->width = mCloud->points.size();
	mCloud->height = 1;
	mCloud->is_dense = true;
	computeCentroid();
	mR = rand() * 255 / RAND_MAX;
	mG = rand() * 255 / RAND_MAX;
	mB = rand() * 255 / RAND_MAX;

}

//---------------------------------------------------------------------------------------------------------------------
ObjectCandidate::ObjectCandidate(pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud) {
	mCloud =  PointCloud<PointXYZ>::Ptr(_cloud);
	mR = rand() * 255 / RAND_MAX; 
	mG = rand() * 255 / RAND_MAX; 
	mB = rand() * 255 / RAND_MAX;
	computeCentroid();
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

Eigen::Vector4f ObjectCandidate::centroid() const{
	return mCentroid;
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

	return pair<int, double>(maxIndex, maxProb);
}

void ObjectCandidate::matchSequentialCandidates(vector<ObjectCandidate> &_globalCandidates, vector<ObjectCandidate> &_newCandidates, float _threshold)
{
	//in the first step _globalCandidates is empty, which is handled in else
	if (_globalCandidates.size() != 0) {
		vector<pair<int, float>> matchIndexDist = matchCandidates(_newCandidates, _globalCandidates, _threshold);

		// 666 here I need to take care if 2 new candidates match with the same global candidate
		for (int i = 0; i < _newCandidates.size(); i++) {
			int match = matchIndexDist[i].first;
			if (match == -1) {
				cout << "No match found, distance to closest is: " << matchIndexDist[i].second << " adding new candidate" << endl;
				_globalCandidates.push_back(_newCandidates[i]);
			}
			else {
				_globalCandidates[match].update(_newCandidates[i]);
				cout << i << ":found match with " << match << " distance is " << matchIndexDist[i].second << endl;
			}
		}
	} 
	else{	
		_globalCandidates = _newCandidates;
	}
}

std::vector<std::pair<int, float>> ObjectCandidate::matchCandidates(vector<ObjectCandidate> & _querryCandidate, vector<ObjectCandidate> & _targetCandidate, float _threshold)
{
	vector<pair<int, float>> matchIndexDist;
	for (ObjectCandidate newCandidate : _querryCandidate) {
		Eigen::Vector4f cent = newCandidate.centroid();
		vector<float> distances;
		for (ObjectCandidate candidate : _targetCandidate) {
			distances.push_back((cent - candidate.centroid()).norm());
		}
		vector<float>::iterator it = min_element(distances.begin(), distances.end());
		int index = it - distances.begin();
		if (*it < _threshold)
			matchIndexDist.push_back(pair<int, float>(index, *it));
		else
			matchIndexDist.push_back(pair<int, float>(-1, *it));
	}
	return matchIndexDist;
}

void ObjectCandidate::computeCentroid()
{
	Eigen::Vector4f temp;
	compute3DCentroid(*mCloud, temp);
	mCentroid = temp; // PointXYZ(temp[0], temp[1], temp[2]);
}

void ObjectCandidate::update(ObjectCandidate & _nextInstance)
{
	mPointIndices = _nextInstance.mPointIndices;
	mCloud = _nextInstance.mCloud;
	computeCentroid();
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