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
void ObjectCandidate::addView(cv::Mat _view, std::vector<double> probs) {
	mViewHistory.push_back(_view);
	mCathegoryHistory.push_back(probs);
}

//---------------------------------------------------------------------------------------------------------------------
std::pair<unsigned, float> ObjectCandidate::cathegory() const {
	if (mCathegoryHistory.size() == 0) {
		return pair<unsigned, float>(9999,0);
	}

	std::vector<double> lastData = mCathegoryHistory[mCathegoryHistory.size()-1];

	double maxProb=0;
	int maxIndex;
	for (unsigned i = 0; i < lastData.size(); i++) {
		if (lastData[i] > maxProb) {
			maxIndex = i;
			maxProb = lastData[i];
		}
	}

	return pair<unsigned, float>(maxIndex, lastData[maxIndex]);
}

void ObjectCandidate::matchSequentialCandidates(vector<ObjectCandidate> &_globalCandidates, vector<ObjectCandidate> &_newCandidates)
{
	//in the first step _globalCandidates is empty, which is handled in else
	if (_globalCandidates.size() != 0) {
		float threshold = 0.2;
		vector<int> matchIndex;
		vector<float> matchDistance;
		for (ObjectCandidate newCandidate:_newCandidates) {
			Eigen::Vector4f cent = newCandidate.centroid();
			vector<float> distances;
			for (ObjectCandidate candidate : _globalCandidates) {
				distances.push_back((cent - candidate.centroid()).norm());
			}
			 vector<float>::iterator it = min_element(distances.begin(), distances.end());
			 int index = it - distances.begin();
				 matchDistance.push_back(*it);
			 if (*it < threshold) 
				 matchIndex.push_back(index);
			 else 
				 matchIndex.push_back(-1);
		}
		//here I need to take care if 2 global candidates match with the same new candidate
		for (int i = 0; i < _newCandidates.size(); i++) {
			int match = matchIndex[i];
			if (match == -1) {
				_globalCandidates.push_back(_newCandidates[i]);
			}
			else {
				_globalCandidates[match].update(_newCandidates[i]);
			}
		}
	} 
	else{	
		_globalCandidates = _newCandidates;
	}
}

void ObjectCandidate::computeCentroid()
{
	compute3DCentroid(*mCloud, mCentroid);
}

void ObjectCandidate::update(ObjectCandidate & _nextInstance)
{
	mPointIndices = _nextInstance.mPointIndices;
	mCloud = _nextInstance.mCloud;
	computeCentroid();
}
