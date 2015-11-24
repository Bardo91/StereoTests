//
//
//
//
//


#ifndef OBJECTCANDIDATE_H_
#define OBJECTCANDIDATE_H_

#include <pcl/PointIndices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <opencv2/opencv.hpp>

class ObjectCandidate
{
public:
	ObjectCandidate(pcl::PointIndices _pointIndeces, pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud);
	~ObjectCandidate();

	/// Add a view to it's history
	void addView(cv::Mat _view);

	/// Get point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud() const;

	/// Get centroid
	Eigen::Vector4f centroid() const;

	unsigned R() const;
	unsigned G() const;
	unsigned B() const;


	void addView(cv::Mat _view, std::vector<double> _cathegories);
	std::pair<unsigned, float>  cathegory() const;

 	static void matchSequentialCandidates(std::vector<ObjectCandidate> &_globalCandidates, std::vector<ObjectCandidate> &_newCandidates);
 
private:
	void computeCentroid();
	void update(ObjectCandidate &_nextInstance);
	pcl::PointIndices mPointIndices;
	pcl::PointCloud<pcl::PointXYZ>::Ptr mCloud;
	Eigen::Matrix<float,4,1,Eigen::DontAlign> mCentroid;
	unsigned mR, mG, mB;

	std::vector<cv::Mat> mViewHistory;
	std::vector<std::vector<double>> mCathegoryHistory;
};

#endif
