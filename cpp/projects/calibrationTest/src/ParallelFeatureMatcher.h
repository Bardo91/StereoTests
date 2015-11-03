//
//
//
//
//


#ifndef PARALLELFEATUREMATCHER_H_
#define PARALLELFEATUREMATCHER_H_

#include <opencv2/opencv.hpp>
#include <vector>

class ParallelFeatureMatcher: public cv::ParallelLoopBody{
public:
	ParallelFeatureMatcher(const cv::Mat &_frame1, const cv::Mat &_frame2, const std::vector<cv::Point2i> &_kps, const std::vector<cv::Vec3f> &_epis, const std::pair<int, int> &_disparityRange, const int &_squareSize, std::vector<std::vector<cv::Point2i>> &_points1, std::vector<std::vector<cv::Point2i>> &_points2, cv::Rect _vl, cv::Rect _vr);
	virtual void operator()(const cv::Range& range) const;

private:
	cv::Point2i findMatch(const cv::Mat &_frame1, const cv::Mat &_frame2, const cv::Point2i &_point, const cv::Vec3f &_epiline, const std::pair<int, int> _disparityRange, const int _squareSize = 11) const;

private:
	const cv::Mat &frame1, &frame2;
	const std::vector<cv::Point2i> &kps;
	const std::vector<cv::Vec3f> &epis;
	const std::pair<int, int> &disparityRange;
	const int &squareSize;
	std::vector<std::vector<cv::Point2i>> &points1;
	std::vector<std::vector<cv::Point2i>> &points2;
	cv::Rect validLeft;
	cv::Rect validRight;
};


#endif	//	PARALLELFEATUREMATCHER_H_