///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-10-01
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

using namespace std;
using namespace cv;

int main(int _argc, char** _argv){
	string leftPath = "C:/Users/Pablo RS/ownCloud/Datasets/StereoTesting/testImages/img_cam1_2.jpg";
	string rightPath = "C:/Users/Pablo RS/ownCloud/Datasets/StereoTesting/testImages/img_cam2_2.jpg";
	Mat left  = imread(leftPath ,IMREAD_COLOR);
	if ( left.empty() )
	{
		return -1;
	}
	Mat right = imread(rightPath,IMREAD_COLOR);
	if ( right.empty() )
	{
		return -1;
	}

	Mat oriDisplay;
	hconcat(left, right, oriDisplay);
	imshow("ori", oriDisplay);


	int max_disp = 16;
	int wsize = 15;

	Mat left_for_matcher, right_for_matcher;

	max_disp/=2;
	if(max_disp%16!=0)
		max_disp += 16-(max_disp%16);
	resize(left ,left_for_matcher ,Size(),0.5,0.5);
	resize(right,right_for_matcher,Size(),0.5,0.5);

	Ptr<StereoBM> left_matcher = StereoBM::create(max_disp,wsize);
	Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
	Ptr<StereoMatcher> right_matcher = cv::ximgproc::createRightMatcher(left_matcher);
	cvtColor(left_for_matcher,  left_for_matcher,  COLOR_BGR2GRAY);
	cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);
	double matching_time = (double)getTickCount();

	Mat left_disp, right_disp;
	left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
	right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
	matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();

	double lambda = 8000.0;
	double sigma = 2.0;
	wls_filter->setLambda(lambda);
	wls_filter->setSigmaColor(sigma);
	double filtering_time = (double)getTickCount();
	Mat filtered_disp;
	wls_filter->filter(left_disp,left,filtered_disp,right_disp);
	filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();

	Mat raw_disp_vis;
	cv::ximgproc::getDisparityVis(left_disp,raw_disp_vis);
	namedWindow("raw disparity", WINDOW_AUTOSIZE);
	imshow("raw disparity", raw_disp_vis);
	Mat filtered_disp_vis;
	cv::ximgproc::getDisparityVis(filtered_disp,filtered_disp_vis);
	namedWindow("filtered disparity", WINDOW_AUTOSIZE);
	imshow("filtered disparity", filtered_disp_vis);
	waitKey();

}
