///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Single object detection - ObjectDetector
//		Author: Pablo R.S.
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef _ALGORITHM_SLIDINGWINDOW_H_
#define _ALGORITHM_SLIDINGWINDOW_H_

#include <vector>
#include <opencv2/opencv.hpp>

namespace algorithm {
	/// Implementation of sliding window extractor.
	class SlidingWindow {
	public:		//	Public interface
				/// Parameters for sliding window.
		struct Params {
			int wHeight;		/// Window's height.
			int wWidth;			/// Window's width.
			int vStep;			/// Vertical step.
			int hStep;			/// Horizontal step.
			int scaleSteps;		///	Number of scale steps (1 means 1 scale, i.e., do not scale image anytime).
			float scaleFactor;	/// Scale factor for every step.
		};

		/// Sliding window constructor.
		/// \params _params: parameters for sliding window method.
		SlidingWindow(Params _params);

		/// Set new params to method.
		/// \params _params: parameters for sliding window method.
		void params(Params _params);

		/// Get current params.
		/// \return  current params of method.
		Params params() const;

		/// Extract windows from images using current params.
		/// \return list of windows extracted from the image.
		std::vector<cv::Mat> subwindows(const cv::Mat &_image);

		/// Get rectangles
		std::vector<cv::Rect> grid(const cv::Mat &_image) const;
	private:	//	Members
		Params mParams;
	};	//	class SlidingWindow
}	//	namesapce algorithm.


#endif	//	_ALGORITHM_SLIDINGWINDOW_H_