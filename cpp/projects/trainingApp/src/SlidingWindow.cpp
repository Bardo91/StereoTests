#include "SlidingWindow.h"
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Single object detection - ObjectDetector
//		Author: Pablo R.S.
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

using namespace cv;
using namespace std;

namespace algorithm {
	//-----------------------------------------------------------------------------------------------------------------
	SlidingWindow::SlidingWindow(Params _params) : mParams(_params) {

	}

	//-----------------------------------------------------------------------------------------------------------------
	void SlidingWindow::params(Params _params) {
		mParams = _params;
	}

	//-----------------------------------------------------------------------------------------------------------------
	SlidingWindow::Params SlidingWindow::params() const {
		return mParams;
	}

	//-----------------------------------------------------------------------------------------------------------------
	std::vector<cv::Mat> SlidingWindow::subwindows(const cv::Mat &_image) {
		assert(_image.channels() == 1 && _image.rows != 0);
		std::vector<cv::Mat> windows;

		float scaleFactor = 1;	// Scale factor for each iteration
		for (int k = 0; k < mParams.scaleSteps; k++) {
			// Resize image
			Mat scaledImg;
			_image.copyTo(scaledImg);
			resize(_image, scaledImg, Size(), scaleFactor, scaleFactor);

			// Compute number of windows in vertical and horizontal directions.
			int vImgs = (scaledImg.rows - mParams.wHeight) / mParams.vStep + 1;
			int hImgs = (scaledImg.cols - mParams.wWidth) / mParams.hStep + 1;

			// Stop scaling if image is smaller than window.
			if (scaledImg.rows < mParams.wHeight || scaledImg.cols < mParams.wWidth) {
				break;
			}

			for (int i = 0; i < vImgs; i++) {
				for (int j = 0; j < hImgs; j++) {
					Mat subWnd = scaledImg.rowRange(i*mParams.vStep, i*mParams.vStep + mParams.wHeight)
						.colRange(j*mParams.hStep, j*mParams.hStep + mParams.wWidth);
					windows.push_back(subWnd);
				}
			}

			scaleFactor *= mParams.scaleFactor;
		}

		return windows;
	}
	std::vector<cv::Rect> SlidingWindow::grid(const cv::Mat &_image) const {
		vector<Rect>  grid;

		float scaleFactor = 1;	// Scale factor for each iteration
		for (int k = 0; k < mParams.scaleSteps; k++) {
			// Compute number of windows in vertical and horizontal directions.
			int vImgs = (_image.rows*scaleFactor - mParams.wHeight) / mParams.vStep + 1;
			int hImgs = (_image.cols*scaleFactor - mParams.wWidth) / mParams.hStep + 1;


			for (int i = 0; i < vImgs; i++) {
				for (int j = 0; j < hImgs; j++) {
					Rect2i rec(j*mParams.hStep / scaleFactor, i*mParams.vStep / scaleFactor, mParams.wWidth / scaleFactor, mParams.wHeight / scaleFactor);
					grid.push_back(rec);
				}
			}

			scaleFactor *= mParams.scaleFactor;
		}

		return grid;
	}
}	//	namesapce algorithm.