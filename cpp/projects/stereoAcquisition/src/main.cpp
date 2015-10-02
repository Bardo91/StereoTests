///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-10-02
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <opencv2/opencv.hpp>
using namespace cv;


Mat frame1, frame2;
unsigned index = 3;
void mouseCallback(int event, int x, int y, int, void* ) {
	if( event != EVENT_LBUTTONDOWN )
		return;
	
	imwrite("C:/Users/Pablo RS/ownCloud/Datasets/StereoTesting/testImages/img_cam1_" + std::to_string(index) + ".jpg", frame1);
	imwrite("C:/Users/Pablo RS/ownCloud/Datasets/StereoTesting/testImages/img_cam2_" + std::to_string(index) + ".jpg", frame2);
	
	index++;
	
}

//---------------------------------------------------------------------------------------------------------------------
int main(int _argc, char ** _argv) {
	VideoCapture camera1(0);
	VideoCapture camera2(1);
	
	namedWindow("display1");
	setMouseCallback("display1", mouseCallback);
	
	for (;;) {
		camera1 >> frame1;
		camera2 >> frame2;
		
		imshow("display1", frame1);
		imshow("display2", frame2);

		waitKey(3);
	}
}
