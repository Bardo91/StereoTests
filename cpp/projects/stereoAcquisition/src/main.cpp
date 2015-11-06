///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-10-02
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <opencv2/opencv.hpp>
#include <windows.h>
using namespace cv;


Mat frame1, frame2;
unsigned index = 0;
bool run = true;
std::string path = "C:/Users/GRVC/Desktop/Training/crawler/";

void mouseCallback(int event, int x, int y, int, void*) {
	if (event == EVENT_RBUTTONDOWN) {
		run = false;

		return;
	}
	else if (event == EVENT_LBUTTONDOWN) {
		imwrite(path + "img_cam1_" + std::to_string(index) + ".jpg", frame1);
		imwrite(path + "img_cam2_" + std::to_string(index) + ".jpg", frame2);
		index++;
		return;
	}

}

//---------------------------------------------------------------------------------------------------------------------
int main(int _argc, char ** _argv) {
	VideoCapture camera1(0);
	VideoCapture camera2(1);

	CreateDirectory(path.c_str(), NULL);
	namedWindow("display1");
	setMouseCallback("display1", mouseCallback);

	while (run) {
		camera1 >> frame1;
		camera2 >> frame2;
		/*
		imwrite("C:/Users/GRVC/Desktop/Training/img_cam1_" + std::to_string(index) + ".jpg", frame1);
		imwrite("C:/Users/GRVC/Desktop/Training/img_cam2_" + std::to_string(index) + ".jpg", frame2);*/

		imshow("display1", frame1);
		imshow("display2", frame2);

		waitKey(3);
		/*index++;*/
	}
}
