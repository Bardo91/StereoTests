///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-10-02
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <opencv2/opencv.hpp>

#ifdef _WIN32
	#include <Windows.h>
	inline void do_mkdir(std::string _filename) {
		CreateDirectory(_filename.c_str(), NULL);
	}
#elif __linux__
	#include <sys/stat.h>
	#include <sys/types.h>
	inline void do_mkdir(std::string _filename) {
		mkdir(_filename.c_str(), 0700);
	}
#endif

using namespace cv;


Mat frame1, frame2;
unsigned indexImages = 0;
bool run = true;
std::string path = "C:/Users/GRVC/Desktop/Training/crawler/";

void mouseCallback(int event, int x, int y, int, void*) {
	if (event == EVENT_RBUTTONDOWN) {
		run = false;

		return;
	}
	else if (event == EVENT_LBUTTONDOWN) {
		imwrite(path + "img_cam1_" + std::to_string(indexImages) + ".jpg", frame1);
		imwrite(path + "img_cam2_" + std::to_string(indexImages) + ".jpg", frame2);
		indexImages++;
		return;
	}

}

//---------------------------------------------------------------------------------------------------------------------
int main(int _argc, char ** _argv) {
	VideoCapture camera1(0);
	VideoCapture camera2(1);

	do_mkdir(path.c_str());
	namedWindow("display1");
	setMouseCallback("display1", mouseCallback);

	while (run) {
		camera1 >> frame1;
		camera2 >> frame2;

		imshow("display1", frame1);
		imshow("display2", frame2);

		waitKey(3);
	}
}
